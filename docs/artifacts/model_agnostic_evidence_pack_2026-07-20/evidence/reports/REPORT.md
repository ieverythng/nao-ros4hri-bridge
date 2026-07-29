# CRITIC Seam Hardening, 2026-07-17

## Target contract

Close four failures from the 8.8 CRITIC round without adding normal-path
fallback wording: correlate injected failures from structured traces, keep
execution acknowledgement consistent with the final route, distinguish robot
return from object delivery, and reject plans that omit requested actions.

## Baseline evidence

- The prior harness could observe `fail_once` in container logs but could not
  correlate it to the scored goal.
- A dialogue response could be repaired to execution while retaining a
  capability-style acknowledgement.
- Planner recovery could admit navigation and reporting after dropping posture,
  look, or gesture actions.
- `return to ALEX` could enter the delivery interpretation used by `return it to
  ALEX`.

## Accepted changes

- The questionnaire now correlates JSONL events through dialogue turn, goal,
  plan, version, step, fake mode, terminal outcome, and speech. Incomplete
  lineage and absent required fixtures are `not_scored`.
- Response-first route repair performs one locked chatbot wording retry. A
  conflicting or non-committing result cannot execute. The lock treats the
  selected route as authoritative and supplies an open execution goal without
  mislabeling it as a fallback intent.
- Ordered `intent_sequence` is authoritative. Summary intent fields no longer
  append a duplicate or contradictory action.
- `return to PERSON` creates a visit selection when the person is grounded;
  `return it to PERSON` remains delivery.
- Planner admission checks ordered capability coverage, including selected
  look targets, before accepting model or deterministic recovery plans.

## Runtime evidence

Candidate image: `iiia:nao-runtime-v26-critic-hardening`

Image ID: `sha256:70f3875ff0116e44290d2edf71cc1fb8e09efd21ce4d6af2e8d411b7770255e7`

Parent HEAD observed at final capture: `71c3aead31b7d873079a1a2d4649ab0cefa9a398`

Nested chatbot HEAD: `96ecc34374795c3ba2c25e1f55e99851cd134fe4`

| Probe | Result | Evidence |
| --- | --- | --- |
| Polite head execution | Pass | Natural commitment, five motions, report, complete lineage |
| Explicit head execution | Pass | Natural commitment, five motions, report, complete lineage |
| Walk, pick, sit, report | Pass | `navigate_to`, `pick_object`, `perform_motion`, `report_result` |
| Robot return to ALEX | Not scored | Required grounded ALEX entity absent; clarification was truthful |
| All objects, look, wave, sit | Fail | Complete fixture, planner rejected incomplete action coverage |

Raw artifacts are stored in this directory. The structured replay also
correlated the prior `fail_once` navigation case to two plan versions and a
terminal recovery.

## Validation

- `./scripts/run_tests.sh`: 547 tests and launch smoke passed.
- Harness tests: 64 passed.
- `git diff --check`: passed for parent and nested chatbot repositories.
- ROS4HRI change audit: no ownership violation reported.
- Both vLLM preflights passed on the first attempt for every clean launch.

## Decision

Accept the harness correlation, route/ack atomicity, ordered-intent ownership,
and planner capability admission changes. Do not promote v26 to the frozen
canonical image yet. The all-objects objective remains a real model-plan
failure, and the isolated robot-return case requires a harness fixture-readiness
gate before it can be scored.

## Next probe

Wait for canonical entity IDs to appear in the planner-visible grounded
projection before injecting the robot-return case. Then rerun robot return and
the all-objects objective. The latter must execute stand, each visit/look pair,
wave, sit, and one final report without deterministic plan synthesis.

## Compound-Plan Closure Addendum

The all-objects objective is closed on the clean
`iiia:nao-runtime-v28-intent-timeout` overlay
(`sha256:76fe210c293499f2cf22a0524e2aebdcc8e7e8e9a45fea3960d7da75ed4425b8`).
The v27 replacement-backend probe exposed a launch projection gap: the normal
and first-response timeouts were configurable, but intent extraction retained
the nested 10-second default. That timeout caused the earlier six-intent request
to collapse to `wave_greet` before planning.

V28 exposed the existing intent timeout parameter and ran it at 180 seconds for
this slower backend. Both model preflights passed on attempt one. The exact case
preserved stand, three navigate/look pairs, wave, sit, and `report_result`.
The first plan misplaced `report_policy=final` into the fake-only
`wave_greet.result_mode` argument, which the fake skill rejected. Planner
version 2 retained the full objective and completed it. The final chatbot report
named ATLAS, MIDAS, TITAS, and ALEX. The explicit head-motion route/ack control
also passed.

This closes the compound planner-recovery seam. The replacement was a temporary
Codex-backed Ollama-compatible adapter because the prescribed Qwen cloud tags
returned HTTP 410 and the tested local Qwen models failed the real planner
contract or exceeded the scoring window. It is runtime acceptance evidence for
the stack, not qualification of the canonical Qwen/Ollama production backend.
Planner admission still checks required arguments without rejecting unsupported
optional arguments. That residual deserves a separate registry-validation
audit; it did not prevent truthful bounded recovery in this case.

Evidence:

- `extreme_all_objects_v27_codex_gpt54.json`: failed before planning due to the
  10-second intent timeout.
- `extreme_all_objects_v28_codex_gpt54_timeout180.json`: pass.
- `route_ack_explicit_control_v28_codex_gpt54.json`: pass.

## Production backend ablation

The canonical vLLM endpoint was probed before the ablation and remained
unreachable (`HTTP 000`, connection refused) at `10.7.138.215:8004`. The
following runs used the host Ollama daemon through the real `/api/chat`
transport, the same v28 image, the same canonical launch profile, the same
180-second timeout policy, and the same maximal fixture. No temporary adapter
was used for these runs.

| Backend | Startup | Maximal case | Adjudicated finding |
| --- | --- | --- | --- |
| `nemotron-3-super:cloud` | Pass, both preflights on attempt 1 | Fail | Chatbot preserved all six intents and target selection. The planner produced no admissible plan after its bounded validation retries and emitted `explain_failure`; no execution was admitted. |
| `gemma4:31b-cloud` | Pass, both preflights on attempt 1 | Fail | Intent output was malformed JSON. After the bounded retry, only `wave_greet` was handed to the planner and `target_selection` was lost, despite a complete spoken acknowledgement. |
| `gemma4:cloud` | Pass, both preflights on attempt 1 | Fail | The response lacked a safe acknowledgement and intent JSON was invalid. Route repair plus the rules retry reduced the request to `navigate_to`; planner recovery executed navigation and reporting while dropping posture, look, wave, and sit. |
| `nemotron-3-ultra:cloud` | Not qualified | Not run | Ollama `/api/chat` returned HTTP 400 `unexpected EOF`; `ollama run` also reported a missing cloud-stub manifest despite exiting successfully. |

The full-stack artifacts are:

- `extreme_all_objects_ollama_nemotron_super.json`
- `extreme_all_objects_ollama_gemma4_31b.json`
- `extreme_all_objects_ollama_gemma4.json`

These results support a capability-sensitive conclusion. Nemotron preserved
the upstream semantic request but could not satisfy the planner JSON contract.
Gemma models reached later chatbot seams but produced malformed structured
outputs and exposed the existing lossy route-repair/recovery behavior. The
ablation therefore does not justify adding another normal-path fallback. It
does justify keeping the maximal case as a backend qualification gate and
keeping route/ack atomicity, planner coverage, and target-selection preservation
as explicit CRITIC checks.
