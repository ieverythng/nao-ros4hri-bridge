---
name: robot-stack-iteration-loop
description: Run a bounded, evidence-gated iteration loop over the live NAO ROS4HRI stack. Use for cross-package runtime regressions, repeated CRITIC failures, fake-skill and replanning hardening, prompt-versus-contract uncertainty, or supervisor-demo stabilization that requires diagnosis, TDD, clean container rebuilds, adversarial runtime review, and documented acceptance.
---

# Robot Stack Iteration Loop

Coordinate existing specialist skills into one traceable loop. Deepen the
smallest owning seam, preserve ROS ownership, and separate semantic failures
from harness or observability failures.

## 1. Freeze The Round

Read the repository guidance, active contracts, launch profile, issue trackers,
and latest runtime artifact. Record:

- target behavior and exact failing trajectory;
- protected owners and non-goals;
- source and live acceptance gates;
- evidence budget and maximum iteration count;
- current base image, active overlay, source hashes, and dirty repositories.

Use `$seam-hypothesis-audit` when two or more materially different causes remain
plausible. Use `$diagnosing-bugs` for a difficult but locally owned failure.
Completion criterion: one discriminating next probe exists for every live
hypothesis; unsupported explanations are marked blocked or rejected.

## 2. Establish A Red Gate

Use `$tdd` to reproduce the failure at the narrowest truthful layer. Prefer a
contract or pure-helper test before node-level integration. Preserve one control
that protects the known-good path.

Classify the finding before editing:

- **contract loss**: structured evidence is absent or malformed;
- **ownership mismatch**: behavior lives in the wrong package;
- **model behavior**: complete structured input reaches the model but output is
  still wrong;
- **runtime wiring**: source is correct but launch, QoS, lifecycle, or installed
  code differs;
- **harness/observability**: the stack cannot be scored from correlated evidence.

Completion criterion: the new test fails for the observed reason, or the runtime
probe proves that source-only reproduction is insufficient.

## 3. Patch The Owning Seam

Apply the smallest structural change that resolves the frozen cause. Run
`$iiia-ros4hri-check` before and after the edit.

- Structure evidence and normalize contracts deterministically.
- Let `chatbot_llm` remain the normal wording authority.
- Let `planner_llm` plan and supervise; let `nao_orchestrator` gate, execute, and
  apply KB effects.
- Prefer registry metadata and typed payloads over lexical recognizers.
- Do not add output sanitizers or sentence-building fallbacks as normal policy.
- Preserve unrelated dirty work and nested repositories.

For prompt or LLM-facing addendum changes, stop this branch and invoke
`$skillopt-skill-iteration`. Require baseline, bounded mutation, focused train
cases, protected holdouts, and an accept/reject ledger before keeping the edit.

Completion criterion: the red test passes, protected controls pass, and the
change audit reports no ownership violation.

## 4. Widen Source Validation

Run focused tests first, then affected package suites, compilation, registry
consistency where relevant, and:

```bash
python3 scripts/ros4hri_change_audit.py --mode working
git diff --check
```

Use `$deslop-refactor` only after behavior is green. Limit it to touched hot
paths, remove duplication introduced by the round, then rerun the same tests.

Completion criterion: every affected source gate passes and no prompt mutation
exists without a SkillOpt ledger.

## 5. Replace The Runtime Cleanly

Follow `$robot-runtime-performance-review` for exact build and launch commands.
When source changed:

1. Build one new overlay from the intended `iiia:nao` base.
2. Stop the full stack and container.
3. Start one fresh `nao_ros2` container with the required display, XDG, camera,
   network, model, and validation flags.
4. Launch the full profile once.
5. Verify lifecycle state, parameters, source hashes, and node uniqueness.
6. Delete the superseded overlay after the replacement is healthy.

Never hot-copy source, restart one node, or retain a chain of disposable images
for scored evidence. Keep the base and one active overlay unless the user asks
to preserve another image. Preserve traceability through JSON artifacts and
hashes rather than image accumulation.

Completion criterion: the running image contains the tested source, all required
nodes appear once, and the active parameters match the frozen round.

## 6. Run The Runtime Ladder

Invoke `$robot-runtime-performance-review` and run sequentially:

1. **Discriminating probe**: exact former failure plus one known-good control.
2. **Focused matrix**: success, clarification, truthful failure, and recovery.
3. **Adversarial variants**: paraphrase, multi-turn detour, role exclusion,
   changed KB state, and one harder failure mode.
4. **Full suite**: only after focused probes are stable.

Correlate by turn/voice, goal, plan lineage, and timestamps. Require closure
speech after the relevant execution event. A complete-context clarification is
a semantic failure; missing correlated evidence is `not_scored`. Harness defects
must not reduce the robot score.

Track at least:

- semantic case status and report naturalness;
- planner versions and recovery outcome;
- duplicate goals or utterances;
- invalid JSON/plan events;
- report, rules, route-repair, and transport fallback rates;
- KB precondition, post-effect, and stale-relation verification;
- latency by response, plan, execution, and closure phase.

Completion criterion: focused controls pass, the failure is live-proven closed,
and new variants either pass or produce a bounded evidence-backed finding.

## 7. Decide And Record

Accept a round only when source and live evidence agree. Otherwise revert the
round's hypothesis-specific edit or keep it explicitly staged as unaccepted.
Update the active Markdown/HTML pair, runtime trackers, and masterplan with:

- hypothesis and discriminating evidence;
- source and image hashes;
- exact tests and artifacts;
- semantic score separated from observability;
- residual risks and the next highest-ROI probe.

Do not stage, commit, or push unless the user asks. Do not claim completion from
source tests when live validation is required.

## Stop Conditions

Stop iterating when the frozen acceptance gate passes, the configured iteration
budget is exhausted, or an external dependency prevents a discriminating probe.
Report external blockers as `not_scored` with a reopen condition. Do not convert
an unavailable endpoint, stale container, failed harness, or absent trace into a
semantic regression.
