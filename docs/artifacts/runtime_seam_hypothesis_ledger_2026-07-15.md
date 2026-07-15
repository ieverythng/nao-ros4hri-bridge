# Runtime Seam Hypothesis Ledger, 15 July 2026

## Target contract

The chatbot must turn the current utterance and grounded context into one explicit route and, for execution, a complete semantic target selection. The planner may plan only admitted work. The orchestrator validates and dispatches without inventing targets. AB=1 results and KB postconditions are execution evidence. User-facing completion must be truthful and spoken once through the dialogue owner.

Protected owners are `chatbot_llm` for route and semantic handoff, `planner_llm` for plan/replan policy, `nao_orchestrator` for deterministic admission and dispatch, `kb_skills` for KB transport, and `dialogue_manager` for speech. No prompt edits, keyword route guards, new ROS interfaces, or executor-side target invention are in scope.

Acceptance requires: non-empty exact selected members for quantified execution; object, source, recipient, and support roles consistent with grounded entity kinds; no skill dispatch on an incomplete selection; Contract 6 outcome coverage; verified KB effects; exactly-once natural closure; and a clean full main plus deep-fake run on one provider/source tuple.

## Baseline evidence

- Running image: `iiia:nao-deslop-20260714-v8-kb-spatial`, image id `sha256:b02fe04c12b905296cb621b38b4ffe8f6cecee9d774599826c0aafc46266850b`.
- Both LLMs used Ollama `qwen3-coder:480b-cloud`; turn pipeline was `response_first`.
- Runtime source hashes matched the host changes under review for all principal planner, orchestrator, report, and chatbot seams.
- Valid positive evidence includes relation revision, exact TITAS/MIDAS delivery selection, direct KB postconditions, environment fixtures, head-and-wave, look-at/report, and navigation/report.
- Counterexample A: `visible_objects` delivery carried an empty member set and never executed.
- Counterexample B: “bring me the cup” selected the kitchen as recipient and falsely reported delivery to the kitchen.
- External cutoff: explicit Ollama HTTP 429 at 03:01:31 CEST. Later model-dependent cases and deep fake are blocked.

## Approach registry

| ID | Mechanism | Discriminating probe | Evidence | Status | Code/node target |
|---|---|---|---|---|---|
| H1 | Grounded facts were absent or stale | Compare KB query, grounded trace, selection, and post-query | TITAS/MIDAS revisions and postconditions were present | Rejected for the two critical counterexamples | `nao_scene_grounding`, grounded renderer |
| H2 | Chatbot emitted a structurally present but semantically invalid selection | Inspect `user_intent.target_selection` before adapter salvage | Empty member set and kitchen recipient were already in the handoff | Supported | `chatbot_llm/planner_request_adapter.py`, intent result validation |
| H3 | Planner admission treats non-empty dictionaries as complete selections | Submit operation-specific empty members and contradictory recipient roles | `_request_requires_target_selection` checks selection presence; `_target_selection_decision` returns `None`, then ordinary model planning continues | Supported | `planner_llm/planner_engine.py` |
| H4 | Executor or report code changed the correct recipient | Compare admitted selection, plan step args, result, and report | The admitted recipient was already `codex_kitchen` | Rejected as root cause | `nao_orchestrator`, `report_outcome.py` |
| H5 | Long-context model drift caused environment misses | Isolated rerun of grouped environment case on same tuple | Adjacent cases passed; isolated rerun unavailable after quota | Blocked | chatbot history and response/intent stages |
| H6 | Provider exhaustion caused late failures | Inspect transport status and first-error timestamp | Explicit repeated HTTP 429 session-limit errors | Accepted for post-cutoff cases only | `ollama_transport.py`, provider account |
| H7 | Harness or trace correlation produced false negatives | Compare raw robot output, voice-correlated log, target oracle, and timeout | Raw speech existed without voice id; empty target set passed old composite oracle; main timed out at 420 s | Accepted | questionnaire runner and snapshot collector |

## Discriminating probes and results

1. **KB state versus wording:** direct `/kb/query` postconditions passed for both deliveries. This rejects missing post-effects and isolates one immediate-answer omission to response generation or capture.
2. **Role lineage:** the kitchen identifier appears as `recipient_id` before planning and skill dispatch. This locates the first proven semantic error at the chatbot handoff/admission boundary.
3. **Empty selection:** the handoff was a non-empty dictionary with `member_ids=[]`. Current planner completeness logic distinguishes missing selection from present selection, but not present-invalid selection.
4. **Provider boundary:** pre-limit successes and counterexamples precede the first 429. `language_model_unreachable_speech` after 03:01:31 is provider failure, not stack semantics.
5. **Speech correlation:** exact robot wording was visible in raw logs where the case-scoped voice filter returned no payload. The harness was corrected and covered by tests.

## Adversarial audit

- Ownership remains intact. The proposed next checks validate structured handoff and admission; they do not move policy into the orchestrator or make the executor infer targets.
- No new topic, service, action, registry alias, or AB level is proposed.
- Planning-time context is not treated as proof. The KB stress suite queries declared postconditions after skill execution.
- People, objects, supports, and locations remain separate roles. The kitchen-recipient counterexample demonstrates why kind validity alone is insufficient for semantic role consistency.
- No prompt change was made, so no SkillOpt mutation ledger is required for this pass.
- No current-tuple claim is made for deep-fake recovery because the provider cutoff prevented it.

## Decision: bounded handoff

Accept the KB stress harness and oracle corrections. Reject claims that the complete stack passed this run. The strongest supported diagnosis is that semantically invalid target selections can cross the chatbot handoff and are not rejected as incomplete before planner fallback. Provider exhaustion explains later unreachable-model cases but does not explain the two pre-limit semantic counterexamples.

The smallest safe implementation route is an operation-aware target-selection validator shared at the contract/admission boundary. It should reject empty member sets for delivery/visit, require a grounded recipient for delivery, preserve role kinds, and return structured clarification or response retry. It must not infer targets from keywords or fabricate a recipient.

## Residual risk and next probe

- The exact origin of the kitchen recipient inside response versus intent-stage output is not preserved in the current concise trace. Reopen H2 with a trace that records the validated intent-stage selection before adapter normalization.
- The environment grouped-location miss may be long-context drift or ingress/correlation loss. Re-run it in isolation and in its original conversation group after quota reset.
- Run the complete deep-fake matrix on the same frozen tuple. Current historical fake evidence cannot substitute for this run.
- Add adversarial tests for an empty visible-object set, absent operator recipient, source-equals-recipient, location-as-recipient when the request denotes a person, and exact post-report coverage.
