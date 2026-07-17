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
| H8 | Plan `scene_targets` inherited detector history instead of admitted execution scope | Compare request targets, normalized plan steps, and emitted plan context | A stand-and-wave run carried many anonymous people although the executable wave step referenced one | Accepted; source fix gated | `planner_llm/planner_engine.py`, `chatbot_llm/planner_handoff.py` |
| H9 | Disconnected posture could not enter open loop after the real action accepted and failed | Compare action acceptance, NAOqi connection state, and fallback eligibility | `standinit` was accepted by replay motion, then failed; orchestrator fallback only applies to rejected actions | Accepted; source fix gated | `nao_replay_motion/replay_motion_skill_server.py` |

## Discriminating probes and results

1. **KB state versus wording:** direct `/kb/query` postconditions passed for both deliveries. This rejects missing post-effects and isolates one immediate-answer omission to response generation or capture.
2. **Role lineage:** the kitchen identifier appears as `recipient_id` before planning and skill dispatch. This locates the first proven semantic error at the chatbot handoff/admission boundary.
3. **Empty selection:** the handoff was a non-empty dictionary with `member_ids=[]`. Current planner completeness logic distinguishes missing selection from present selection, but not present-invalid selection.
4. **Provider boundary:** pre-limit successes and counterexamples precede the first 429. `language_model_unreachable_speech` after 03:01:31 is provider failure, not stack semantics.
5. **Speech correlation:** exact robot wording was visible in raw logs where the case-scoped voice filter returned no payload. The harness was corrected and covered by tests.
6. **Shared admission:** one `planner_common` validator now rejects empty members, unknown visit targets, non-object delivery members, and non-person or self-overlapping recipients. Chatbot and planner consume the same contract; plan coverage validation no longer carries a second recipient policy.
7. **Spatial replacement ownership:** fake delivery now reports only the achieved `isAt` relation and released hold. The orchestrator queries, retracts the complete stale spatial family, applies the new relation, and verifies the postcondition.
8. **Execution-bounded target scope:** when model output omits plan-level targets, planner output now derives them from admitted executable step arguments before considering request targets. Scene projection also removes positively timestamped person identities older than five seconds relative to the newest observation.
9. **Disconnected posture:** replay motion retains direct NAOqi execution as its first route. An explicit launch parameter permits `open_loop` completion only when NAOqi is unavailable; it does not replace successful real execution.

## Adversarial audit

- Ownership remains intact. The proposed next checks validate structured handoff and admission; they do not move policy into the orchestrator or make the executor infer targets.
- No new topic, service, action, registry alias, or AB level is proposed.
- Planning-time context is not treated as proof. The KB stress suite queries declared postconditions after skill execution.
- People, objects, supports, and locations remain separate roles. The kitchen-recipient counterexample demonstrates why kind validity alone is insufficient for semantic role consistency.
- No prompt change was made, so no SkillOpt mutation ledger is required for this pass.
- No current-tuple claim is made for deep-fake recovery because the provider cutoff prevented it.
- The new posture mode is explicit in the NAO adapter and launch profile. It does not move motion execution into planner or chatbot code, and its result identifies `open_loop` rather than claiming physical convergence.

## Decision: bounded handoff

Accept the KB stress harness and oracle corrections. Reject claims that the complete stack passed this run. The strongest supported diagnosis is that semantically invalid target selections can cross the chatbot handoff and are not rejected as incomplete before planner fallback. Provider exhaustion explains later unreachable-model cases but does not explain the two pre-limit semantic counterexamples.

The smallest safe implementation route is an operation-aware target-selection validator shared at the contract/admission boundary. It should reject empty member sets for delivery/visit, require a grounded recipient for delivery, preserve role kinds, and return structured clarification or response retry. It must not infer targets from keywords or fabricate a recipient.

That implementation is accepted at source-test level. The same pass also removed duplicate fake-skill spatial cleanup, bounded plan target scope, filtered stale person projections, and added explicit disconnected posture open loop. Focused and package tests pass. Runtime acceptance remains pending a clean versioned overlay; the running container was not modified during this source pass.

The clean candidate overlay is `iiia:nao-deslop-20260715-v10-contract-scope-detector` (`sha256:962ab5d6e8f4fa4f99d4b9fad903c4785a9833547fac6f02b441e23f86da26f8`). Its source hashes match the staged host files, and `planner_common`, `chatbot_llm`, `nao_replay_motion`, `emorobcare_cv_msgs`, `emorobcare_cv_object_detection`, and `my_game_interface` resolve from `/home/ubuntu/ws/install`. The existing live container remained on `iiia:nao` during the build and verification.

## Residual risk and next probe

- The exact origin of the kitchen recipient inside response versus intent-stage output is not preserved in the current concise trace. Reopen H2 with a trace that records the validated intent-stage selection before adapter normalization.
- The environment grouped-location miss may be long-context drift or ingress/correlation loss. Re-run it in isolation and in its original conversation group after quota reset.
- Run the complete deep-fake matrix on the same frozen tuple. Current historical fake evidence cannot substitute for this run.
- Add adversarial tests for an empty visible-object set, absent operator recipient, source-equals-recipient, location-as-recipient when the request denotes a person, and exact post-report coverage.
- Confirm in the rebuilt runtime that repeated detector IDs do not inflate planner `scene_targets`, and that a disconnected `standinit` reports `open_loop` success while a connected robot still uses `direct_naoqi`.
