# Runtime Review: Lab Endpoint, Adjudicated Evidence

Date: 15 July 2026

Runtime tuple: `iiia:nao`, response-first, JSON-only grounded context, chatbot and planner model `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` through `http://10.7.138.215:8004`.

## Standing

**Degraded but operational. Score: 7.0/10.** The score is capped by a user-visible stale post-delivery location answer. No duplicate speech, raw planner leakage, invalid planner JSON, invalid executable plan, model-connectivity failure, or false action-success report was observed in the accepted run windows.

The questionnaire's raw pass/fail values are not used as the final score. Manual adjudication compared fixture readiness, raw robot speech, grounded context, target selection, planner lineage, fake-policy consumption, AB=1 feedback, terminal state, and direct KB postconditions. Cases whose configured fake failure never reached an applicable skill are marked not scored.

## 🔴 Critical

No false action-success or duplicate-speech event was established in this tuple. The prior kitchen-as-recipient false completion was not reproduced. The current underspecified kitchen case ended with a safe request for the cup and destination.

## 🟠 Grounding and KB instability

1. **Post-delivery source relations remain user-visible.** Direct queries proved that TITAS and MIDAS gained `oro:isAt codex_stress_alex`, but the final mixed query placed both back on `work_table`. The fake post-effect adds the recipient relation without retiring or subordinating the old support relation. Target: fake skill KB-effect contract, `planner_common/report_outcome.py`, and grounded relation precedence.
2. **Grouped target admission is inconsistent despite complete fixtures.** Work-table and IIIA kitchen fixtures were present in grounded context, but response repair produced execution at confidence `0.00` with empty `target_selection`; the planner then truthfully asked for a complete selection. Target: validated intent-stage target selection and planner admission, not the fake server.

## 🟡 Route and plan admission

1. Exact ordered navigation passed in all-success and recovered under fail-once navigation. The recovery trace contained the exact four members, a failed navigation event, replan evidence, terminal completion, and reports covering all members.
2. The natural-language grouped-delivery wording remains the dominant failure family. It prevented `delivery_blocked` and parts of `fail_once_pick` from reaching the configured fake policy.
3. `fail_once_pick` is not exercised by tasks that dispatch composite `bring_object` without a distinct `pick_object` event. This is a validation-profile applicability gap, not successful pick recovery.

## 🔵 Runtime and performance

- Startup preflight passed with all required lifecycle nodes active, KnowledgeCore query/revise available, and both LLM preflights succeeding on the first attempt.
- Object detection was enabled in this profile. Face detection repeatedly skipped approximately 100 frames per five to six seconds, and transient person identifiers churned. Detector pressure is reported separately from stable fixture semantics.
- The final snapshot incorrectly marked KnowledgeCore unready because its readiness detector searched the current log window. Direct `/kb/query` succeeded during the same window.

## 🟣 Harness validity findings

1. `phase_observations` can report `route_observed=false` and `planner_request_observed=false` while the same case contains planner feedback, exact target selection, terminal completion, and speech. The log-window/correlation oracle is therefore not internally consistent.
2. Grounded speech terms compare literal spaces. Correct answers containing `work_table` or `storage_shelf` fail expectations written as `work table` and `storage shelf`.
3. `finished_at_unix_sec` is written in incremental artifacts while the questionnaire process is still running. Process completion, not that field, must freeze the artifact.
4. Case excerpts can include older goal lines. Evidence must be correlated by current `turn_id`, `goal_id`, `plan_id`, and `plan_version` before extracting failure markers.
5. A fake profile can be reported as failed even when its configured failure was inapplicable to the dispatched skill. These cases must be not scored.
6. Reusing group voice identifiers across policy runs preserves dialogue state unless the complete stack is restarted. Scored policy profiles require a fresh graph or run-specific voice namespace.

## Adjudicated results

| Evidence set | Raw result | Adjudicated result | Evidence basis |
|---|---:|---:|---|
| Deep fake, all success | 7 pass / 2 fail | 7 pass / 2 fail | Two complete-fixture grouped deliveries lacked target selection and clarified. |
| Deep fake, fail-once navigation | 5 pass / 1 degraded / 1 fail / 2 not scored | 5 pass / 1 degraded / 3 not scored | Ordered walk recovered successfully; its raw fail was missing correlation evidence. |
| Fail-once pick, targeted | 2 not scored | 2 not scored | No distinct pick failure was consumed. |
| Delivery blocked, targeted | 2 not scored | 2 not scored | Admission failed before fake dispatch. |
| Recipient missing, targeted | 1 pass / 1 not scored | 1 pass / 1 not scored | Missing recipient produced a concise clarification. |
| Stateful KB stress | 3 pass / 4 fail | 6 pass / 1 fail | Three failures were underscore-format mismatches; final location answer was stale. |
| Hard main subset | 0 pass / 6 fail | 3 pass / 2 degraded / 1 fail | Mutation, wave, and safe ambiguity passed; head closure and per-target reporting degraded; grouped delivery failed admission. |

Across adjudicated, scoreable cases: **22 pass, 3 degraded, 4 fail (weighted case rate 81.0%)**. Eight policy cases are not scored because the configured failure was not exercised or target admission prevented dispatch.

## Checks passed

- Explicit KB mutation created a red Cup (`object_1`), confirmed by direct `/kb/query`.
- Five head-motion steps and `report_result` all succeeded; terminal speech capture was incomplete.
- Person navigation plus wave used the exact ALEX target and produced natural final wording.
- Ordered three-object navigation selected ATLAS, MIDAS, and VEGA and completed all navigation/report steps.
- Stateful KB insertion, support revision, exact TITAS and MIDAS selections, and two direct recipient postconditions passed.
- Missing-recipient clarification stayed truthful and did not dispatch a skill.
- Fail-once navigation produced failed-step and replan evidence, then completed the exact ordered task.

## E2E questionnaire

- Simple dialogue: **historically passed; not rerun in this targeted pass**
- KB query and mutation: **pass, with stale post-effect closure defect**
- Simple skill: **pass at execution depth**
- Composite skill: **degraded; ordered tasks pass, grouped delivery admission fails**
- Simple fake scenarios: **partially covered**
- Composite/deep fake scenarios: **degraded; all-success 7/9, navigation recovery proven, other policies partly blocked before dispatch**
- Exactly-once speech: **pass in accepted traces**
- Harness reliability: **fail; manual adjudication required**

## Next fixes and probes

1. Make questionnaire evidence extraction goal-scoped and assert internal consistency when feedback exists without route/planner markers.
2. Normalize symbolic labels for speech-term comparison without changing the LLM contract.
3. Add policy applicability metadata that identifies the skill expected to consume each fake failure.
4. Validate grouped `target_selection` before planner handoff and retry the intent stage when complete grounded fixtures exist but the selection is empty.
5. Define mutually exclusive location/support post-effects or explicit relation precedence after delivery.
6. Re-run grouped delivery, blocked delivery, and pick recovery in isolation after those corrections. Do not rerun the entire main suite first.

## Artifacts

- `/tmp/nao_fake_deep_lab_clean_all_success_20260715.json`
- `/tmp/nao_fake_deep_lab_fail_once_navigation_20260715.json`
- `/tmp/nao_fake_deep_lab_fail_once_pick_targeted_20260715.json`
- `/tmp/nao_fake_deep_lab_delivery_blocked_targeted_20260715.json`
- `/tmp/nao_fake_deep_lab_recipient_missing_targeted_20260715.json`
- `/tmp/nao_kb_stress_lab_clean_20260715.json`
- `/tmp/nao_main_hard_lab_20260715.json`
- `/tmp/nao_runtime_snapshot_lab_clean_20260715.json`
- `/tmp/nao_runtime_snapshot_lab_final_20260715.json`

Summary: 8 material findings: 🔴 0, 🟠 2, 🟡 3, 🔵 3, 🟣 6. Some observations affect more than one band.
