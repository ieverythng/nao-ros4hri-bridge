# Runtime Review: Ollama Qwen3-Coder, 15 July 2026

**Standing:** blocked after useful pre-limit evidence. The runtime was healthy at startup and several high-value KB and composite cases passed. Ollama then returned explicit HTTP 429 session-limit errors, so later cases and the unstarted deep-fake matrix are not semantic evidence.

**Score: 6.0/10 (critical-failure cap).** The uncapped pre-limit behaviour was materially stronger, but one complete-context delivery selected no objects and one underspecified kitchen request executed with the kitchen as recipient, then reported success. The review rubric caps false execution success at 6/10.

## 🔴 Critical

1. **Wrong recipient and false completion.** `maximal_kitchen_cup_to_operator` asked to bring the cup to the user. The structured selection used `recipient_id=codex_kitchen`, `bring_object` succeeded, and the robot said, “I brought the cup to the kitchen.” This is not a provider-limit artifact; it occurred before the first 429. The next code probe belongs at chatbot target-role validation and planner admission, not in a phrase list.
2. **Empty quantified selection reached planner handoff.** `composite_bring_every_object_to_person` routed to execution with `selection_kind=visible_objects`, `member_ids=[]`, and recipient ALEX. No skill feedback followed. A non-empty target-selection object currently bypasses the planner's missing-selection check even when its member set is empty.

## 🟠 Grounding and KB

1. **The hardened KB chain established the important positive result.** The fixture inserted TITAS, MIDAS, two supports, and ALEX. A relation revision moved TITAS from the work table to the storage shelf and was visible in the next answer. Both delivery cases selected the expected object and recipient, and direct postcondition queries observed `oro:isAt codex_stress_alex`.
2. **Immediate revision wording was incomplete once.** The robot answered “Let me check...” instead of stating both revised locations. The next turn correctly reported TITAS on the storage shelf and MIDAS on the work table. This is a response-completion issue, not missing KB state.
3. **Detector/profile drift was present.** Detector-authored objects and people appeared although this review intended to isolate the semantic cool profile. Those extra objects made superset-based visible-object checks less discriminating. Detector results are excluded from this score.

## 🟡 Route and semantic admission

1. Dialogue, name/color/location queries, simple motion, head-and-wave composition, look-at/report, navigation/report, grouped environment delivery, and a gold-apple handoff all produced useful pre-limit evidence.
2. The environment grouped-location case did not produce a planner request in one long-context run, while adjacent environment and follow-up cases passed. It remains unresolved because an isolated rerun was prevented by the provider limit.

## 🔵 Runtime and provider pressure

1. The first explicit Ollama 429 occurred at **2026-07-15 03:01:31 CEST**. The response body stated that the account had reached its session usage limit. Repeated 429 responses followed for `qwen3-coder:480b-cloud`.
2. Cases carrying `language_model_unreachable_speech` after that cutoff are **not scored**. The current-tuple deep-fake suite was not run.
3. The final snapshot marked lifecycle nodes non-active because ANSI transport warnings preceded the literal `active [3]` text. The nodes were active; this is a snapshot-parser defect and not a lifecycle failure.

## 🟣 Observability and harness findings

1. Robot speech can be present in `robot_speech_debug` without a voice identifier. Earlier KB-stress artifacts incorrectly removed it during voice correlation. The harness now extracts robot output from the raw case log.
2. The formal main suite used a seven-minute global timeout and stopped after 14 cases. The default is now 1,200 seconds.
3. `composite_walk_every_object_reports` could pass without a target-selection oracle. It now requires the three fixture identifiers and per-target reporting.

## Checks passed

- Source provenance matched the running image for the planner, supervisor, contracts, report-outcome, orchestrator, chatbot turn engine, planner-request adapter, and grounding renderer.
- KnowledgeCore query and revise services were available before fixture injection.
- Both LLM preflights passed before the quota cutoff.
- KB insertion, relation retraction/update, next-turn grounded use, two exact delivery selections, and two direct postcondition queries were observed.
- No duplicate semantic speech, raw planner leakage, planner-invalid JSON, or invalid executable-plan event was observed in the selected pre-limit evidence.
- Questionnaire unit tests: `46 passed`.

## E2E questionnaire

- Simple dialogue: **pass before provider cutoff**
- KB query dialogue: **pass with one incomplete immediate revision answer**
- Simple skill execution: **pass in sampled cases**
- Composite skill execution: **fail under semantic oracle**
- Environment fixtures: **degraded, 9 pass / 1 degraded / 1 unresolved fail**
- KB mutation and postcondition stress: **pass at state depth; speech observability qualified**
- Simple fake-skill scenarios: **not run on this tuple**
- Composite/deep fake-skill scenarios: **not run on this tuple due HTTP 429**

## Artifacts

- `/tmp/nao_main_questionnaire_20260715_winner.json`
- `/tmp/nao_main_questionnaire_20260715_winner_remaining.json`
- `/tmp/nao_environment_20260715_winner.json`
- `/tmp/nao_kb_stress_20260715_winner_valid.json`
- `/tmp/nao_composite_20260715_winner.json`
- `/tmp/nao_runtime_snapshot_20260715_winner_final.json`

## Next probes

1. After the Ollama limit resets, run the fixed main suite once, then the complete `fake_deep` policy matrix without runtime edits.
2. Reject or repair target selections whose operation requires members but whose `member_ids` is empty before planner admission.
3. Add a role-consistency admission test in which source, recipient, object, and operator are distinct grounded roles; reject a destination that contradicts the structured request.
4. Correct ANSI stripping in runtime-snapshot lifecycle parsing.

**Summary:** 7 findings: 🔴 2, 🟠 3, 🟡 1, 🔵 3, 🟣 3. The counts overlap where one observation affects more than one seam.
