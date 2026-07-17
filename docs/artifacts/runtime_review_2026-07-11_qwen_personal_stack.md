# Runtime Review 2026-07-11: Qwen Personal Stack

This artifact records the live `nao_ros2` review run performed against the
user's Qwen personal stack on 11 July 2026.

## Runtime Fingerprint

- Container: `nao_ros2`
- Image tag: `iiia:nao`
- Image id: `sha256:d0dd959a870ef3fdf75da683bd535084b20e3dce29501f61b56555d71b5f9854`
- Created: `2026-07-11T15:22:51Z`
- Chatbot pipeline: `response_first`
- Grounded-context digest: `False`
- Planner provider: `openai_compatible`
- Planner model: `qwen36-turbo-hermes`
- Planner base URL: `http://10.88.140.94:4000`
- Duplicate node warning: `/interaction_trace_viewer` was duplicated.

The live seam modules matched the 10 July source fingerprints:

| Module | Live symbol | SHA-256 prefix |
| --- | --- | --- |
| `planner_common.report_outcome` | `build_report_outcome` | `e58cbb1a34bbbbd7` |
| `nao_orchestrator.kb_effects` | `remove_stale_spatial_effect_values` | `794ecb9f4b745953` |
| `chatbot_llm.response_fallbacks` | `_uses_generic_completion_for_ordered_navigation` | `c6ab126e1d9b66e0` |

## Score

**5.8 / 10, degraded relative to the previous JSON-only fake-deep baseline.**

The model gave more fluent acknowledgements and the base main questionnaire was
strong, but the deep fake/replan suite regressed and KB mutation correctness did
not improve. This stack should not be used as the supervisor-facing runtime
unless the patching thread fixes planner JSON validity, KB mutation, and
complete-context delivery behavior.

## Artifacts

| Artifact | Result |
| --- | --- |
| `/tmp/nao_qwen_runtime_snapshot_pre_20260711.json` | Pre-run snapshot |
| `/tmp/nao_qwen_architecture_sweep_20260711.json` | KB mutation failed after add; grouped delivery still stale |
| `/tmp/nao_qwen_main_full_20260711.json` | 20 pass |
| `/tmp/nao_qwen_environment_full_20260711.json` | 3 pass, 1 degraded |
| `/tmp/nao_qwen_environment_targeted_20260711.json` | 2 pass, 1 degraded |
| `/tmp/nao_qwen_fake_deep_all_success_20260711.json` | 6 pass, 1 degraded, 1 fail |
| `/tmp/nao_qwen_fake_deep_fail_once_navigation_20260711.json` | 5 pass, 1 degraded, 2 fail |
| `/tmp/nao_qwen_fake_deep_fail_once_pick_20260711.json` | 4 pass, 2 degraded, 2 fail |
| `/tmp/nao_qwen_fake_deep_delivery_blocked_20260711.json` | 4 pass, 2 degraded, 2 fail |
| `/tmp/nao_qwen_fake_deep_recipient_missing_20260711.json` | 3 pass, 1 degraded, 4 fail |
| `/tmp/nao_qwen_runtime_snapshot_post_20260711.json` | Post-run snapshot |

Overall questionnaire aggregate across produced artifacts:

- Pass: 47
- Degraded: 9
- Fail: 11

## Improvements

- The main speech questionnaire was superficially strong: 20/20 cases were
  marked pass.
- Qwen produced more natural acknowledgements in several execution cases,
  including maximal kitchen cup delivery and person-wave-report.
- `composite_head_all_directions` and several ordinary skill cases stayed out
  of the earlier response/action mismatch.
- Missing-object recovery and missing-recipient clarification passed in several
  fake-deep profiles.
- `delivery_blocked` preserved ordered-walk behavior better than the navigation
  and pick failure profiles.

## Regressions

### KB Mutation

The architecture sweep failed the structured KB mutation sequence after the
initial add:

- Add passed: `codex_arch_marker` gained `rdf:type Cube`, `dbp:name NOVA`, and
  `dbp:color green`.
- Revise failed: after "Update codex_arch_marker so its dbp:color is blue",
  `/kb/query` still showed `dbp:color green`.
- Non-mutating query failed the expected blue-color recall because the KB had
  not changed.
- Remove failed: the model said it removed the entity, but `/kb/query` still
  returned the marker facts.

Natural `kb_mutation_add_red_cup` also remained weak. The main questionnaire
marked it pass, but the log evidence showed route-repair pressure and failure
wording rather than a verified KB postcondition.

### Grouped Delivery And Spatial KB Effects

Qwen improved the surface behavior for grouped kitchen delivery: it admitted
execution and reached terminal evidence. The persisted KB state did not improve.

Direct `/kb/query` after grouped kitchen delivery showed:

- `codex_kitchen_cup` still had `isContainedIn`, `isAt`, and `isIn`
  `codex_kitchen`, while also having recipient facts.
- `codex_kitchen_book` still had stale kitchen predicates.
- `codex_recipient_person` acquired object-like spatial facts such as
  `isAt codex_kitchen_cup`, `placeOf codex_kitchen_cup`,
  `placeOf codex_kitchen_book`, and `isUnder codex_kitchen_cup`.

This confirms the 10 July post-effect bug is still present.

### Deep Fake/Replan

The previous JSON-only baseline had strong fake-deep all-success and
fail-once-navigation behavior. Qwen regressed here:

- `all_success`: 6 pass, 1 degraded, 1 fail.
- `fail_once_navigation`: 5 pass, 1 degraded, 2 fail.
- `fail_once_pick`: 4 pass, 2 degraded, 2 fail.
- `delivery_blocked`: 4 pass, 2 degraded, 2 fail.
- `recipient_missing`: 3 pass, 1 degraded, 4 fail.

Common failure shapes:

- Ordered walk/report often lacked terminal or speech evidence.
- Grouped work-table delivery failed under `fail_once_navigation` and
  `recipient_missing`.
- IIIA kitchen delivery failed under `all_success`, `fail_once_pick`, and
  `recipient_missing`.
- Gold-apple multiturn failed in every produced failure-profile artifact.
- `delivery_blocked` had a safety issue: the missing-recipient clarification
  case showed execution feedback when it should clarify first.

### Runtime Pressure

Post-run snapshot:

- `planner_invalid_json`: 9 deduplicated events
- `planner_invalid_executable_plan`: 9 deduplicated events
- `planner_gate_rejected`: 6 deduplicated events
- `route_repair`: 53 deduplicated events
- `fallback_event_total_count`: 77
- `face_skip_warning_count`: 1272
- `planner_request_count`: 449
- `report_result_count`: 821
- `knowledge_expiry_count`: 819

The invalid planner JSON and invalid executable plan counts are the main
runtime-regression signal. The base dialogue surface looked cleaner than the
failure-profile behavior.

## Comparison Against 10 July Baseline

| Seam | 10 July current image | 11 July Qwen |
| --- | --- | --- |
| Main/simple dialogue | Good enough | Stronger surface pass |
| Natural KB mutation | Failing/weak | Still failing |
| Structured KB revise/remove | Passed in architecture sweep | Failed after add |
| Grouped kitchen delivery behavior | Pass surface, stale KB | Better surface, stale KB unchanged |
| Fake-deep all-success | Focused subset healthy | Full run: 6 pass, 1 degraded, 1 fail |
| Fake-deep fail-once navigation | Focused subset passed | Full run: 5 pass, 1 degraded, 2 fail |
| Planner JSON validity | No invalid JSON in focused post snapshot | 9 deduplicated invalid JSON events |

## Patch Priorities

1. Keep the previous model or a model-specific planner prompt/schema strategy
   for scored fake-deep validation until Qwen can produce valid executable plan
   JSON consistently.
2. Add a hard postcondition gate for KB mutation questionnaire cases. Do not
   score "I will update/remove" as pass unless `/kb/query` proves the fact
   changed.
3. Patch `nao_orchestrator.kb_effects` for delivered-object stale predicate
   cleanup and recipient-side inverse pollution.
4. Add a model-ablation scoreboard to the runtime tracker so dense/agentic
   models are compared on the same seams, not on response fluency.
5. Investigate why `run_active_questionnaire.py` keeps observation processes
   alive after complete artifacts. The runner completed JSON artifacts but had
   to be interrupted several times during post-case waits.
