# Runtime Review 2026-07-10: SV-Critical Seams

This artifact records the live `nao_ros2` review run performed against the
currently running `iiia:nao` image on 10 July 2026. It is evidence for the next
patching thread, not a replacement for the active trackers.

## Score

**6.7 / 10, current image baseline.**

The stack shows real progress in `report_result` wording, execution routing,
and fake-skill recovery. It is not yet supervisor-demo solid for KB mutation
and spatial post-effects because grouped delivery can leave contradictory
location facts and natural KB mutation can still reach the orchestrator with
invalid statement shapes.

## Source Fingerprint

The running container does include the new seam modules. The earlier suspicion
that they were absent came from probing old or incorrect symbol names.

| Module | Live path | Expected live symbol | SHA-256 prefix |
| --- | --- | --- | --- |
| `planner_common.report_outcome` | `/home/ubuntu/ws/build/planner_common/planner_common/report_outcome.py` | `build_report_outcome` | `e58cbb1a34bbbbd7` |
| `nao_orchestrator.kb_effects` | `/home/ubuntu/ws/build/nao_orchestrator/nao_orchestrator/kb_effects.py` | `remove_stale_spatial_effect_values` | `794ecb9f4b745953` |
| `chatbot_llm.response_fallbacks` | `/home/ubuntu/ws/build/chatbot_llm/chatbot_llm/response_fallbacks.py` | `_uses_generic_completion_for_ordered_navigation` | `c6ab126e1d9b66e0` |

The image provenance is still not strong enough for scored rebuild claims:

- The container is tagged only as mutable `iiia:nao`.
- The branch head was `ac40cfe revert: remove non-doc runtime hardening changes`.
- Runtime seam files were present as unstaged worktree changes.

Future scored runs should use a unique image tag and a pre-score import
fingerprint check.

## Artifacts

| Artifact | Purpose |
| --- | --- |
| `/tmp/nao_runtime_snapshot_20260710_pre.json` | Pre-review graph/log snapshot |
| `/tmp/nao_architecture_sweep_20260710_sv.json` | Architecture sweep, KB mutation, fake guard, preloaded delivery |
| `/tmp/nao_sv_main_focus_20260710.json` | Focused main questionnaire |
| `/tmp/nao_sv_environment_focus_20260710.json` | Preloaded environment and grouped-location checks |
| `/tmp/nao_sv_fake_deep_all_success_20260710.json` | Fake-deep all-success subset |
| `/tmp/nao_sv_fake_deep_fail_nav_20260710.json` | Fake-deep fail-once-navigation subset |
| `/tmp/nao_runtime_snapshot_20260710_post.json` | Post-review graph/log snapshot |

## Passed Evidence

- Architecture KB lifecycle passed through chatbot, planner, orchestrator, and
  KnowledgeCore for explicit structured add, revise, non-mutating query, and
  remove.
- Direct fake guard succeeded for a grounded recipient and failed safely for a
  missing recipient.
- `composite_head_all_directions` executed multiple `perform_motion` steps and
  completed. This is a concrete improvement over the earlier "nice response,
  wrong action" regression.
- `maximal_kitchen_cup_to_operator` planned and completed a multi-step path
  with `navigate_to`, `find_object`, `pick_object`, `navigate_to`,
  `place_object`, and `report_result`.
- Fake-deep all-success passed ordered walk/report, grouped work-table
  delivery, missing-object recovery, and missing-recipient clarification.
- Fake-deep `fail_once_navigation` produced a failed `navigate_to`, then a
  replanned successful `navigate_to`/`scan`/`report_result` chain.
- No planner invalid JSON, invalid executable plan, language-model-unreachable
  speech, execution-report fallback, or rules-response fallback appeared in the
  focused post-run metrics.

## Failed Or Weak Evidence

### Natural KB Mutation

`kb_mutation_add_red_cup` was marked as `pass` by the harness, but the trace
shows a semantic failure:

- The robot first said it would add the cup.
- The planner emitted `kb_add` with invalid statement strings such as
  `dbp:name: red cup`, `dbp:color: red`, and `rdf:type: Cup`.
- `nao_orchestrator` rejected the step with:
  `KnowledgeCore mutation requires explicit RDF-style statements`.
- A later system turn explained the failure.

The questionnaire status is therefore too lenient for KB mutation scoring.
These cases must require postcondition queries.

### Grouped Location Delivery

`environment_grouped_location_delivery` was marked as `pass`, but the spoken
and persisted evidence is not acceptable:

- The report said only: `I have brought the kitchen book to ALEX.`
- The follow-up answered: `The kitchen cup is currently in the kitchen.`
- Direct KB query showed both `codex_kitchen_cup` and `codex_kitchen_book`
  retained stale kitchen facts after delivery while also gaining recipient
  facts.
- The recipient gained object-like inverse spatial facts such as
  `isAt codex_kitchen_cup` and `isUnder codex_kitchen_cup`.

This is the main remaining KB mutation/post-effect seam.

### Complete Context Clarification

`environment_iiia_kitchen_delivery` failed with complete fixture context. The
robot asked:

`Could you please specify which location I should collect the objects from?`

This should not happen when the fixture already contains the requested kitchen
and its objects.

## Post-Run Snapshot

The post-run snapshot recorded:

- `planner_request_count=211`
- `report_result_count=408`
- `knowledge_update_count=571`
- `knowledge_delete_count=526`
- `fallback_total_count=117`
- `route_repair=110`
- `planner_invalid_json=0`
- `execution_report_fallback=0`
- `face_skip_warning_count=714`

The ROS graph still reports duplicate node names, with `/interaction_trace_viewer`
duplicated.

## Patching Brief

1. Patch planner KB mutation normalization so natural requests such as
   "Add a red cup to your KB" produce subject-qualified RDF-style statements or
   ask for clarification before planner execution.
2. Tighten `nao_orchestrator.kb_effects` so successful object delivery removes
   stale object-side `isAt`, `isIn`, `isOn`, `isContainedIn`, `placeOf`, and
   `isUnder` facts and does not write object-like spatial facts onto the human
   recipient.
3. Make grouped delivery report completion derive from successful object-delivery
   steps, not from prompt wording or partial target lists.
4. Strengthen `run_active_questionnaire.py` scoring so KB mutation cases require
   postcondition queries and complete-context execution cases fail when they ask
   for avoidable clarification.
5. Add a rebuild fingerprint gate: unique image tag, source/head capture,
   module hash check, and live symbol check before any scored run.
