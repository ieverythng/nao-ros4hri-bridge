# Runtime Closure Hardening, 13 July 2026

## Objective

The implementation addresses the remaining 8.2/10 CRITIC failures without
moving ROS ownership or making prompt text the first intervention. The target
seams are grouped target resolution, repeated blocking replans, person placement
semantics, fixture isolation, recovery evidence correlation, and ROS4HRI SVG
compatibility.

## Implemented Source Changes

1. `planner_common.PlannerRequest` now normalizes a bounded
   `target_selection` contract.
2. `chatbot_llm` preserves explicit target selections and derives grouped
   delivery selections from existing structured intent fields plus unambiguous
   grounded locations and recipients.
3. `planner_llm` validates location membership and consumes the selection after
   normal model validation fails. The superseded lexical grouped-delivery and
   ordered-walk fallbacks have been removed; an absent or invalid authoritative
   selection now fails closed.
4. `PlannerSupervisor` allows one retry and rejects a second unchanged plan
   after the same blocking failure.
5. `planner_common.report_outcome` separates handoff from placement. A person
   cannot be a placement support, even when an upstream result claims success.
6. Fixture preloading validates explicit robot and person locations. The runtime
   questionnaire removes previous fixture facts between independent groups and
   refuses to score unverified cleanup.
7. Recovery scoring now requires user-facing speech after terminal evidence.
8. The four packaged SVG maps now use the upstream `rqt_human_radar` environment
   schema and millimetre coordinate scale.

## Evidence

- Planner contracts, planner engine, supervisor, and report outcome: 106 tests
  passed.
- Nested chatbot planner request adapter: 28 tests passed.
- Runtime questionnaire assessment: 14 tests passed.
- Fixture and SVG schema validation: 4 tests passed.
- Fake-skill policy engine: 22 tests passed.
- Orchestrator ROS tests remain a container gate because generated ROS message
  packages are unavailable in the host Python environment.

The final overlay image was rebuilt from `iiia:nao` and launched as one fresh
container. Both lifecycle nodes were active, core nodes were unique,
`response_first` was selected, and the compact grounded-context digest was
disabled.

Runtime results:

- Ordered walk/report passed under all-success and fail-once navigation.
- Grouped work-table delivery passed under all-success without clarification.
- Blocked delivery retried the grounded handoff once, then produced truthful
  chatbot-authored help. It no longer replaced delivery with table placement or
  claimed success.
- Fail-once pick now fails the first pick and succeeds on the second attempt
  across `object_id` and `target` aliases. The replan subsequently failed to
  confirm the placement target in KnowledgeCore and asked for help.
- Severe fallback markers were zero in the final pick run. Route repair remains
  visible in grouped-delivery runs.

Artifacts: `/tmp/nao_fake_deep_20260713_round2.json`,
`/tmp/nao_fake_deep_fail_once_navigation_20260713.json`,
`/tmp/nao_fake_deep_delivery_blocked_fixed_20260713.json`, and
`/tmp/nao_composite_fail_once_pick_fixed_20260713.json`.

## Remaining Evidence

The pick replan must preserve a KB-confirmable placement support after the
object is acquired. SVG acceptance also requires an rqt screenshot showing the
selected map and loaded static objects. The packaged maps now comply with the
upstream loader schema, but visual rendering was not observed in this pass.

Prompt mutation is not accepted at this checkpoint. If grouped requests still
fail after these structural changes, the next step is a bounded SkillOpt run
that exposes `target_selection` in the canonical schema and evaluates train and
holdout cases before acceptance.

## 14 July Source Extension

The later semantic audit found that the earlier runtime rubric could accept a
complete ROS trajectory even when the selected entity roles were wrong. The
source closure now adds the following gates:

- `target_selection` is copied into the nested plan envelope and checked again
  by the orchestrator before dispatch.
- Ordered visits must navigate to exactly the selected object identifiers.
  Delivery plans must cover every selected object, preserve the person
  recipient, and exclude support or location anchors.
- Route-repaired grouped requests can resolve source and recipient names from
  grounded identifiers, labels, aliases, and `dbp:name` facts.
- Explicit non-action instructions cannot leak an executable KB mutation after
  a dialogue response.
- Final `report_outcome` evidence marks selected members that lack successful
  execution evidence. The chatbot remains the normal wording authority; the
  deterministic fallback is used only after empty or unsafe output.
- Spatial effects use KnowledgeCore type evidence rather than words embedded in
  identifiers when projecting a target to a containing place.
- Fixture cleanup retracts and verifies incoming as well as outgoing facts.
  Questionnaire phase evidence is correlated by voice, dialogue, turn, and
  planner goal identifiers.

Current focused source gates: 53 planner-common tests, 78 planner tests (with
one environment skip), 124 chatbot handoff/turn tests, 15 questionnaire tests,
and 2 pure KB-effect tests passed. Orchestrator ROS tests and the full runtime
matrix remain pending a clean rebuild from `iiia:nao`; no score is assigned to
this source checkpoint.
