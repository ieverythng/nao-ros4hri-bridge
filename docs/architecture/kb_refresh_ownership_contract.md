# KB Refresh and Spatial Evidence Ownership

## Decision

Each runtime node owns the freshness of facts produced from its own observations
or execution results. Nodes do not write facts on behalf of unrelated sensors or
skills, and LLM nodes never write directly to KnowledgeCore.

All writes cross the `kb_skills` boundary through the canonical `/kb/revise`
service. Planner-visible mutations (`kb_add`, `kb_revise`, and `kb_remove`) are
explicit user-authorized operations dispatched by `nao_orchestrator` through
`KnowledgeCoreMutationClient`.

## Ownership Matrix

| Owner | Intake | Refresh/write responsibility | Consumer-facing output |
| --- | --- | --- | --- |
| `nao_scene_grounding` | detector messages | transient object visibility, class, visual center, score, source, last-seen time | `/scene/summary`, KnowledgeCore predicates |
| ROS4HRI person managers | face/body/voice streams | person identity and tracking state | standard ROS4HRI person topics |
| AB=1 execution skills | live state relevant to the skill | execution effects and result evidence produced by that skill | typed action result / `SkillResultPayload` |
| `kb_skills` | query and mutation requests | KnowledgeCore transport and normalization only | KB query/mutation results |
| `chatbot_llm` / `planner_llm` | compact `grounded_context` | no autonomous refresh or direct KB writes | dialogue intent / plan |
| `nao_orchestrator` | validated plan steps | deterministic dispatch only; delegates explicit KB mutations | execution feedback |

## Freshness Rule

The planner request contains a compact T0 view for reasoning. Before claiming a
real-world effect or current observation, the responsible AB=1 skill must refresh
or query the live state it owns. T0 is evidence for planning, not proof of
execution-time truth.

## Spatial Evidence

The current object grounding contract exposes image-plane `center_x` and
`center_y`. These values support attention targeting and visual disambiguation,
but they are not metric TF coordinates and must not be used to answer proximity
questions.

Metric proximity uses a source-owned transform contract:

1. The detector or simulator publishes a source frame and 3D pose.
2. The source transforms that pose into an agreed robot/world frame and publishes
   it on the configured spatial-overlay topic keyed by grounded entity id.
3. The scene summary and KB expose frame-qualified position and observation time.
4. Chatbot/planner projections include those fields only when the user task
   requires spatial comparison.

`nao_scene_grounding` now owns the optional overlay merge, distance derivation,
KB refresh, and enriched scene-summary publication. Until a source supplies that
overlay, proximity questions must be answered as unknown or clarified rather
than inferred from image-plane centers. Selecting the canonical world frame and
real/simulator pose source remains a design decision for the next supervisor
meeting.
