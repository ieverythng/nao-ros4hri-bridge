# Planner Grounding + MoE Ownership Contract (2026-06-01)

## Goal

Formalize how grounded context should flow through planner seams without
blurring ownership across `chatbot_llm`, `planner_llm`, `nao_orchestrator`, and
AB=1 skills.

## Contract Decisions

1. Planner ingress (`/planner/request`) remains the place where compact T0
   context is attached (`knowledge_snapshot`, `scene_summary`, `state_t0`).
2. Planner egress (`/intents`) should stay plan-centric. Grounded world-state
   payloads are transitional only and should be reduced to lightweight
   `context_ref` lineage as soon as downstream validation is complete.
3. AB=1 skills own live world-state resolution at execution time through AB=0
   seams (for example `/scene/summary`, `/kb/query`, tracked-person feeds,
   skill-local service checks). Planner context is advisory T0 evidence, not an
   execution-time source of truth.

## Why This Matches ROS4HRI Ownership

- `chatbot_llm` owns user-facing grounding assembly for planner ingress.
- `planner_llm` owns task decomposition and replanning policy, not live sensor
  polling.
- `nao_orchestrator` owns deterministic dispatch/feedback, not global world
  modeling.
- AB=1 skills remain the execution experts and can specialize by evidence type
  (object-centric, people-centric, navigation-centric), which gives a practical
  MoE pattern without introducing a separate coordinator node.

## JSON Shape Guidance

### Planner Request (authoritative grounding entry)

In `scene_summary`, keep `objects` object-only and represent people in
`people` so downstream seams do not treat humans as generic objects.

```json
{
  "grounded_context": {
    "knowledge_snapshot": {
      "schema_version": "knowledge_snapshot_v2",
      "references": [],
      "counts": {"entities": 0, "people": 0, "objects": 0}
    },
    "scene_summary": {
      "schema_version": "scene_summary_v2",
      "observer": "myself",
      "backend": "emorobcare_cv",
      "captured_at_sec": 0.0,
      "objects": [],
      "people": []
    },
    "state_t0": {
      "schema_version": "state_t0_v2",
      "observer": "myself",
      "backend": "emorobcare_cv",
      "captured_at_sec": 0.0,
      "entity_counts": {"entities": 0, "people": 0, "objects": 0},
      "entities": []
    }
  }
}
```

### Planner Output (target steady-state)

```json
{
  "plan": {
    "goal_id": "goal_turn_123",
    "plan_id": "plan_123",
    "plan_version": 1,
    "context_ref": {
      "captured_at_sec": 1777040000.0,
      "observer": "myself",
      "backend": "emorobcare_cv"
    },
    "steps": []
  }
}
```

## MoE-by-AB Practical Pattern

1. Planner chooses AB=1 skills and passes compact targets (`scene_targets`,
   `look_at`/`find` args). Any `state_t0.entities[*].id` is a valid
   `look_at.target_frame` candidate by prompt policy.
2. Each AB=1 skill performs evidence resolution against its own AB=0 sources.
3. Execution feedback publishes the evidence summary and status.
4. Replanning uses feedback + new ingress context, not stale output copies.

This keeps seams deterministic and allows each skill to specialize without
turning the planner output into a world-model transport channel.
