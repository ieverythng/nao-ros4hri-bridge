# Runtime Contracts

Last updated: 2026-04-27

This document is the richer reference for the JSON payloads that move task,
scene, and execution state between nodes. The root README contains compact
examples; this file is the contract-focused view.

## Planner Request

Topic:

- `/planner/request`
- type: `hri_actions_msgs/msg/Intent`
- publisher: `chatbot_llm`
- consumer: `planner_llm`

ROS envelope policy:

- `Intent.priority` and `Intent.confidence` are part of
  `hri_actions_msgs/msg/Intent`, so they will always appear in
  `ros2 topic echo /planner/request`.
- `chatbot_llm` now publishes planner requests with a deterministic priority
  (`128`) and a bounded route confidence. Execution-routed turns with no model
  confidence use a conservative floor rather than `0.0`.
- Planner semantics live in `Intent.data`; do not duplicate priority or
  confidence inside the JSON payload unless a planner policy genuinely needs it.

Preferred payload:

```json
{
  "request_id": "turn_123",
  "goal_id": "goal_turn_123",
  "parent_goal_id": "",
  "supersedes_goal_id": "",
  "request_kind": "new_goal",
  "goal_text": "navigate to the kitchen and report completion",
  "normalized_intents": ["navigate_to"],
  "ack_text": "I will work on that.",
  "ack_mode": "say",
  "scene_targets": ["kitchen"],
  "dialogue_context": [],
  "requested_plan": [],
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "planner_mode": "default",
  "interaction_mode": "speech",
  "dialogue_turn_id": "role:turn"
}
```

Field policy:

- `goal_text`: concise planner-facing objective parsed by
  `planner_common.PlannerRequest` and sent to `planner_llm`.
- `normalized_intents`: strict intent labels from `chatbot_llm`.
- `scene_targets`: target labels/entities extracted by `chatbot_llm`.
- `grounded_context`: bounded symbolic context.
- `requested_plan`: optional hint or fallback. It should not be required for the
  planner to abstract a simple task.
- `user_text`: legacy parser input accepted by `PlannerRequest` for backward
  compatibility, but omitted from normal `chatbot_llm` planner requests and
  from the `planner_llm` prompt payload.

## Planner Output

Topic:

- `/intents`
- type: `hri_actions_msgs/msg/Intent`
- publisher: `planner_llm`
- consumer: `nao_orchestrator`

Payload:

```json
{
  "goal_id": "goal_turn_123",
  "ack_text": "",
  "ack_mode": "",
  "scene_targets": ["kitchen"],
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "plan": {
    "goal_id": "goal_turn_123",
    "plan_id": "plan_123",
    "plan_version": 1,
    "status": "planning",
    "validation_status": "draft",
    "failure_reason": "",
    "user_facing_reason": "",
    "replan_hint": "",
    "retry_budget": 1,
    "scene_targets": ["kitchen"],
    "communication_policy": {
      "emit_acknowledge": false,
      "emit_progress": false,
      "emit_completion": true,
      "emit_failure": true
    },
    "steps": [
      {
        "id": "step_1",
        "type": "skill",
        "name": "perform_motion",
        "args": {"object": "head_look_left"},
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 0
      }
    ]
  }
}
```

Allowed step types:

- `noop`
- `say`
- `skill`
- `look_at`

Allowed failure policies:

- `fail`
- `continue`
- `replan`
- `clarify`
- `ask_user`
- `ignore`

Current limitation: unsupported planner steps can be filtered by the skill
registry. The next diagnostic should verify whether multi-step plans can degrade
into partial plans without being classified as invalid.

## Execution Feedback

Topic:

- `/planner/execution_feedback`
- type: `std_msgs/msg/String`
- publisher: `nao_orchestrator`
- consumer: `planner_llm`

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "intent": "raw_user_input",
  "source": "planner_llm",
  "event_type": "step_failed",
  "status": "failed",
  "reason": "target action server unavailable",
  "validation_status": "draft",
  "replan_hint": "",
  "retry_budget": 1,
  "blocking": true,
  "unmet_preconditions": [],
  "needs_user_input": false,
  "scene_targets": ["kitchen"],
  "validation_errors": [],
  "timestamp_sec": 1777040000.0,
  "step": {
    "id": "step_1",
    "type": "skill",
    "name": "perform_motion",
    "retry_budget": 1,
    "on_failure": "replan",
    "requires": []
  }
}
```

Important event types:

- `plan_accepted`
- `step_started`
- `step_succeeded`
- `step_failed`
- `plan_invalid`
- `plan_completed`

## Planner Dialogue Act

Topic:

- `/planner/dialogue_act`
- type: `std_msgs/msg/String`
- publisher: `planner_llm`
- consumer: `dialogue_manager`

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "act": "ask_clarification",
  "priority": "normal",
  "await_user_response": true,
  "reason": "missing target object",
  "text_hint": "Which target should I use?",
  "slots_needed": ["target"],
  "context": {
    "scene_targets": ["kitchen"],
    "status": "waiting_user"
  }
}
```

Allowed dialogue acts:

- `acknowledge`
- `progress_update`
- `ask_clarification`
- `ask_for_help`
- `explain_failure`
- `notify_completion`
- `notify_cancellation`

## Knowledge Snapshot

Owner:

- `chatbot_llm`

Source:

- `/kb/query` through `kb_skills`

Purpose:

- prompt context for response and intent/planner routing stages.

`knowledge_snapshot` is the chatbot's textual/symbolic view of KnowledgeCore
facts. It is useful because it gives the LLM a compact fact set without exposing
raw KB transport details.

Default query group:

```text
myself sees ?entity && ?entity rdf:type ?type
```

`grounded_context.knowledge_snapshot` example:

```json
{
  "summary_text": "Entities currently seen by the robot: cup_1 (Cup)"
}
```

This is not a `knowledge_core` native concept; it is the local prompt-facing
formatting layer in `chatbot_llm`.

## Scene Summary

Topic:

- `/scene/summary`
- type: `std_msgs/msg/String`
- publisher: `nao_scene_grounding`
- consumer: `chatbot_llm` and operator/debug tooling

Purpose:

- transient detector-grounded observation summary.
- carries detection metadata that is not yet represented as KB facts, including
  labels/classes, scores, image positions, observer/source, and recency.
- supports debugging and future `object_manager`/world-model work without
  making the planner subscribe directly to detector output.

Current policy:

- Keep `/scene/summary` while the KB only stores the selected symbolic facts
  needed for reasoning, such as `myself sees entity` and `entity rdf:type Type`.
- If the KB/object-manager schema later stores confidence, position,
  provenance, and recency directly, the scene summary can become mostly
  operator/debug output.

Example payload:

```json
{
  "observer": "myself",
  "backend": "emorobcare_cv",
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.91,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 320.0,
      "center_y": 240.0,
      "last_seen_sec": 1777040000.0
    }
  ]
}
```

`/scene/summary` is not the same as `knowledge_snapshot`: it is a current object
summary, while `knowledge_snapshot` is prompt-ready text from KB queries.

## World Model Context

Planned/enriched context fields:

- `/world_model/enriched_snapshot`
- `/world_model/enriched_text`

These feed `grounded_context.world_model_snapshot` and
`grounded_context.world_model_text`. The WME layer is future work relative to
the Monday planner-loop priority.
