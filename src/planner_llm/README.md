# planner_llm

`planner_llm` is now the planner-facing supervisory ROS node. It tracks goals
over time, emits executable plan envelopes on `/intents`, and publishes
planner-owned dialogue acts on `/planner/dialogue_act`.

It sits between `chatbot_llm` and `nao_orchestrator` when planner mode is
enabled:

```text
dialogue_manager -> chatbot_llm -> /planner/request -> planner_llm
planner_llm -> /intents -> nao_orchestrator
planner_llm -> /planner/dialogue_act -> chatbot_llm/dialogue layer
nao_orchestrator -> /planner/execution_feedback -> planner_llm
```

The package does not own final user phrasing. It owns:

- goal supervision
- structured plan generation
- bounded replanning after downstream failure feedback
- clarification and cancellation policy
- planner-side communication decisions

The package does not own:

- text-to-speech lifecycle
- robot skill execution
- detector subscriptions
- KnowledgeCore writes

## Topics

Primary ROS interfaces:

- subscribe: `/planner/request` as `hri_actions_msgs/msg/Intent`
- publish: `/intents` as `hri_actions_msgs/msg/Intent`
- publish: `/planner/dialogue_act` as `std_msgs/msg/String`
- subscribe: `/planner/execution_feedback` as `std_msgs/msg/String`
- subscribe: `/world_model/enriched_snapshot` as `std_msgs/msg/String`
- subscribe: `/world_model/enriched_text` as `std_msgs/msg/String`

## Planner Request Shape

`chatbot_llm` publishes planner ingress requests with JSON in `Intent.data`.

Expected fields:

```json
{
  "request_id": "turn_123",
  "goal_id": "goal_123",
  "request_kind": "new_goal",
  "user_text": "look left and then sit down",
  "normalized_intents": ["head_look_left"],
  "ack_text": "I will do that.",
  "ack_mode": "say",
  "scene_targets": [],
  "dialogue_context": [],
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "planner_mode": "multi_step",
  "interaction_mode": "default",
  "dialogue_turn_id": "dialogue_123"
}
```

Notes:

- `normalized_intents` are best-effort hints, not the sole source of truth.
- `user_text` remains the authoritative user request string.
- `goal_id` is the supervisor key; replans and clarification answers should
  reuse it.
- `request_kind` differentiates new goals, clarification answers, updates, and
  supervisor-local cancellation.

## Output Shape

`planner_llm` emits the normal downstream `Intent` contract on `/intents` when
it has an executable plan. The executable structure lives inside
`Intent.data.plan`.

Typical output:

```json
{
  "goal_id": "goal_123",
  "ack_text": "",
  "ack_mode": "",
  "scene_targets": [],
  "plan": {
    "goal_id": "goal_123",
    "plan_id": "plan_123",
    "plan_version": 2,
    "status": "replanning",
    "validation_status": "draft",
    "failure_reason": "",
    "replan_hint": "",
    "retry_budget": 1,
    "scene_targets": [],
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
      },
      {
        "id": "step_2",
        "type": "skill",
        "name": "perform_motion",
        "args": {"object": "sit"},
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 0
      }
    ]
  }
}
```

## Dialogue Act Shape

When the supervisor decides something should be surfaced to the user without
issuing a new executable plan, it publishes a dialogue act on
`/planner/dialogue_act`.

```json
{
  "goal_id": "goal_123",
  "plan_id": "plan_123",
  "plan_version": 2,
  "act": "ask_clarification",
  "priority": "normal",
  "await_user_response": true,
  "reason": "missing target object",
  "text_hint": "Which cup do you mean?",
  "slots_needed": ["target_object"],
  "context": {
    "scene_targets": ["cup"],
    "status": "waiting_user"
  }
}
```

## Supervisor Loop

`nao_orchestrator` publishes downstream execution feedback on
`/planner/execution_feedback`. `planner_llm` uses that feedback to either:

- keep the current goal executing
- emit a revised plan version
- publish a clarification/help/failure dialogue act
- mark a goal completed or cancelled

The planner therefore stays execution-aware without taking over robot skill
ownership from `nao_orchestrator`.

## Parameters

Defaults live in [`config/00-defaults.yml`](./config/00-defaults.yml).

Most important parameters:

- `planner_request_topic`
- `intent_topic`
- `planner_feedback_topic`
- `planner_dialogue_act_topic`
- `skill_registry_path`
- `provider`
- `model`
- `base_url`
- `temperature`
- `max_tokens`
- `timeout_sec`
- `default_retry_budget`
- `auto_replan`

## Local Smoke Tests

Planner-only harness:

```bash
ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py
```

Fixture publisher:

```bash
ros2 run planner_llm publish_fixture request
ros2 run planner_llm publish_fixture feedback
```

Focused validation:

```bash
colcon build --packages-select planner_common planner_llm
pytest src/planner_common/test/test_contracts.py src/planner_llm/test/test_planner_engine.py
```

## Design Notes

- `planner_llm` is now supervisor-facing rather than a stateless planner shim.
- `chatbot_llm` and `dialogue_manager` still own final phrasing and speech
  lifecycle.
- `nao_orchestrator` remains the deterministic execution layer.
- World-model inputs are optional and currently arrive through enriched snapshot
  topics, not through direct detector subscriptions.
- The skill registry in `config/skill_registry.json` is the planner-owned
  abstract execution surface; the planner should not reason over raw robot
  topics or NAOqi APIs.
