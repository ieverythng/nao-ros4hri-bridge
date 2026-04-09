# planner_llm

`planner_llm` is the planner-facing ROS node that turns execution-oriented
planner requests into executable plan envelopes on `/intents`.

It sits between `chatbot_llm` and `nao_orchestrator` when planner mode is
enabled:

```text
dialogue_manager -> chatbot_llm -> /planner/request -> planner_llm
planner_llm -> /intents -> nao_orchestrator
nao_orchestrator -> /planner/execution_feedback -> planner_llm
```

The package does not own user dialogue. It owns:

- planner request interpretation
- structured plan generation
- bounded replanning after downstream failure feedback

The package does not own:

- text-to-speech dispatch
- robot skill execution
- detector subscriptions
- KnowledgeCore writes

## Topics

Primary ROS interfaces:

- subscribe: `/planner/request` as `hri_actions_msgs/msg/Intent`
- publish: `/intents` as `hri_actions_msgs/msg/Intent`
- subscribe: `/planner/execution_feedback` as `std_msgs/msg/String`
- subscribe: `/world_model/enriched_snapshot` as `std_msgs/msg/String`
- subscribe: `/world_model/enriched_text` as `std_msgs/msg/String`

## Planner Request Shape

`chatbot_llm` publishes planner ingress requests with JSON in `Intent.data`.

Expected fields:

```json
{
  "request_id": "turn_123",
  "user_text": "look left and then sit down",
  "normalized_intents": ["head_look_left"],
  "ack_text": "I will do that.",
  "ack_mode": "say",
  "scene_targets": [],
  "dialogue_context": [],
  "grounded_context": {
    "knowledge_snapshot": {}
  },
  "planner_mode": "multi_step"
}
```

Notes:

- `normalized_intents` are best-effort hints, not the sole source of truth.
- `user_text` remains the authoritative user request string.
- `planner_mode` is how `chatbot_llm` flags composite turns such as multi-step
  or sequenced actions.

## Output Shape

`planner_llm` emits the normal downstream `Intent` contract on `/intents`. The
executable structure lives inside `Intent.data.plan`.

Typical output:

```json
{
  "ack_text": "I will look left and then sit down.",
  "ack_mode": "say",
  "scene_targets": [],
  "plan": {
    "plan_id": "plan_123",
    "validation_status": "draft",
    "failure_reason": "",
    "replan_hint": "",
    "retry_budget": 1,
    "scene_targets": [],
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

## Replan Loop

`nao_orchestrator` publishes downstream execution feedback on
`/planner/execution_feedback`. `planner_llm` uses that feedback to either:

- keep the current plan completed
- emit a revised plan
- emit a bounded clarify/fail response as a planned `say` step

The planner layer therefore stays execution-aware without taking over robot
skill ownership from `nao_orchestrator`.

## Parameters

Defaults live in [`config/00-defaults.yml`](./config/00-defaults.yml).

Most important parameters:

- `planner_request_topic`
- `intent_topic`
- `planner_feedback_topic`
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

- `planner_llm` stays planner-facing rather than dialogue-facing.
- `chatbot_llm` still owns the immediate spoken acknowledgement.
- `nao_orchestrator` remains the deterministic execution layer.
- World-model inputs are optional and currently arrive through enriched snapshot
  topics, not through direct detector subscriptions.
