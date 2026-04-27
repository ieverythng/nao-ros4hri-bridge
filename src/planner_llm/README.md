# planner_llm

`planner_llm` is the planner and goal supervisor. It consumes `/planner/request`,
generates executable plan envelopes on `/intents`, listens to
`/planner/execution_feedback`, and publishes `/planner/dialogue_act` when the
dialogue side needs to speak, clarify, or report failure.

## Owns

- task planning over abstract skill metadata
- goal IDs, plan IDs, and plan versions
- retry/replan/clarification/failure decisions
- planner dialogue acts

It does not own:

- final speech realization
- robot skill execution
- raw detector subscriptions
- direct KnowledgeCore transport

## Public ROS Interfaces

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| subscribe | `/planner/request` | `hri_actions_msgs/msg/Intent` | Planner ingress from `chatbot_llm` |
| publish | `/intents` | `hri_actions_msgs/msg/Intent` | Executable downstream plan |
| subscribe | `/planner/execution_feedback` | `std_msgs/msg/String` | Executor feedback from `nao_orchestrator` |
| publish | `/planner/dialogue_act` | `std_msgs/msg/String` | Planner communication request |
| subscribe | `/world_model/enriched_snapshot` | `std_msgs/msg/String` | Optional future WME snapshot |
| subscribe | `/world_model/enriched_text` | `std_msgs/msg/String` | Optional future WME text |

## Contract Role

Planner input should be understood as:

- `goal_text`, `normalized_intents`, `scene_targets`, and `grounded_context`
  are the clean planner signals.
- `requested_plan` is a hint/fallback, not the only way to make the planner work.
- raw `user_text` is legacy input only and is not included in the model prompt
  payload during normal operation.

Planner output is an `Intent.data.plan` envelope with step types `noop`, `say`,
`skill`, and `look_at`. Full examples are in `../../docs/contracts.md`.

## Important Parameters

Defaults live in `config/00-defaults.yml`.

- `planner_request_topic`
- `intent_topic`
- `planner_feedback_topic`
- `planner_dialogue_act_topic`
- `enriched_snapshot_topic`
- `enriched_text_topic`
- `skill_registry_path`
- `provider`
- `model`
- `base_url`
- `temperature`
- `max_tokens`
- `timeout_sec`
- `default_retry_budget`
- `auto_replan`

## Launch And Smoke Test

```bash
ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py
ros2 run planner_llm publish_fixture request
ros2 run planner_llm publish_fixture feedback
```

Observe:

```bash
ros2 topic echo /planner/request
ros2 topic echo /intents
ros2 topic echo /planner/execution_feedback
ros2 topic echo /planner/dialogue_act
```

## Tests

```bash
PYTHONPATH=src/planner_common:src/planner_llm:src/kb_skills \
python3 -m pytest -q src/planner_llm/test
```

## Current Limitations

- Multi-step completeness needs direct diagnostic coverage.
- Unsupported skill filtering can hide planner/model issues if not checked.
- Preconditions are currently metadata and feedback labels, not full world-state
  gates.
