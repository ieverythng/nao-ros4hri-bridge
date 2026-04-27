# nao_orchestrator

`nao_orchestrator` is the deterministic executor for downstream
`hri_actions_msgs/msg/Intent` messages. It validates structured plans, executes
steps in order, and publishes planner feedback.

## Owns

- `/intents` subscription
- structured `Intent.data.plan` validation
- ordered execution of supported plan steps
- action clients for NAO speech, replay motion, head motion, and look-at
- `/planner/execution_feedback`

It does not own user dialogue, LLM prompting, planner policy, detector
subscriptions, or KnowledgeCore transport.

## Public ROS Interfaces

| Direction | Interface | Type | Purpose |
| --- | --- | --- | --- |
| subscribe | `/intents` | `hri_actions_msgs/msg/Intent` | Direct or planner-generated intents |
| optional subscribe | `/chatbot/intent` | `std_msgs/msg/String` | Legacy bridge, disabled by default |
| publish | `/planner/execution_feedback` | `std_msgs/msg/String` | Plan lifecycle feedback |
| action client | `/nao/say` | `communication_skills/action/Say` | Speech step execution |
| action client | `/skill/replay_motion` | `nao_skills/action/ReplayMotion` | Motion/posture execution |
| action client | `/skill/do_head_motion` | `nao_skills/action/DoHeadMotion` | Head motion execution |
| action client | `/skill/look_at` | `interaction_skills/action/LookAt` | Gaze execution |

Temporary fallbacks:

- `/chatbot/posture_command`
- `/joint_angles`

## Supported Plan Steps

- `noop`: explicit no-op.
- `say`: dispatches through `/nao/say` when speech dispatch is enabled.
- `skill` with `perform_motion` or `motion`: replay/head/look-at-reset routing.
- `skill` with `look_at`: target-frame or reset gaze routing.
- `look_at`: target-frame or reset gaze routing.

Unsupported steps should produce validation or step-failure feedback rather than
becoming silent behavior.

## Important Parameters

Defaults live in `config/00-defaults.yml`.

- `intent_topic`
- `enable_legacy_intent_bridge`
- `legacy_intent_topic`
- `nao_say_action`
- `dispatch_speech_intents`
- `replay_motion_action`
- `head_motion_action`
- `look_at_action`
- `posture_command_topic`
- `planner_feedback_topic`
- `dedupe_window_sec`

## Launch And Tests

```bash
ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py
```

Focused tests:

```bash
PYTHONPATH=src/planner_common:src/nao_orchestrator:src/kb_skills \
python3 -m pytest -q src/nao_orchestrator/test
```

## Design Rule

Keep this package downstream-only. Planner decisions belong in `planner_llm`,
dialogue and speaking policy belong in `dialogue_manager`/`chatbot_llm`, and KB
transport belongs in `kb_skills`.
