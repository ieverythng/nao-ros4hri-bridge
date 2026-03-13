# nao_orchestrator

`nao_orchestrator` is the new lifecycle orchestration package for NAO.

Steady-state target:

- consume `/intents`
- dispatch canonical and NAO-specific skills
- replace the local mission-controller role previously hosted in `nao_chatbot`

What moved out of the old `mission_controller` on purpose:

- user-text ingestion now belongs to `dialogue_manager`
- chatbot turn execution now belongs to `dialogue_manager` + `chatbot_llm`
- assistant-text generation/history no longer lives in the orchestrator
- `nao_orchestrator` only owns downstream intent normalization and robot-skill dispatch

Transition support:

- optional subscription to the legacy string topic `/chatbot/intent`
- direct dispatch to `/nao/say`, `/skill/replay_motion`, `/skill/do_head_motion`,
  and `/skill/look_at`
- topic fallbacks matching the old mission-controller flow:
  `/chatbot/posture_command` and `/joint_angles`
- duplicate-intent suppression to avoid double-dispatch while legacy and new paths coexist

Current migration boundary:

- `nao_orchestrator` already covers the old mission-controller execution side:
  say dispatch, posture/replay-motion dispatch, retained head motion, and look-at reset
- `chatbot_llm` does not connect directly to `nao_orchestrator` in steady state
  because the canonical flow is `dialogue_manager -> /intents -> nao_orchestrator`
- legacy `/chatbot/intent` support remains only as a temporary adapter for older producers

Manual smoke examples:

```bash
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: 'posture_stand'}"
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: 'head_look_left'}"
ros2 topic pub --once /chatbot/intent std_msgs/msg/String "{data: '{\"intent\":\"__intent_say__\",\"object\":\"Testing migrated say dispatch.\"}'}"
```

```bash
ros2 topic pub --once /intents hri_actions_msgs/msg/Intent "{intent: 'perform_motion', data: '{\"object\":\"stand\"}'}"
```
