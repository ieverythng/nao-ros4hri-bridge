# nao_orchestrator

`nao_orchestrator` is the new lifecycle orchestration package for NAO.

Steady-state target:

- consume `/intents`
- dispatch canonical and NAO-specific skills
- replace the local mission-controller role previously hosted in `nao_chatbot`

Transition support:

- optional subscription to the legacy string topic `/chatbot/intent`
- direct dispatch to `/nao/say`, `/skill/replay_motion`, `/skill/do_head_motion`,
  and `/skill/look_at`
