# Current Workflow

Last updated: 2026-03-13

This document describes the active migrated runtime. The old
`mission_controller`, `ollama_chatbot`, and `nao_skill_servers` flow has been
removed from the workspace.

## Runtime Summary

Primary runtime:

```text
/humans/voices/*/speech
  -> dialogue_manager
  -> chatbot_llm
  -> /intents
  -> nao_orchestrator
  -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

ASR is still transitional:

```text
simple_audio_capture -> asr_vosk -> /humans/voices/anonymous_speaker/speech
```

That ASR path can be launched independently through
`nao_chatbot_asr_only.launch.py` or together with the migrated stack through
`nao_chatbot_ros4hri_with_asr.launch.py` while the upstream ASR contract is
being restored.

## Primary Launch Files

| Launch file | Purpose |
| --- | --- |
| `nao_chatbot_ros4hri_migration.launch.py` | Main migrated runtime |
| `nao_chatbot_ros4hri_with_asr.launch.py` | Main migrated runtime plus `simple_audio_capture` and `asr_vosk` |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR testing |

## Responsibility Split

| Package | Owns |
| --- | --- |
| `dialogue_manager` | Canonical `/skill/chat`, `/skill/ask`, `/skill/say` and dialogue tracking |
| `chatbot_llm` | Chatbot backend contract (`start_dialogue`, `dialogue_interaction`) |
| `nao_orchestrator` | `/intents` consumption and robot-side dispatch |
| `nao_say_skill` | `/nao/say` |
| `nao_replay_motion` | `/skill/replay_motion`, temporary `/skill/do_posture`, retained `/skill/do_head_motion` |
| `nao_look_at` | `/skill/look_at` scaffold |

## Transitional Interfaces Still Present

- `/chatbot/posture_command`
- `/joint_angles`
- optional legacy `/chatbot/intent` subscription in `nao_orchestrator`

These exist only to keep motion execution stable while the migration finishes.

## Verification Commands

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
ros2 launch nao_chatbot nao_chatbot_ros4hri_with_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
ros2 action list -t
ros2 service list -t
ros2 lifecycle get /dialogue_manager
ros2 lifecycle get /chatbot_llm
ros2 lifecycle get /nao_orchestrator
```

Lifecycle nodes in the migrated stack are configured on process start and only
activated once they report `inactive`; the launch surface no longer relies on
fixed startup delays between transitions.
