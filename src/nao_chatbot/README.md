# nao_chatbot

`nao_chatbot` is now the launch and operator-utility package for the migrated
NAO ROS4HRI stack.

It no longer owns chatbot execution or mission control logic. Those
responsibilities have moved to:

- `chatbot_llm`: chatbot backend contract
- `dialogue_manager`: canonical `/skill/chat`, `/skill/ask`, `/skill/say`
- `nao_orchestrator`: downstream intent dispatch

## Launch Files

Primary migrated stack:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
```

Useful demo overrides:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_naoqi_driver:=true \
  start_rqt_console:=true \
  start_rqt_chat:=true \
  ollama_model:=gpt-oss:120b-cloud
```

Primary migrated stack with ASR:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_with_asr.launch.py
```

ASR-only utility profile:

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py
```

## Operator Utility

The package still ships the push-to-talk helper used by `asr_vosk`:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli
```

It also ships a small speech-debug helper that mirrors final robot utterances
from `/debug/nao_say/speech` and `/dialogue_manager/closed_captions` into ROS
logs, which makes the exact spoken text visible in `rqt_console`:

```bash
ros2 run nao_chatbot robot_speech_debug
```

## Notes

- legacy mission-controller and `/skill/chat` server code has been removed from
  this package
- old `nao_chatbot_stack` launch wrappers have been removed as part of the
  ROS4HRI cleanup
- this package is now a launch surface, not a skill implementation package
- the migrated launch enables default chat by default so incoming speech is
  routed to `chatbot_llm` immediately
- `start_rqt_console:=true` now opens the full `rqt` shell instead of only the
  standalone console plugin
- `start_rqt_chat:=true` launches upstream `rqt_chat` unchanged, but remaps its
  internal `/tts_engine/tts` server onto `/debug/say` so it can display robot
  speech without conflicting with `nao_say_skill`
- `debug_tts_action_name:=/debug/say` controls the debug-only TTS action used
  between `nao_say_skill` and `rqt_chat`
- the Docker images install `rqt_chat` from the `socialminds-ros-jazzy-rqt-chat`
  system package; the repo does not vendor that package in `src/`
