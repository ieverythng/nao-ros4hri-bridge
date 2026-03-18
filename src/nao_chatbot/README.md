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
  start_interaction_sim:=true \
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

It also ships a small speech-debug helper that mirrors the live conversation
into ROS logs:

- robot speech from `/debug/nao_say/speech`
- system captions from `/dialogue_manager/closed_captions`
- user captions from `/dialogue_manager/closed_captions`

The helper labels user and robot captions separately so the operator trace in
`rqt_console` does not make user speech look like robot speech:

```bash
ros2 run nao_chatbot robot_speech_debug
```

## Provenance

- `nao_chatbot` is a local utility/launch package, not a forked upstream runtime
  repo
- it exists to compose the migrated stack, expose demo/debug launch surfaces,
  and ship operator helpers such as `asr_push_to_talk_cli` and
  `robot_speech_debug`
- chatbot execution remains in the forked `chatbot_llm` repo and dialogue
  execution remains in the forked `dialogue_manager` repo

## Notes

- legacy mission-controller and `/skill/chat` server code has been removed from
  this package
- old `nao_chatbot_stack` launch wrappers have been removed as part of the
  ROS4HRI cleanup
- this package is now a launch surface, not a skill implementation package
- the migrated launch enables default chat by default so incoming speech is
  routed to `chatbot_llm` immediately
- `start_rqt_console:=true` opens a single remapped `rqt` shell; with
  `start_interaction_sim:=true` it loads the official `interaction_sim`
  perspective so `rqt_console`, `rqt_chat`, `rqt_human_radar`, and the image
  views are available in one window
- `start_rqt_chat:=true` is now optional and only needed if you want a separate
  dedicated `rqt_chat` window when the simulator perspective is not in use
- `start_interaction_sim:=true` launches the official simulator-side webcam,
  face/person/emotion, visualization, expressive_face, rosbridge, and RQT
  support without duplicating `chatbot_llm`, `dialogue_manager`, or
  `knowledge_core`
- `start_interaction_sim_ui:=true` also starts `ui_server` for the official UI
  server path
- `interaction_sim_gscam_config:=...` lets you override the webcam pipeline for
  home testing
- `debug_tts_action_name:=/debug/say` controls the debug-only TTS action used
  between `nao_say_skill` and `rqt_chat`
- the Docker images install `rqt_chat` from the `socialminds-ros-jazzy-rqt-chat`
  system package; the repo does not vendor that package in `src/`
- `KnowledgeCore` is consumed unchanged as the shared symbolic store; the
  migrated NAO stack reads it via `chatbot_llm` but does not write to it
- in the current Docker test path, `kb_msgs`, `knowledge_core`, `oro`,
  `interaction_sim`, and the simulator-side HRI/UI packages come from the
  official SocialMinds Jazzy apt feed
- reference-only upstream clones still belong under `ref_src/knowledge_sources/`
- the simulator integration is composed from `nao_chatbot` rather than the raw
  upstream `interaction_sim/simulator.launch.py` so the migrated stack does not
  start duplicate `chatbot_llm`, `dialogue_manager`, or `knowledge_core` nodes
