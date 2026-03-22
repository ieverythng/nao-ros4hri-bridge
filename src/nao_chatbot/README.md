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

Real-robot camera + RViz validation:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  nao_ip:=172.26.112.62
```

Real robot + simulator tools-only overlay:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=172.26.112.62
```

Real robot + object detection + scene grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  scene_grounding_detector_topic:=/detected_objects \
  nao_ip:=172.26.112.62
```

Fallback detector profile with `yolo_ros`:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=yolo_ros \
  scene_grounding_detector_topic:=/yolo/tracking \
  object_detection_model:=yolov8n.pt \
  object_detection_device:=cpu \
  nao_ip:=172.26.112.62
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

## New Perception Nodes

The main launch surface now composes two new perception pieces for object-aware
demo flows:

- external detector backend
  - either `emorobcare_cv_object_detection` or `yolo_ros`
  - owns raw detections and detector-side debug images
- `nao_scene_grounding`
  - subscribes to detector outputs
  - refreshes transient object facts in `knowledge_core`
  - publishes `/scene/summary` as a compact JSON view of the grounded scene

Recommended ownership split:

- let detector packages focus on pixels, inference, and debug overlays
- let `nao_scene_grounding` stay the single bridge from detections into
  KnowledgeCore-facing symbolic facts
- let `chatbot_llm` keep consuming those grounded facts through its normal
  `knowledge_snapshot` path

The launch arguments you will use most often for this path are:

- `start_object_detection`
- `object_detection_backend`
- `object_detection_input_image_topic`
- `scene_grounding_detector_topic`
- `start_scene_grounding`
- `scene_grounding_allowed_labels`
- `scene_grounding_knowledge_lifespan_sec`

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
- `start_interaction_sim:=true` enables the local wrapper around the official
  simulator support launch without duplicating `chatbot_llm`,
  `dialogue_manager`, or `knowledge_core`
- `start_interaction_sim_perception:=true` launches the simulator-side webcam,
  face/person/emotion, visualization, expressive_face, and simulator TF path
- `start_interaction_sim_tools:=true` launches the simulator-side support tools
  such as rosbridge and optional `ui_server`; this is the intended mode to pair
  `rqt_human_radar` with `start_nao_robot:=true`
- `start_interaction_sim_ui:=true` adds `ui_server` on top of the simulator
  tools layer when those tools are enabled
- `start_nao_robot:=true` launches the packaged real-robot bring-up from the
  SocialMinds apt repository; it already includes `naoqi_driver`, the NAO front
  camera topics, and `hri_face_detect_yunet`
- `start_nao_robot_hri_visualization:=true` keeps the packaged
  `hri_visualization` overlays from `nao_robot` enabled for the real-robot path
- `start_rviz:=true` launches `rviz2` with the packaged `nao_robot` RViz config
  for robot-model, TF, and camera validation
- `start_object_detection:=true` launches the selected detector backend; the
  default is `emorobcare_cv` and the supported fallback is `yolo_ros`
- `start_scene_grounding:=true` launches `nao_scene_grounding`, which bridges
  object detections into transient KnowledgeCore facts and `/scene/summary`
- `start_knowledge_core:=true` launches `KnowledgeCore` when it is installed in
  the environment so `chatbot_llm` can query `/kb/query`
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

## Related Docs

- [../../README.md](../../README.md)
- [../../docs/launch_profiles.md](../../docs/launch_profiles.md)
- [../../docs/current_workflow.md](../../docs/current_workflow.md)
- [../../docs/knowledge_core_integration_scope.md](../../docs/knowledge_core_integration_scope.md)
- [../nao_scene_grounding/README.md](../nao_scene_grounding/README.md)
