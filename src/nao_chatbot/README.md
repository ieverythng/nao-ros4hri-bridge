# nao_chatbot

`nao_chatbot` is now the launch and operator-utility package for the migrated
NAO ROS4HRI stack.

It no longer owns chatbot execution or mission control logic. Those
responsibilities have moved to:

- `chatbot_llm`: chatbot backend contract
- `dialogue_manager`: canonical `/skill/chat`, `/skill/ask`, `/skill/say`
- `nao_orchestrator`: downstream intent dispatch

## Launch Files

Primary launch stacks:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

Simulator stack with emorobcare object detection + scene grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Simulator stack with ASR:

```bash
ros2 launch nao_chatbot nao_chatbot_sim_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

Real-robot camera + RViz:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=172.26.112.62
```

Real-robot camera + RViz + ASR:

```bash
ros2 launch nao_chatbot nao_chatbot_robot_asr.launch.py \
  nao_ip:=172.26.112.62
```

Real robot + object detection + scene grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=172.26.112.62
```

Real robot + simulator tools-only overlay:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=172.26.112.62
```

Fallback detector profile with `yolo_ros`:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=yolo_ros \
  scene_grounding_detector_topic:=/yolo/tracking \
  object_detection_model:=yolov8n.pt \
  object_detection_device:=cpu \
  nao_ip:=172.26.112.62
```

ASR-only utility profile:

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py
```

## Demo Priorities

Tomorrow's recommended order:

1. rebuild the overlay Docker image and validate the laptop-camera detector path
2. validate `nao_scene_grounding` plus `chatbot_llm` scene awareness in the
   simulator profile
3. switch to `nao_chatbot_robot.launch.py` for live robot camera, object
   detection, and `nao_look_at` follow-up wiring

Preferred rebuild:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Laptop-camera container run:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

Inside the container, the primary test command is:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Then move to the robot path:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=172.26.112.62 \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

`nao_look_at` does not need a separate demo-specific flag here: the robot
wrapper already starts it by default, and it now exists as the NAO
implementation of the upstream `interaction_skills/look_at` contract.

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

For the emorobcare object detection path specifically:

- keep `emorobcare_cv_object_detection` and `emorobcare_cv_msgs` built in the
  workspace when `object_detection_backend:=emorobcare_cv`
- keep `use_knowledge_base: false` in the detector package so
  `nao_scene_grounding` remains the single KB writer for detections
- keep `use_human_radar: false` unless you explicitly want that legacy path
- set `draw_image: true` if you want `/debug/object_detection` in RViz or
  `rqt_image_view`
- prefer `cpu` on the laptop unless you have already validated a GPU path
- on the real robot, feed the detector from `/camera/front/image_raw`

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
- the new default module entrypoint is `nao_chatbot_sim.launch.py`
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
  `hri_visualization` overlays enabled for the real-robot path and now remaps
  its camera input onto `/camera/front/image_raw`
- `start_rviz:=true` launches `rviz2` with the packaged `nao_robot` RViz config
  for robot-model, TF, raw camera, HRI overlay, and detector-debug validation
- `start_object_detection:=true` launches the selected detector backend; the
  default is `emorobcare_cv` and the supported fallback is `yolo_ros`
- `start_scene_grounding:=true` launches `nao_scene_grounding`, which bridges
  object detections into transient KnowledgeCore facts and `/scene/summary`
- the emorobcare object detection package and its message package are intentionally kept
  outside the monorepo history even when they are present under `src/`
- the upstream `interaction_skills` package is also expected to live under
  `src/` when needed, but it should stay read-only from this repo's point of
  view
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
- reference-only upstream clones still belong under `ref_src/knowledge_sources/`,
  but the active Docker rebuild path now expects `interaction_skills` directly
  under `src/`
- the simulator integration is composed from `nao_chatbot` rather than the raw
  upstream `interaction_sim/simulator.launch.py` so the migrated stack does not
  start duplicate `chatbot_llm`, `dialogue_manager`, or `knowledge_core` nodes

## Related Docs

- [../../README.md](../../README.md)
- [../../docs/launch_profiles.md](../../docs/launch_profiles.md)
- [../../docs/current_workflow.md](../../docs/current_workflow.md)
- [../../docs/knowledge_core_integration_scope.md](../../docs/knowledge_core_integration_scope.md)
- [../nao_scene_grounding/README.md](../nao_scene_grounding/README.md)
