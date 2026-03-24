# Launch Profiles

Last updated: 2026-03-24

This is the quick execution guide for the active launch files in this repo.

## Profile Matrix

| Launch file | What it enables by default | What it disables by default |
|---|---|---|
| `nao_chatbot_sim.launch.py` | Migrated ROS4HRI stack + `interaction_sim` perception/tools + `rqt` | Local ASR, RViz |
| `nao_chatbot_sim_asr.launch.py` | Simulator stack plus `simple_audio_capture` and `asr_vosk` | RViz |
| `nao_chatbot_robot.launch.py` | Real robot camera, RViz, HRI overlays | Local ASR, simulator perception |
| `nao_chatbot_robot_asr.launch.py` | Real robot camera, RViz, HRI overlays, local ASR | Simulator perception |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR pipeline (`simple_audio_capture` + `asr_vosk`) | Dialogue/mission/chat/robot nodes |

## Show Arguments

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_sim_asr.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_robot.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_robot_asr.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py --show-args
```

## Common Execution Commands

### Simulator stack

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

With emorobcare object detection:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

### Simulator stack with ASR

```bash
ros2 launch nao_chatbot nao_chatbot_sim_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

### Robot stack

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=172.26.112.62
```

### Robot stack with ASR

```bash
ros2 launch nao_chatbot nao_chatbot_robot_asr.launch.py \
  nao_ip:=172.26.112.62
```

With simulator tools only on top of the robot path:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=172.26.112.62
```

With real-robot object grounding through emorobcare object detection:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=172.26.112.62
```

With fallback object grounding through `yolo_ros`:

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

### ASR-only profile

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

This profile runs only:

- `simple_audio_capture`
- `asr_vosk`

Push-to-talk helper:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli
```

## Topology-Changing Arguments

### ASR toggles

- `asr_vosk_enabled`: turns local Vosk on/off.
- `asr_audio_capture_enabled`: turns `simple_audio_capture` on/off.
- `asr_microphone_topic`: topic used between capture and ASR.
- `asr_publish_partials`: defaults to `false` in app launch surfaces.
- `asr_push_to_talk_enabled`: requires an explicit Bool gate before ASR listens.
- the local ASR stack can run either standalone, under
  `nao_chatbot_sim_asr.launch.py`, or under `nao_chatbot_robot_asr.launch.py`, but
  it is still the local
  `simple_audio_capture + asr_vosk` path rather than the final upstream ROS4HRI
  ASR contract.

### Migrated stack toggles

- `start_chatbot_llm`
- `start_knowledge_core`
- `start_dialogue_manager`
- `start_nao_orchestrator`
- `start_nao_say_skill`
- `start_nao_replay_motion`
- `start_nao_look_at`
- `start_naoqi_driver`
- `start_nao_robot`
- `start_nao_robot_hri_visualization`
- `hri_visualization_image_topic`
- `start_rviz`
- `start_interaction_sim`
- `start_interaction_sim_perception`
- `start_interaction_sim_tools`
- `start_interaction_sim_ui`
- `dialogue_manager_chatbot`
- `start_object_detection`
- `object_detection_backend`
- `start_scene_grounding`
- `object_detection_namespace`
- `object_detection_model`
- `object_detection_device`
- `object_detection_threshold`
- `object_detection_input_image_topic`
- `object_detection_image_reliability`
- `scene_grounding_detector_topic`
- `scene_grounding_summary_topic`
- `scene_grounding_allowed_labels`
- `scene_grounding_knowledge_lifespan_sec`
- `scene_grounding_knowledge_refresh_interval_sec`

### Robot integration toggles

- `start_naoqi_driver`: include/exclude `naoqi_driver`.
- `start_nao_robot`: include/exclude the packaged `nao_robot` bring-up. This is
  the preferred real-robot camera path because it already wires `naoqi_driver`,
  `/camera/front/*`, and `hri_face_detect_yunet`.
- `start_nao_robot_hri_visualization`: keep the packaged `hri_visualization`
  overlays enabled on the real-robot path.
- `hri_visualization_image_topic`: base image topic consumed by
  `hri_visualization`; the compressed transport of this topic drives
  `/image/hri_overlay`.
- `start_rviz`: launch `rviz2` with the packaged `nao_robot` RViz config for TF
  plus raw camera, HRI overlay, and detector debug validation.
- `start_object_detection`: launch the configured detector backend. The shipped
  options are `emorobcare_cv` and `yolo_ros`.
- `object_detection_backend`: choose which detector backend launch surface to
  activate.
- `object_detection_input_image_topic`: image topic passed into the detector.
- `scene_grounding_detector_topic`: detector output topic consumed by
  `nao_scene_grounding`.
- `start_scene_grounding`: start the detector-to-KnowledgeCore bridge node.
- `scene_grounding_summary_topic`: JSON scene summary output topic.
- `scene_grounding_allowed_labels`: comma-separated grounded object allowlist.
- `scene_grounding_knowledge_lifespan_sec`: fact lifetime written into
  KnowledgeCore for detector-derived objects.
- `start_interaction_sim`: enable the local interaction-sim wrapper launch.
- `start_interaction_sim_perception`: toggle the simulator webcam/person/emotion
  perception path on or off.
- `start_interaction_sim_tools`: toggle simulator-side tools such as rosbridge
  and `ui_server` on or off.
- `start_interaction_sim_ui`: start `ui_server` when simulator tools are
  enabled.
- `posture_command_topic`: temporary transition topic used by
  `nao_replay_motion` and `nao_orchestrator`.

Recommended split:

- Use `nao_chatbot_sim.launch.py` for home webcam testing.
- Use `nao_chatbot_robot.launch.py` for real-robot TF/camera/RViz validation.
- Use `nao_chatbot_robot_asr.launch.py` when the robot demo also needs local ASR.
- Add `start_object_detection:=true start_scene_grounding:=true
  object_detection_backend:=emorobcare_cv` to the sim or robot wrapper for the
  current end-to-end object-grounding demo path.
- Add `start_interaction_sim:=true start_interaction_sim_perception:=false
  start_interaction_sim_tools:=true` on the robot wrapper to combine the real
  robot camera path with simulator-side operator tools such as
  `rqt_human_radar`.

## Emorobcare Object Detection Preflight

Before using `object_detection_backend:=emorobcare_cv`, check:

- `emorobcare_cv_object_detection` and `emorobcare_cv_msgs` are both present in
  the workspace and built
- the detector package keeps `use_knowledge_base: false` so
  `nao_scene_grounding` stays the single writer of detector-derived KB facts
- `use_human_radar: false` unless you intentionally want that older path active
- `draw_image: true` if you want `/debug/object_detection` in RViz or
  `rqt_image_view`
- `yolo_device: cpu` is set when the laptop path should prefer stability over
  acceleration
- the current model is still biased toward labels such as blueberry, corn,
  pear, tomato, and zucchini, so mismatch with demo props is the first place to
  tune

## Docker Rebuild For Tomorrow

Preferred rebuild:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Laptop-camera object-detection path:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

Inside the container:

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

Notes:

- the overlay Dockerfile now uses `src/interaction_skills` directly and also
  rebuilds `nao_scene_grounding`
- `start_nao_look_at` stays enabled by default on the robot wrappers, so the
  next day's wiring work can focus on better target-frame usage rather than
  another launch toggle
- if `emorobcare_cv_msgs` is not present in the workspace, the emorobcare
  detector path will be skipped by launch
- `my_game_interface` is only required if you explicitly turn the detector's
  legacy `use_human_radar` integration back on; the default raw-detections path
  for `nao_scene_grounding` does not depend on it

## ASR Preflight In Docker

Before running ASR profiles in Docker, check:

- model path exists inside the container (`asr_vosk_model_path`)
- host audio is shared (PulseAudio socket or ALSA device)

See: [`asr_vosk_setup.md`](asr_vosk_setup.md)

## Related Docs

- [`asr_vosk_setup.md`](asr_vosk_setup.md)
- [`current_workflow.md`](current_workflow.md)
- [`node_interactions_map.md`](node_interactions_map.md)
- [`nao_camera_vlm_research.md`](nao_camera_vlm_research.md)

## Maintenance Rule

If launch defaults change, update this file in the same commit.

Lifecycle note:
- the migrated lifecycle nodes are brought up through event-driven transitions
  (`process start -> configure`, `inactive -> activate`) rather than fixed
  timer delays.
