# Launch Profiles

Last updated: 2026-03-22

This is the quick execution guide for the active launch files in this repo.

## Profile Matrix

| Launch file | What it enables by default | What it disables by default |
|---|---|---|
| `nao_chatbot_ros4hri_migration.launch.py` | Migrated ROS4HRI stack (`chatbot_llm`, `dialogue_manager`, `nao_orchestrator`, `nao_say_skill`, `nao_replay_motion`, `nao_look_at`) | Local ASR |
| `nao_chatbot_ros4hri_with_asr.launch.py` | Migrated ROS4HRI stack plus `simple_audio_capture` and `asr_vosk` | None |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR pipeline (`simple_audio_capture` + `asr_vosk`) | Dialogue/mission/chat/robot nodes |

Old `nao_chatbot_stack`, `nao_chatbot_skills`, and
`nao_chatbot_skills_asr` wrappers were removed during the ROS4HRI cleanup so
the workspace only exposes the migrated launch surface.

## Show Arguments

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_ros4hri_with_asr.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py --show-args
```

## Common Execution Commands

### Primary migrated stack

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
```

With robot driver:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_naoqi_driver:=true \
  nao_ip:=172.26.112.62
```

With packaged real-robot camera + RViz path:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  nao_ip:=172.26.112.62
```

With packaged real-robot camera + interaction_sim tools-only overlay:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_nao_robot:=true \
  start_rviz:=true \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=172.26.112.62
```

With real-robot object grounding through the colleague detector:

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

With fallback object grounding through `yolo_ros`:

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

### Primary migrated stack with ASR

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_with_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
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
- the local ASR stack can run either standalone or under
  `nao_chatbot_ros4hri_with_asr.launch.py`, but it is still the local
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
  overlays from `nao_robot` enabled on the real-robot path.
- `start_rviz`: launch `rviz2` with the packaged `nao_robot` RViz config for TF
  and robot-camera validation.
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

- Use `start_interaction_sim:=true` for home webcam testing.
- Use `start_nao_robot:=true start_rviz:=true start_interaction_sim:=false` for
  pure real-robot TF/camera validation.
- Use `start_nao_robot:=true start_object_detection:=true
  start_scene_grounding:=true object_detection_backend:=emorobcare_cv` for the
  current end-to-end object-grounding demo path.
- Use `start_nao_robot:=true start_interaction_sim:=true
  start_interaction_sim_perception:=false start_interaction_sim_tools:=true` to
  combine the real robot camera path with simulator-side operator tools such as
  `rqt_human_radar`.

## Colleague Detector Preflight

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
