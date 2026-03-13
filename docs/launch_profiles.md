# Launch Profiles

Last updated: 2026-03-13

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
- ASR is currently isolated in `nao_chatbot_asr_only.launch.py`; the full
  upstream ASR cutover is still pending.

### Migrated stack toggles

- `start_chatbot_llm`
- `start_dialogue_manager`
- `start_nao_orchestrator`
- `start_nao_say_skill`
- `start_nao_replay_motion`
- `start_nao_look_at`
- `start_naoqi_driver`
- `dialogue_manager_chatbot`

### Robot integration toggles

- `start_naoqi_driver`: include/exclude `naoqi_driver`.
- `posture_command_topic`: temporary transition topic used by
  `nao_replay_motion` and `nao_orchestrator`.

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
