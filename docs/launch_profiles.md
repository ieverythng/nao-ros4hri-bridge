# Launch Profiles

Last updated: 2026-03-09

This is the quick execution guide for the profile launch files in this repo.

## Profile Matrix

| Launch file | What it enables by default | What it disables by default |
|---|---|---|
| `nao_chatbot_stack.launch.py` | Full configurable stack surface | `asr_vosk_enabled:=false`, `asr_audio_capture_enabled:=false` |
| `nao_chatbot_skills.launch.py` | Skills-first backend flow (`/skill/chat`, `/skill/say`, `/skill/do_posture`, `/skill/do_head_motion`) | Local ASR |
| `nao_chatbot_skills_asr.launch.py` | Skills flow + local ASR (`simple_audio_capture` + `asr_vosk`) | None (ASR enabled) |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR pipeline (`simple_audio_capture` + `asr_vosk`) | Dialogue/mission/chat/robot nodes |

## Show Arguments

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_skills.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py --show-args
```

## Common Execution Commands

### Skills-only profile

```bash
ros2 launch nao_chatbot nao_chatbot_skills.launch.py
```

This starts orchestration plus execution servers from package `nao_skill_servers`.

### Skills + ASR profile

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

Defaults that matter here:

- `asr_push_to_talk_enabled:=true`
- `asr_publish_partials:=false`
- `dialogue_ignore_user_speech_while_busy:=true`

Stable-turn debug variant:

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  dialogue_user_turn_holdoff_sec:=0.8
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

### Force rules mode in any profile

```bash
ros2 launch nao_chatbot nao_chatbot_skills.launch.py mission_mode:=rules
```

## Topology-Changing Arguments

### ASR toggles

- `asr_vosk_enabled`: turns local Vosk on/off.
- `asr_audio_capture_enabled`: turns `simple_audio_capture` on/off.
- `asr_microphone_topic`: topic used between capture and ASR.
- `asr_publish_partials`: defaults to `false` in app launch surfaces.
- `asr_push_to_talk_enabled`: requires an explicit Bool gate before ASR listens.
- `dialogue_user_turn_holdoff_sec`: buffers and merges consecutive ASR finals before forwarding `/chatbot/user_text`.
- `dialogue_ignore_user_speech_while_busy`: blocks overlapping user turns while the assistant is still processing/speaking.

### Chat behavior toggles

- `mission_mode`: `rules` or `backend`.
- `ollama_enabled`: enable/disable live Ollama calls in the chat skill server.
- `ollama_intent_detection_mode`: `rules`, `llm`, or `llm_with_rules_fallback`.

### Robot integration toggles

- `start_naoqi_driver`: include/exclude `naoqi_driver`.
- `posture_skill_server_fallback_to_topic`: enable posture topic fallback.
- `say_skill_server_fallback_to_topic`: enable say topic fallback.
- `head_motion_skill_server_enabled`: enable/disable head-motion execution.

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
