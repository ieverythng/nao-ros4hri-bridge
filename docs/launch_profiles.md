# Launch Profiles

Last updated: 2026-03-06

This is the quick execution guide for the profile launch files in this repo.

## Profile Matrix

| Launch file | What it enables by default | What it disables by default |
|---|---|---|
| `nao_chatbot_stack.launch.py` | Full configurable stack surface | `asr_vosk_enabled:=false`, `asr_audio_capture_enabled:=false` |
| `nao_chatbot_skills.launch.py` | Skills-first backend flow (`/skill/chat`, `/skill/say`, `/skill/do_posture`) | Local ASR |
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

### Skills + ASR profile (capture node + topic backend)

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

This profile now defaults to `asr_push_to_talk_enabled:=true`.

Recommended stable-turn debug variant:

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  asr_publish_partials:=false \
  dialogue_user_turn_holdoff_sec:=0.6 \
  dialogue_ignore_user_speech_while_busy:=true
```

### ASR-only profile (recommended for debugging)

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

This profile now defaults to `asr_push_to_talk_enabled:=true`.

Push-to-talk debug variant:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli
```

This profile runs only:

- `simple_audio_capture`
- `asr_vosk` (lifecycle node from package `asr_vosk`)

### Force rules mode in any profile

```bash
ros2 launch nao_chatbot nao_chatbot_skills.launch.py mission_mode:=rules
```

## Topology-Changing Arguments

### ASR toggles

- `asr_vosk_enabled`:
  - turns local Vosk node on/off.
- `asr_audio_capture_enabled`:
  - turns `simple_audio_capture` on/off.
- `asr_microphone_topic`:
  - topic used between capture and ASR.
- `asr_publish_partials`:
  - defaults to `false` in app launch surfaces to avoid incremental turn churn.
- `asr_push_to_talk_enabled`:
  - requires an explicit Bool gate before ASR listens.
  - defaults to `true` in `nao_chatbot_skills_asr.launch.py` and `nao_chatbot_asr_only.launch.py`.
- `dialogue_user_turn_holdoff_sec`:
  - buffers and merges consecutive ASR finals before forwarding `/chatbot/user_text`.
- `dialogue_ignore_user_speech_while_busy`:
  - blocks overlapping user turns while the assistant is still processing/speaking.
  - already defaults to `true` in the ASR-enabled stack path.

### Chat behavior toggles

- `mission_mode`: `rules` or `backend`.
- `ollama_enabled`: enable/disable live Ollama calls in chat skill server.
- `ollama_intent_detection_mode`: `rules`, `llm`, or `llm_with_rules_fallback`.

### Robot integration toggles

- `start_naoqi_driver`: include/exclude NAOqi driver.
- `posture_skill_server_fallback_to_topic`: enable posture topic fallback.
- `say_skill_server_fallback_to_topic`: enable say topic fallback.

## ASR Preflight in Docker

Before running ASR profiles in Docker, check:

- model path exists inside container (`asr_vosk_model_path`)
- host audio is shared (PulseAudio socket or ALSA device)

See: [`asr_vosk_setup.md`](asr_vosk_setup.md)

## Related Docs

- [`asr_vosk_setup.md`](asr_vosk_setup.md)
- [`current_workflow.md`](current_workflow.md)
- [`node_interactions_map.md`](node_interactions_map.md)

## Maintenance Rule

If launch defaults change, update this file in the same commit.
