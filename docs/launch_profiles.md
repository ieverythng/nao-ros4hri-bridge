# Launch Profiles

This repository provides profile launch files to reduce long command lines
during demos. Each profile includes `nao_chatbot_stack.launch.py` with
predefined defaults.

## Profiles

### `nao_chatbot_legacy.launch.py`

- Legacy posture path (topic-based)
- Skill posture path disabled
- Laptop Vosk ASR disabled
- Default `mission_mode`: `rules`

### `nao_chatbot_skills.launch.py`

- Skill posture path enabled (`/skill/do_posture`)
- Posture skill server enabled
- Topic bridge kept enabled for controlled fallback
- Backend-first posture timing enabled
- Laptop Vosk ASR disabled
- Default `mission_mode`: `backend`

### `nao_chatbot_skills_asr.launch.py`

- Same as `skills` profile
- Laptop Vosk ASR enabled by default
- Default Vosk model path: `/models/vosk-model-small-en-us-0.15`
- Default `mission_mode`: `backend`

## Common Usage

All profiles keep `mission_mode` overridable:

```bash
# rules mode
ros2 launch nao_chatbot nao_chatbot_skills.launch.py mission_mode:=rules

# backend mode
ros2 launch nao_chatbot nao_chatbot_skills.launch.py mission_mode:=backend
```

## Demo Commands

```bash
# 1) Legacy
ros2 launch nao_chatbot nao_chatbot_legacy.launch.py \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0

# 2) Skills
ros2 launch nao_chatbot nao_chatbot_skills.launch.py \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0

# 3) Skills + Vosk ASR
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0 \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

## Robot-Mic Ready Switch

To use a robot-mic ASR publisher instead of local laptop Vosk:

1. Disable laptop ASR in `skills_asr` profile.
2. Point bridge input to the robot ASR `LiveSpeech` topic.

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  laptop_asr_enabled:=false \
  bridge_input_speech_topic:=/humans/voices/anonymous_speaker/speech
```

`bridge_input_speech_topic` is configurable in the main stack and all profiles.
