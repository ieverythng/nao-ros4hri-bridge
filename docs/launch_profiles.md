# Launch Profiles

This repository provides profile launch files to reduce long command lines
during demos. Each profile includes `nao_chatbot_stack.launch.py` with
predefined defaults.

## Profiles

### `nao_chatbot_legacy.launch.py`

- Legacy posture path (topic-based)
- Skill posture path disabled
- Vosk ASR node (`asr_vosk_node`) disabled
- Default `mission_mode`: `rules`

### `nao_chatbot_skills.launch.py`

- Skill posture path enabled (`/skill/do_posture`)
- Posture skill server enabled
- Topic bridge kept enabled for controlled fallback
- Backend-first posture timing enabled
- Vosk ASR node (`asr_vosk_node`) disabled
- Default `mission_mode`: `backend`

### `nao_chatbot_skills_asr.launch.py`

- Same as `skills` profile
- Vosk ASR node (`asr_vosk_node`) enabled by default
- Default Vosk model path: `/models/vosk-model-small-en-us-0.15`
- ASR speech guard enabled by default (mutes microphone while robot is speaking)
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
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  asr_min_words:=2
```

Useful ASR stability knobs:

- `asr_block_duration_ms` (higher can reduce overflow warnings)
- `asr_status_warn_period_sec` (throttles repeated overflow logs)
- `asr_suppress_during_robot_speech` (keep `true` for turn-taking demos)

## Robot-Mic Ready Switch

`bridge_input_speech_topic` must be `hri_msgs/LiveSpeech`.

`naoqi_driver` robot microphone output is raw audio on `/audio`
(`naoqi_bridge_msgs/msg/AudioBuffer`) and speech recognition is available via
`/listen` (`naoqi_bridge_msgs/action/Listen`).

To use robot microphone with this stack, disable local Vosk and point bridge
to a `LiveSpeech` topic produced by an adapter node.

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_enabled:=false \
  bridge_input_speech_topic:=/humans/voices/anonymous_speaker/speech
```

`bridge_input_speech_topic` is configurable in the main stack and all profiles.
