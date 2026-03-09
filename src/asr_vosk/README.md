# asr_vosk

Runtime Vosk ASR package used by this workspace.

This package runs a ROS2 lifecycle node (`asr_vosk`) that subscribes to
`audio_common_msgs/AudioData` and publishes ROS4HRI `hri_msgs/LiveSpeech`.

## Runtime Contract

- Node: `asr_vosk` (lifecycle)
- Input topic:
  - `microphone_topic` (default: `/laptop/microphone0`)
- Output topic:
  - `output_speech_topic` (default: `/humans/voices/anonymous_speaker/speech`)
- Extra published topics:
  - `/humans/voices/tracked`
  - `/humans/voices/anonymous_speaker/audio`
  - `/humans/voices/anonymous_speaker/is_speaking`
  - `/diagnostics`

## Parameters

- `audio_rate` (int, default: `16000`)
- `model` (string, default: `/models/vosk-model-small-en-us-0.15`)
- `microphone_topic` (string, default: `/laptop/microphone0`)
- `start_listening` (bool, default: `true`)
- `output_speech_topic` (string, default: `/humans/voices/anonymous_speaker/speech`)
- `speech_locale` (string, default: `en_US`)

## Launch

```bash
ros2 launch asr_vosk asr_vosk.launch.py \
  model:=/models/vosk-model-small-en-us-0.15 \
  microphone_topic:=/laptop/microphone0
```

The launch file automatically transitions lifecycle state:

1. `CONFIGURE`
2. `ACTIVATE`

## Note on Models

This repository does not vendor Vosk model binaries in `src/asr_vosk`.
Provide models via mounted host path (for example `/models`) and set `model:=...`.
