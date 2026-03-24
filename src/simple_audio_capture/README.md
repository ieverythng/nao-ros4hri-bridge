# simple_audio_capture

`simple_audio_capture` is the local microphone-source package used by the
workspace ASR path.

It captures audio through GStreamer and publishes `audio_common_msgs`
messages for `asr_vosk`.

## ROS API

- node: `simple_audio_capture`
- published audio topic:
  - `audio_topic` (default: `/laptop/microphone0`, type:
    `audio_common_msgs/msg/AudioData`)
- extra published topics:
  - `${audio_topic}_stamped` (`audio_common_msgs/msg/AudioDataStamped`)
  - `/audio_info` (`audio_common_msgs/msg/AudioInfo`, latched)

## Parameters

Defaults come from `config/speech_recognition.yaml`.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `source_type` | `pulsesrc` | GStreamer source element (`pulsesrc` or `alsasrc`) |
| `device` | `""` | Optional PulseAudio or ALSA device identifier |
| `format` | `wave` | Audio format label |
| `sample_format` | `S16LE` | PCM sample format |
| `sample_rate` | `16000` | Capture sample rate in Hz |
| `channels` | `1` | Number of audio channels |
| `depth` | `16` | Bit depth metadata |
| `chunk_size` | `2048` | Bytes per published chunk |
| `audio_topic` | `/laptop/microphone0` | Output topic used by `asr_vosk` |

## Launch

Standalone:

```bash
ros2 launch simple_audio_capture audio_capture.launch.py
```

Override the published topic:

```bash
ros2 launch simple_audio_capture audio_capture.launch.py \
  audio_topic:=/laptop/microphone0
```

## Notes

- `nao_chatbot_asr_only.launch.py`, `nao_chatbot_sim_asr.launch.py`, and
  `nao_chatbot_robot_asr.launch.py` use this package as the microphone source
  for `asr_vosk`
- this is still a local utility package, not the final upstream ROS4HRI ASR
  ingestion path
