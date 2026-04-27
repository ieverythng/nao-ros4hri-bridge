# simple_audio_capture

`simple_audio_capture` owns the local laptop microphone source used by the ASR
path. It captures audio through GStreamer and publishes ROS audio messages for
`asr_vosk`.

This is a utility package, not the final ROS4HRI perception abstraction.

## Public ROS Interfaces

| Interface | Type | Role |
| --- | --- | --- |
| `/laptop/microphone0` by default | `audio_common_msgs/msg/AudioData` | Raw audio chunks for ASR. |
| `/laptop/microphone0_stamped` by default | `audio_common_msgs/msg/AudioDataStamped` | Timestamped audio mirror. |
| `/audio_info` | `audio_common_msgs/msg/AudioInfo` | Latched audio metadata. |

The audio topic is configurable with the `audio_topic` parameter and launch
argument.

## Important Parameters

| Parameter | Default in config | Purpose |
| --- | --- | --- |
| `source_type` | `pulsesrc` | GStreamer source element, commonly `pulsesrc` or `alsasrc`. |
| `device` | `""` | Optional PulseAudio/ALSA device identifier. |
| `format` | `wave` | Audio format label. |
| `sample_format` | `S16LE` | PCM sample format. |
| `sample_rate` | `16000` | Capture sample rate. |
| `channels` | `1` | Channel count. |
| `depth` | `16` | Bit depth metadata. |
| `chunk_size` | `2048` | Bytes per published chunk from config. |
| `audio_topic` | `/laptop/microphone0` | Output topic. |

## Planner Contract Role

This package does not interact with the planner. It is the first ASR input node:
audio goes to `asr_vosk`, recognized speech goes to the dialogue layer, and
planner requests are produced later by `chatbot_llm`.

## Launch And Test

```bash
ros2 launch simple_audio_capture audio_capture.launch.py
```

Override the audio topic:

```bash
ros2 launch simple_audio_capture audio_capture.launch.py \
  audio_topic:=/laptop/microphone0
```

Targeted unit check:

```bash
python3 -m pytest -q src/simple_audio_capture/test/unit/test_audio_capture_node_unit.py
```

## Notes

- Local utility package.
- Used by `nao_chatbot_asr_only.launch.py`, `nao_chatbot_sim_asr.launch.py`,
  and `nao_chatbot_robot_asr.launch.py`.
- Keep this package focused on capture and publication; ASR filtering belongs
  in `asr_vosk`.
