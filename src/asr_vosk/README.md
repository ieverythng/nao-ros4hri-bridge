# asr_vosk

`asr_vosk` owns local offline speech recognition for the workspace ASR path. It
runs a ROS 2 lifecycle node that consumes microphone audio and publishes
ROS4HRI `hri_msgs/msg/LiveSpeech`.

This package is a local runtime package, but its public surface is ROS4HRI
speech-oriented and should remain compatible with the dialogue stack.

## Public ROS Interfaces

| Interface | Type | Role |
| --- | --- | --- |
| `/laptop/microphone0` by default | `audio_common_msgs/msg/AudioData` | Microphone audio input. |
| `/humans/voices/anonymous_speaker/speech` | `hri_msgs/msg/LiveSpeech` | Final and optional partial ASR output. |
| `/humans/voices/tracked` | `hri_msgs/msg/IdsList` | Anonymous voice tracking marker. |
| `/humans/voices/anonymous_speaker/audio` | `audio_common_msgs/msg/AudioData` | Voice audio mirror. |
| `/humans/voices/anonymous_speaker/is_speaking` | `std_msgs/msg/Bool` | Speaking state. |
| `/asr_vosk/push_to_talk` | `std_msgs/msg/Bool` | Optional listening gate. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Runtime diagnostics. |

## Important Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `audio_rate` | `16000` | Expected sample rate. |
| `model` | `/models/vosk-model-small-en-us-0.15` | Vosk model directory. |
| `microphone_topic` | `/laptop/microphone0` | Audio input topic. |
| `start_listening` | `true` | Begin listening after activation. |
| `output_speech_topic` | `/humans/voices/anonymous_speaker/speech` | LiveSpeech output topic. |
| `speech_locale` | `en_US` | LiveSpeech locale metadata. |
| `publish_partials` | `false` | Publish incremental hypotheses. |
| `min_final_chars` | `2` | Drop shorter final hypotheses. |
| `min_final_words` | `1` | Drop final hypotheses with fewer words. |
| `min_final_confidence` | `0.0` | Minimum average confidence. |
| `ignore_single_token_fillers` | `true` | Drop one-token fillers. |
| `single_token_fillers_csv` | `uh,um,hmm,huh,erm,ah,eh` | Filler list. |
| `debug_log_results` | `false` | Log ASR filtering decisions. |
| `push_to_talk_enabled` | `false` | Require explicit listening gate. |
| `push_to_talk_topic` | `/asr_vosk/push_to_talk` | Gate topic. |

## Planner Contract Role

This package does not talk to the planner directly. It feeds the dialogue side:
audio becomes `LiveSpeech`, `dialogue_manager`/`chatbot_llm` interpret the text,
and `chatbot_llm` publishes planner requests when needed.

## Launch And Test

```bash
ros2 launch asr_vosk asr_vosk.launch.py \
  model:=/models/vosk-model-small-en-us-0.15 \
  microphone_topic:=/laptop/microphone0
```

The launch file configures and activates the lifecycle node automatically.

Targeted unit check:

```bash
python3 -m pytest -q src/asr_vosk/test/unit/test_node_vosk_unit.py
```

## Notes

- Vosk model binaries are not vendored in this package; mount/provide them at
  runtime and set `model`.
- Standalone defaults listen immediately, while higher-level `nao_chatbot` ASR
  profiles may enable push-to-talk.
