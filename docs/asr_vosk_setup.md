# ASR Vosk Setup and Operations

Last updated: 2026-03-06

This repo now uses the imported package from `src/asr_vosk` (colleague-provided), not
`nao_chatbot/asr_vosk.py`, for runtime ASR.

## Active ASR Nodes

- `simple_audio_capture` (`src/simple_audio_capture`)
  - publishes `audio_common_msgs/AudioData` to `/laptop/microphone0` (configurable)
- `asr_vosk` lifecycle node (`src/asr_vosk/asr_vosk/node_vosk.py`)
  - subscribes to the microphone topic
  - publishes `hri_msgs/LiveSpeech` to `/humans/voices/anonymous_speaker/speech`

## Launch Profiles

- Full stack with ASR:
  - `ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py`
- ASR isolation only:
  - `ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py`

Both profiles run `asr_vosk` as a lifecycle node and automatically trigger:

1. `CONFIGURE`
2. `ACTIVATE`

## Minimum Working Command

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

This profile now defaults to push-to-talk enabled, so ASR will stay closed until you
open the gate.

## Key Parameters

### `asr_vosk` lifecycle node

- `asr_vosk_model_path`: absolute model path
- `asr_microphone_topic`: input `AudioData` topic
- `asr_sample_rate_hz`: recognizer sampling rate
- `asr_output_speech_topic`: output `LiveSpeech` topic
- `asr_speech_locale`: locale in `LiveSpeech`
- `asr_start_listening`: start listening after activation
- `asr_publish_partials`: publish incremental hypotheses
- `asr_min_final_chars`: drop short final hypotheses
- `asr_min_final_words`: drop final hypotheses with too few words
- `asr_min_final_confidence`: drop low-confidence final hypotheses
- `asr_ignore_single_token_fillers`: drop one-token fillers (`uh`, `um`, `huh`, ...)
- `asr_single_token_fillers_csv`: comma-separated filler list
- `asr_debug_log_results`: logs filter decisions and final hypotheses
- `asr_push_to_talk_enabled`: disables listening until an external Bool gate enables it
- `asr_push_to_talk_topic`: Bool topic used by push-to-talk mode

### `simple_audio_capture`

- `asr_audio_capture_source_type`: `pulsesrc` or `alsasrc`
- `asr_audio_capture_device`: optional device ID
- `asr_audio_capture_sample_rate`: capture sample rate
- `asr_audio_capture_channels`: capture channels
- `asr_audio_capture_sample_format`: usually `S16LE`
- `asr_audio_capture_chunk_size`: capture chunk size

### `dialogue_manager` ASR turn controls

- `dialogue_accept_incremental_speech`: keep `false` for Vosk turn routing
- `dialogue_ignore_user_speech_while_busy`: ignore new user ASR while a turn is already awaiting a reply
- `dialogue_user_turn_holdoff_sec`: buffer/merge consecutive ASR finals before forwarding one turn
- `dialogue_user_turn_min_chars`: minimum forwarded character length
- `dialogue_user_turn_min_words`: minimum forwarded word count

## Docker Requirements

You typically need:

1. Vosk model volume:
   - host path -> `/models` in container
2. Audio passthrough:
   - PulseAudio socket mount and env, and/or
   - `--device /dev/snd`

## Quick Verification

1. Confirm capture topic flow:
   - `ros2 topic hz /laptop/microphone0`
2. Confirm ASR subscription:
   - `ros2 topic info /laptop/microphone0 -v`
3. Confirm speech output:
   - `ros2 topic echo /humans/voices/anonymous_speaker/speech`
4. Confirm chatbot turn input:
   - `ros2 topic echo /chatbot/user_text`

## Systematic ASR Quality Debug

1. Verify raw audio is flowing:
   - `ros2 topic hz /laptop/microphone0`
2. Inspect final transcript confidence:
   - `ros2 topic echo /humans/voices/anonymous_speaker/speech`
   - watch `final` and `confidence` fields together
3. Enable ASR debug logs:
   - launch with `asr_debug_log_results:=true`
4. Increase filtering if filler outputs dominate:
   - `asr_min_final_confidence:=0.35`
   - `asr_min_final_words:=2`
5. Confirm turn coalescing in the dialogue bridge:
   - keep `dialogue_accept_incremental_speech:=false`
   - start with `dialogue_user_turn_holdoff_sec:=0.6`
   - keep `dialogue_ignore_user_speech_while_busy:=true`
6. Switch model for accuracy testing:
   - keep sample rate at `16000`
   - set `asr_vosk_model_path:=/models/<new-model-dir>`
7. Isolate ASR from full stack:
   - `ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py ...`

If the isolated profile still produces frequent filler finals with acceptable confidence,
the most likely root cause is model quality/domain mismatch (not ROS wiring).

## Push To Talk

Push-to-talk is supported directly in `asr_vosk` and the ASR-focused launch profiles now
enable it by default.

Example launch:

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

Recommended operator utility:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli
```

Interactive controls:

- `space` or `t`: toggle listening
- `o`: open listening
- `c`: close listening
- `q`: quit and close

One-shot commands:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli --open
ros2 run nao_chatbot asr_push_to_talk_cli --close
ros2 run nao_chatbot asr_push_to_talk_cli --pulse 5
```

Manual Bool publishing still works:

```bash
ros2 topic pub --once /asr_vosk/push_to_talk std_msgs/msg/Bool '{data: true}'
```

Disable listening:

```bash
ros2 topic pub --once /asr_vosk/push_to_talk std_msgs/msg/Bool '{data: false}'
```

This is intentionally terminal-first. A true press-and-release keyboard hold is not
portable in a plain shell without extra OS-specific hooks, so the supported operator UX is
toggle/open/close/pulse.

## Common Pitfalls

- Multiline launch commands missing a trailing `\`:
  - arguments after that line are ignored by shell.
- Wrong model path inside container:
  - verify `asr_vosk_model_path` exists from inside container.
- Pulse/ALSA not mounted:
  - capture node may run but publish unusable/empty audio.
