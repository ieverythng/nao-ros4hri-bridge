# nao_say_skill

`nao_say_skill` is the NAO-specific speech execution package introduced during
the ROS4HRI refactor.

It intentionally exposes `/nao/say`, not the canonical `/skill/say`
endpoint. The canonical communication skill remains owned by
`dialogue_manager`; this package is the robot-side execution hook.

## ROS API

- action: `/nao/say`
- type: `communication_skills/action/Say`
- compatibility TTS action: `/tts_engine/tts`
- downstream TTS action: optional `tts_backend_action_name`
- direct speech topic fallback: `/speech`
- debug mirror topic: `/debug/nao_say/speech`

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `say_action_name` | `/nao/say` | NAO-specific speech action endpoint |
| `tts_action_name` | `/tts_engine/tts` | TTS action endpoint exposed for `dialogue_manager` |
| `tts_backend_action_name` | `""` | Optional downstream TTS action server used behind the compatibility endpoint |
| `speech_topic` | `/speech` | Direct robot speech topic used for fallback execution |
| `debug_speech_topic` | `/debug/nao_say/speech` | Debug-only speech mirror |
| `default_language` | `en-US` | Default language used for outgoing TTS goals |
| `default_volume` | `1.0` | Default TTS volume |
| `tts_server_wait_sec` | `0.5` | Wait time before considering the TTS server unavailable |
| `fallback_to_speech_topic` | `true` | Publish to `/speech` if no downstream TTS action is available |
| `also_publish_debug_topic` | `true` | Mirror successful speech requests to the debug topic |
| `fallback_to_debug_topic` | `true` | Publish to the debug topic if TTS is unavailable |

## Launch

Standalone:

```bash
ros2 launch nao_say_skill nao_say_skill.launch.py
```

As part of the migration stack:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
```

## Expected Behavior

- when the TTS action server is available, `/nao/say` forwards the request to it
- `dialogue_manager` now talks to this package through `/tts_engine/tts`
- when no downstream TTS action is configured, the package falls back to
  publishing the utterance on `/speech` for the robot driver
- the debug topic remains available alongside the robot speech path and is
  mirrored by `nao_chatbot/robot_speech_debug` into `rqt_console`
- this package never claims canonical `/skill/say`; that remains the
  responsibility of `dialogue_manager`

## Review Notes

- package-local config lives in `config/00-defaults.yml`
- lifecycle bring-up is provided by `launch/nao_say_skill.launch.py`
- the implementation is in `nao_say_skill/skill_impl.py`
- unit coverage currently focuses on goal metadata handling and small utility
  helpers; runtime forwarding is exercised through launch/integration smoke
  tests rather than mocked end-to-end TTS responses

## Notes

- when TTS is available, the skill forwards to the configured TTS action
- when TTS is unavailable, it can optionally fall back to debug-topic output
- the package is a lifecycle node and ships its own launch/config/module files
