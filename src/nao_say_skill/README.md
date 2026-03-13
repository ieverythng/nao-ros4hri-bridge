# nao_say_skill

`nao_say_skill` is the NAO-specific speech execution package introduced during
the ROS4HRI refactor.

It intentionally exposes `/nao/say`, not the canonical `/skill/say`
endpoint. The canonical communication skill remains owned by
`dialogue_manager`; this package is the robot-side execution hook.

## ROS API

- action: `/nao/say`
- type: `communication_skills/action/Say`
- downstream TTS action: `/tts_engine/tts` by default
- debug mirror topic: `/debug/nao_say/speech`

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `say_action_name` | `/nao/say` | NAO-specific speech action endpoint |
| `tts_action_name` | `/tts_engine/tts` | Downstream TTS server used for execution |
| `debug_speech_topic` | `/debug/nao_say/speech` | Debug-only speech mirror |
| `default_language` | `en-US` | Default language used for outgoing TTS goals |
| `default_volume` | `1.0` | Default TTS volume |
| `tts_server_wait_sec` | `0.5` | Wait time before considering the TTS server unavailable |
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
- when the TTS action server is missing, the node can still surface the text on
  `/debug/nao_say/speech`
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
