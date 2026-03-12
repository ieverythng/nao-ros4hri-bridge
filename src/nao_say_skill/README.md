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

## Notes

- when TTS is available, the skill forwards to the configured TTS action
- when TTS is unavailable, it can optionally fall back to debug-topic output
- the package is a lifecycle node and ships its own launch/config/module files
