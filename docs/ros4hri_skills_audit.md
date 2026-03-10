# ROS4HRI Skills Audit

Last updated: 2026-03-09
Status: current snapshot

This file records how the local workspace currently maps onto ROS4HRI skill conventions.

## Canonical Skill Interfaces In Use

### Communication skills

- `/skill/say` -> `communication_skills/action/Say`
- `/skill/chat` -> `communication_skills/action/Chat`

### NAO-specific skills

- `/skill/do_posture` -> `nao_skills/action/DoPosture`
- `/skill/do_head_motion` -> `nao_skills/action/DoHeadMotion`

## Runtime Ownership

| Runtime concern | Package |
|---|---|
| dialogue turn bridge | `dialogue_manager` |
| mission/chat orchestration | `nao_chatbot` |
| posture/head/say execution | `nao_skill_servers` |
| NAO skill interfaces only | `nao_skills` |
| local ASR | `asr_vosk` + `simple_audio_capture` |

## Confirmed Current Posture Flow

1. `mission_controller` sends `DoPosture` to `/skill/do_posture`
2. `posture_skill_server` executes directly through NAOqi when available
3. if direct NAOqi is unavailable, it publishes `/chatbot/posture_command`
4. `nao_posture_bridge_node` executes that fallback topic path

## Confirmed Current Head-Motion Flow

1. `mission_controller` sends `DoHeadMotion` to `/skill/do_head_motion`
2. `head_motion_skill_server` validates ranges and speed
3. it publishes `naoqi_bridge_msgs/msg/JointAnglesWithSpeed` to `/joint_angles`
4. `naoqi_driver` applies the motion to the robot head joints

## Confirmed Current Speech Flow

1. `dialogue_manager` receives assistant text
2. it dispatches `/skill/say`
3. `say_skill_server` forwards to `/tts_engine/tts`
4. optional `/speech` publication remains available for NAO/TTS compatibility and ASR suppression workflows

## Local ASR Status

- `simple_audio_capture` publishes `audio_common_msgs/msg/AudioData`
- `asr_vosk` publishes `hri_msgs/msg/LiveSpeech`
- application launches default to final-only forwarding into `dialogue_manager`
- push-to-talk is supported through `/asr_vosk/push_to_talk`

## Package Boundary Decision

The old `nao_posture_bridge` package name no longer matched reality once head motion and say execution were added. That runtime package is now `nao_skill_servers`.

This keeps the boundary clean:

- `nao_skills` remains interface-only
- `nao_skill_servers` contains executable servers and bridges

## Notes

- Compatibility action aliases are no longer part of the active stack.
- The legacy lifecycle/chatbot-based `dialogue_manager` runtime has been removed; the maintained runtime is `dialogue_manager_node`.
