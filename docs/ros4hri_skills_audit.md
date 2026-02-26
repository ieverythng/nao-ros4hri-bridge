# ROS4HRI Skills Audit (Phase 1)

Date: 2026-02-24

## Scope

This audit compares the local migration with upstream ROS4HRI skill conventions
and speech stack packages hosted in:

- `socialminds/ros4hri/std_skills`
- `socialminds/ros4hri/motions_skills`
- `socialminds/ros4hri/communication_skills`
- `socialminds/ros4hri/interaction_skills`
- `socialminds/ros4hri/asr_vosk`
- `socialminds/ros4hri/rqt_chat`
- `socialminds/ros4hri/architecture_schemas`

## Confirmed Current Posture Flow

The posture request path is action-based at orchestration level, with optional
execution fallback:

1. `mission_controller` sends `DoPosture` goal to `/skill/do_posture`
2. `posture_skill_server` accepts goal
3. If direct NAOqi is unavailable, server publishes `/chatbot/posture_command`
4. `nao_posture_bridge_node` executes posture via NAOqi

This means action client/server wiring is correct, while low-level execution can
still route through topic fallback.

## Skill Manifest Convention (Upstream)

From upstream packages and schemas:

- Skill definition packages declare one or more `<skill content-type="yaml|json">`
  entries in `package.xml`.
- `datatype` is expressed as `package/interface/Type` (for example
  `communication_skills/action/Say`).
- Parameters are documented in `in`, `out`, and optionally `feedback`.
- Functional domains use schema enum values (for posture: `motions`).

Local `nao_skills` manifest follows this convention:

- `id: do_posture`
- `default_interface_path: /skill/do_posture`
- `datatype: nao_skills/action/DoPosture`
- input/output/feedback fields aligned with `DoPosture.action`

## Speech/TTS Upstream Baseline

### `communication_skills`

- Defines skills:
  - `/skill/say` (`communication_skills/action/Say`)
  - `/skill/chat` (`communication_skills/action/Chat`)
  - `/skill/ask` (`communication_skills/action/Ask`)
- Uses `std_skills/Meta`, `std_skills/Result`, `std_skills/Feedback`.

### `rqt_chat`

- Publishes user input as `hri_msgs/msg/LiveSpeech` on:
  - `/humans/voices/anonymous_speaker/speech`
- Exposes TTS action endpoint:
  - `/tts_engine/tts` (`tts_msgs/action/TTS`)

### `asr_vosk`

- Subscribes:
  - `audio/channel0` (`audio_common_msgs/AudioData`)
  - `audio/voice_detected` (`std_msgs/Bool`)
  - `/robot_speaking` (`std_msgs/Bool`)
- Publishes:
  - `/humans/voices/anonymous_speaker/speech` (`hri_msgs/LiveSpeech`)
  - `/humans/voices/anonymous_speaker/audio`
  - `/humans/voices/anonymous_speaker/is_speaking`
  - `/humans/voices/tracked`

## Local Workspace Changes Applied

- Cloned upstream `std_skills` into `src/std_skills`.
- Validated build:
  - `std_skills` and `nao_skills` build successfully.

## Current Build Gap

- `motions_skills` currently fails in this workspace due missing `moveit_msgs`:
  - `rosidl_generate_interfaces() ... dependency 'moveit_msgs' has not been found`

This is an environment dependency issue, not a skill-manifest issue.

## Phase 2 Update (2026-02-25)

The high-priority dialogue migration items are now implemented in this workspace:

- `nao_rqt_bridge` migrated to `dialogue_manager` architecture.
  - `src/nao_chatbot/nao_chatbot/dialogue_manager.py`
  - Backward-compat alias kept at
    `src/nao_chatbot/nao_chatbot/nao_rqt_bridge.py`
- New skill interfaces in `nao_skills`:
  - `action/SayText.action` (`/skill/say`)
  - `action/Chat.action` (`/skill/chat`)
- New skill servers:
  - `nao_posture_bridge/say_skill_server.py` (`/skill/say`)
  - `nao_chatbot/chat_skill_server.py` (`/skill/chat`)
- New skill clients:
  - `nao_chatbot/say_skill_client.py` (used by `dialogue_manager`)
  - `nao_chatbot/chat_skill_client.py` (used by `mission_controller`)
- `mission_controller` backend mode now supports action-first chat through
  `/skill/chat`, with legacy backend-topic fallback.
