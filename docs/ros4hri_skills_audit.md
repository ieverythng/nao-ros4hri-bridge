# ROS4HRI Skills Audit (Phase 1)

Date: 2026-02-24
Status: Historical reference (for current runtime view use `current_workflow.md`)

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

- `nao_rqt_bridge` migration completed into standalone `dialogue_manager` package
  runtime (`dialogue_manager_node`).
- Canonical skill interfaces now use `communication_skills`:
  - `/skill/say` -> `communication_skills/action/Say`
  - `/skill/chat` -> `communication_skills/action/Chat`
- Compatibility action endpoints removed:
  - `/skill/say_text_compat` removed
  - `/skill/chat_compat` removed
- `nao_skills` now only exposes posture action (`DoPosture`) for NAO-specific motion.
- New skill servers:
  - `nao_posture_bridge/say_skill_server.py` (`/skill/say`)
  - `nao_chatbot/chat_skill_server.py` (`/skill/chat`)
- New skill clients:
  - `nao_chatbot/say_skill_client.py` (used by `dialogue_manager`)
  - `nao_chatbot/chat_skill_client.py` (used by `mission_controller`)
- `mission_controller` backend mode supports canonical action-first chat
  (`/skill/chat`) with rule-based timeout/unavailable fallback.

## Phase 3 Update (2026-03-03)

Backend chat execution has been standardized around a modular ROS4HRI-friendly
architecture while preserving canonical skill interfaces:

- `/skill/chat` remains `communication_skills/action/Chat`
- `ollama_chatbot_node` now uses two LLM stages:
  1. response generation (`verbal_ack`)
  2. intent extraction (`user_intent`)
- `mission_controller` remains the single action authority for:
  - publishing assistant text
  - publishing normalized intents
  - dispatching posture via `/skill/do_posture`

Traceability update:

- `turn_id` is propagated in user text payloads, chat goal configuration, and
  chat result payloads, enabling cross-node log tracing for each interaction.
