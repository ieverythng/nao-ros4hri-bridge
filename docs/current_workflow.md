# Current Workflow

Last updated: 2026-03-09

This is the live architecture reference for this repository.
It mirrors the launch behavior in:

- `src/nao_chatbot/launch/nao_chatbot_stack.launch.py`
- `src/nao_chatbot/launch/nao_chatbot_skills.launch.py`
- `src/nao_chatbot/launch/nao_chatbot_skills_asr.launch.py`
- `src/nao_chatbot/launch/nao_chatbot_asr_only.launch.py`

## Profiles At A Glance

| Launch profile | Purpose | ASR mode | Notes |
|---|---|---|---|
| `nao_chatbot_stack.launch.py` | Full configurable stack | Disabled by default | Base launch used by all profiles |
| `nao_chatbot_skills.launch.py` | Skills-first runtime | Disabled | Backend mode defaults, no local ASR |
| `nao_chatbot_skills_asr.launch.py` | Skills + local ASR | Topic-fed lifecycle ASR | Launches `simple_audio_capture` + `asr_vosk` |
| `nao_chatbot_asr_only.launch.py` | ASR-only isolation/debug profile | Topic-fed lifecycle ASR | Launches only `simple_audio_capture` + `asr_vosk` |

## End-To-End Runtime Flow

```text
simple_audio_capture
    -> /laptop/microphone0 (AudioData)
    -> asr_vosk
    -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
    -> dialogue_manager_node
        final-only extraction by default
        short holdoff + merge of consecutive ASR finals
        busy-guard against overlapping user turns
        -> /chatbot/user_text (JSON payload with turn_id)
        -> /speech (assistant speech mirror / ASR guard path)
    -> mission_controller_node
        rules mode:
            -> /chatbot/assistant_text
            -> /chatbot/intent
        backend mode:
            -> /skill/chat (communication_skills/action/Chat)
            -> ollama_chatbot_node (two-stage: response + intent)
            -> /chatbot/assistant_text
            -> /chatbot/intent
        posture route:
            -> /skill/do_posture (nao_skills/action/DoPosture)
            -> posture_skill_server_node
            -> fallback /chatbot/posture_command -> nao_posture_bridge_node
        head route:
            -> /skill/do_head_motion (nao_skills/action/DoHeadMotion)
            -> head_motion_skill_server_node
            -> /joint_angles
```

## Mermaid: Active Nodes

```mermaid
flowchart LR
    SAC["simple_audio_capture"] -->|AudioData| ASR["asr_vosk (lifecycle)"]
    ASR -->|LiveSpeech finals by default| DM["dialogue_manager_node"]

    DM -->|/chatbot/user_text| MC["mission_controller_node"]
    MC -->|/skill/chat| CHAT["ollama_chatbot_node"]
    CHAT -->|Chat result| MC

    MC -->|/chatbot/assistant_text| DM
    MC -->|/chatbot/intent| INTENT["/chatbot/intent"]

    DM -->|/skill/say| SAY["say_skill_server_node"]
    SAY -->|/tts_engine/tts| TTS["TTS engine"]

    MC -->|/skill/do_posture| PS["posture_skill_server_node"]
    MC -->|/skill/do_head_motion| HEAD["head_motion_skill_server_node"]
    PS -->|direct| NAOQI["NAOqi ALRobotPosture"]
    PS -->|fallback /chatbot/posture_command| PB["nao_posture_bridge_node"]
    PB --> NAOQI
    HEAD -->|/joint_angles| JOINT["naoqi_driver joint interface"]

    NAOQIDRIVER["naoqi_driver (optional)"] --> CAM1["camera/front/image_raw"]
    NAOQIDRIVER --> CAM2["camera/bottom/image_raw"]
```

## Mermaid: ASR Variants

```mermaid
flowchart TD
    A["nao_chatbot_skills_asr.launch.py"] --> C["simple_audio_capture -> /laptop/microphone0 (AudioData)"]
    K["ros2 run nao_chatbot asr_push_to_talk_cli"] --> P
    C --> D["asr_vosk lifecycle node subscribes microphone_topic"]
    P["optional /asr_vosk/push_to_talk Bool gate"] --> D
    D --> F["/humans/voices/anonymous_speaker/speech (LiveSpeech)"]
    F --> G["dialogue_manager_node input_speech_topic"]
    G --> H["holdoff/merge/dedupe before /chatbot/user_text"]
```

## Runtime Toggles That Change Topology

- `asr_vosk_enabled`: enables/disables `asr_vosk` lifecycle node.
- `asr_audio_capture_enabled`: enables/disables `simple_audio_capture` node.
- `asr_microphone_topic`: topic used between capture and ASR.
- `asr_speech_locale`: locale published in `LiveSpeech`.
- `asr_publish_partials`: publish incremental hypotheses or final-only output.
- `asr_push_to_talk_enabled`: enables explicit operator-gated listening.
- `asr_push_to_talk_topic`: Bool topic used by the operator CLI or manual publishers.
- `dialogue_user_turn_holdoff_sec`: buffers/merges consecutive ASR finals into one turn.
- `dialogue_ignore_user_speech_while_busy`: blocks overlapping user turns while a reply is pending.
- `mission_mode`: selects `rules` vs `backend` behavior in `mission_controller_node`.
- `use_head_motion_skill`: enables/disables `/skill/do_head_motion` dispatch from mission controller.
- `head_motion_skill_server_enabled`: enables/disables the head-motion execution server.
- `start_naoqi_driver`: includes/excludes `naoqi_driver`.
- `start_rqt_chat`: includes/excludes `rqt_chat`.

## Notes

- The execution package is now `nao_skill_servers`; the fallback bridge executable remains `nao_posture_bridge_node`.
- `naoqi_driver` already exposes front and bottom image topics, which is the correct starting point for future VLM/VLA work.

## Related Docs

- `ollama_chatbot_architecture.md`
- `repo_runtime_audit.md`
- `nao_camera_vlm_research.md`

## Source Of Truth Rule

If this file and launch code ever diverge, launch code is authoritative.
Update this file whenever launch defaults or node wiring changes.
