# NAO + ROS4HRI Migration Whiteboard Diagram

Date: 2026-02-26

This diagram shows the current hybrid architecture (legacy + migrated paths)
and the intended post-migration target.

## Legend

- `-->` Topic flow
- `==>` Action goal/result flow
- `[legacy]` Existing compatibility path kept during migration
- `[new]` Migrated skill/action path

## Current Hybrid (What You Have Running)

```text
USER / ASR INPUT
  /humans/voices/anonymous_speaker/speech (hri_msgs/LiveSpeech)
      |
      v
  [dialogue_manager_node] [new]
      --> /chatbot/user_text (std_msgs/String)
      --> /chatbot/dialogue_state (std_msgs/String JSON)
      --> /speech (std_msgs/String) [compat + ASR guard]
      ==> /skill/say (nao_skills/action/SayText)
              |
              v
        [say_skill_server_node] [new]
            ==> /tts_engine/tts (tts_msgs/action/TTS)
            --> /speech (optional fallback publish)

  /chatbot/user_text
      |
      v
  [mission_controller_node] [hybrid]
      --> /chatbot/intent (std_msgs/String)
      --> /chatbot/assistant_text (std_msgs/String)
      ==> /skill/chat (nao_skills/action/Chat) [new]
              |
              v
        [chat_skill_server_node] [new]
            --> Ollama HTTP backend (internal)
      --> /chatbot/backend/request (std_msgs/String) [legacy fallback]
      <-- /chatbot/backend/response (std_msgs/String) [legacy fallback]
             ^
             |
      [ollama_responder_node] [legacy fallback]

  posture intent branch from mission_controller:
      ==> /skill/do_posture (nao_skills/action/DoPosture) [new]
              |
              v
        [posture_skill_server_node] [new]
            direct: NAOqi ALRobotPosture
            fallback --> /chatbot/posture_command (std_msgs/String)
                              |
                              v
                        [nao_posture_bridge_node] [legacy]
                            -> NAOqi ALRobotPosture

  Optional UI:
      [rqt_chat / rqt_gui_py_node]
      uses /tts_engine/tts + publishes speech UI feedback
```

## Old vs New Mapping (Quick Whiteboard Table)

```text
OLD (topic-first)                                NEW (skill/action-first)
-----------------------------------------------------------------------------------
nao_rqt_bridge monolith                          dialogue_manager_node + say_skill_client
/chatbot/backend/request -> ollama_responder     /skill/chat -> chat_skill_server_node
direct TTS/topic output in bridge                /skill/say -> say_skill_server_node
posture topic command only                        /skill/do_posture with topic fallback
```

## Target Post-Migration (Cleaner End State)

```text
LiveSpeech -> dialogue_manager -> /chatbot/user_text -> mission_controller
mission_controller ==> /skill/chat -> chat_skill_server -> /chatbot/assistant_text
dialogue_manager ==> /skill/say -> say_skill_server ==> /tts_engine/tts
mission_controller ==> /skill/do_posture -> posture_skill_server -> NAOqi

Removed/disabled in final target:
- /chatbot/backend/request + /chatbot/backend/response
- ollama_responder_node (topic backend path)
- nao_posture_bridge_node fallback path (if direct posture skill execution is reliable)
- nao_rqt_bridge_node alias
```
