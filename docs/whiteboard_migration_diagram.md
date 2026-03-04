# NAO + ROS4HRI Backend Chatbot Whiteboard

Date: 2026-03-03

This board view reflects the current skills-first backend workflow, including:

- Ollama two-stage pipeline (`verbal_ack` then `user_intent`)
- mission controller intent publication/execution authority
- end-to-end `turn_id` traceability across dialogue, mission, and chat skill nodes
- say/posture skill routing and posture fallback bridge

## Legend

- `-->` topic pub/sub
- `==>` action goal/result
- `~~>` HTTP request/response
- `[canonical]` ROS4HRI-standard skill interface

## Current Backend Workflow (Skills + Intent + Ack + Trace)

```text
/humans/voices/anonymous_speaker/speech (hri_msgs/LiveSpeech)
    --> [dialogue_manager_node] [canonical]
        --> /chatbot/user_text (std_msgs/String JSON payload)
            {"text":"...","turn_id":"<id>"}
        --> /speech (std_msgs/String) [compat/asr-guard]

/chatbot/user_text
    --> [mission_controller_node]
        ==> /skill/chat (communication_skills/action/Chat) [canonical]
            role.configuration includes:
            {"user_message":"...","conversation_history":[...],"turn_id":"<id>"}
              |
              v
          [ollama_chatbot_node]
              stage 1 ~~> Ollama /api/chat (response model)
              stage 1 <~~ {"verbal_ack":"..."}
              stage 2 ~~> Ollama /api/chat (intent model)
              stage 2 <~~ {"user_intent":{"type":"..."},"intent_confidence":0.xx}
              ==> Chat.Result.role_results:
                  {
                    "assistant_response"/"verbal_ack",
                    "intent",
                    "user_intent",
                    "intent_source",
                    "intent_confidence",
                    "updated_history",
                    "turn_id"
                  }
        --> /chatbot/assistant_text (std_msgs/String JSON payload)
            {"text":"...","turn_id":"<id>"}
        --> /chatbot/intent (std_msgs/String)

Assistant speech branch:
    [dialogue_manager_node]
        ==> /skill/say (communication_skills/action/Say) [canonical]
              |
              v
          [say_skill_server_node]
              ==> /tts_engine/tts (tts_msgs/action/TTS)

Posture execution branch:
    [mission_controller_node]
        ==> /skill/do_posture (nao_skills/action/DoPosture)
              |
              v
          [posture_skill_server_node]
              direct --> NAOqi ALRobotPosture
              fallback --> /chatbot/posture_command (std_msgs/String)
                              |
                              v
                         [nao_posture_bridge_node]
                              --> NAOqi ALRobotPosture
```

## Mermaid: End-to-End Backend Flow

```mermaid
flowchart LR
    U["User Speech\n/humans/voices/.../speech"] --> DM["dialogue_manager_node"]
    DM -->|/chatbot/user_text\n{text,turn_id}| MC["mission_controller_node"]
    DM -->|/speech (compat)| Speech["/speech topic"]

    MC -->|Chat goal /skill/chat\nrole.configuration:{...,turn_id}| CS["ollama_chatbot_node"]
    CS -->|stage 1 /api/chat\nresponse model| OLL1["Ollama"]
    OLL1 -->|JSON: verbal_ack| CS
    CS -->|stage 2 /api/chat\nintent model| OLL2["Ollama"]
    OLL2 -->|JSON: user_intent| CS
    CS -->|Chat.Result.role_results\nassistant + intent + turn_id| MC

    MC -->|/chatbot/assistant_text\n{text,turn_id}| DM
    MC -->|/chatbot/intent| IntentTopic["/chatbot/intent"]

    DM -->|Say goal /skill/say| SAY["say_skill_server_node"]
    SAY -->|/tts_engine/tts| TTS["TTS engine"]

    MC -->|Posture goal /skill/do_posture| PS["posture_skill_server_node"]
    PS -->|direct| NAOQI["NAOqi ALRobotPosture"]
    PS -->|fallback /chatbot/posture_command| PB["nao_posture_bridge_node"]
    PB --> NAOQI
```

## Mermaid: Intent/Ack Decision Path

```mermaid
flowchart TD
    A["/chatbot/user_text"] --> B{"mission mode"}
    B -->|rules| R1["detect_intent(text)\nbuild_rule_response(intent)"]
    B -->|backend| C1["send /skill/chat goal\nwith turn_id"]

    C1 --> C2["stage 1: generate verbal_ack"]
    C2 --> C3["stage 2: extract user_intent"]
    C3 --> C4["normalize intent\n(intent_rules.normalize_intent)"]
    C4 --> C5["mission_controller publishes:\n/chatbot/assistant_text (ack)\n/chatbot/intent"]
    C5 --> C6{"posture intent?"}
    C6 -->|yes| C7["dispatch /skill/do_posture"]
    C6 -->|no| C8["no posture action"]

    R1 --> R2["publish /chatbot/assistant_text + /chatbot/intent"]
    R2 --> R3{"posture intent?"}
    R3 -->|yes| R4["dispatch /skill/do_posture"]
    R3 -->|no| R5["no posture action"]
```
