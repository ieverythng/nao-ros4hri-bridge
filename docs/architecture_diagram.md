# NAO Chatbot Stack Architecture

This diagram reflects `src/nao_chatbot/launch/nao_chatbot_stack.launch.py` and current node wiring.

```mermaid
flowchart LR
    %% Inputs
    UI[rqt_chat UI]
    ASR[laptop_asr_node\noptional]
    LIVE[/humans/voices/anonymous_speaker/speech\nhri_msgs/LiveSpeech/]

    %% Core nodes
    BRIDGE[nao_rqt_bridge_node]
    MC[mission_controller_node]
    OLLAMA[ollama_responder_node\noptional]
    POSTURE[nao_posture_bridge_node\noptional]

    %% Chatbot topics
    UTXT[/chatbot/user_text\nstd_msgs/String/]
    INTENT[/chatbot/intent\nstd_msgs/String/]
    BTREQ[/chatbot/backend/request\nstd_msgs/String/]
    BTRESP[/chatbot/backend/response\nstd_msgs/String/]
    ATXT[/chatbot/assistant_text\nstd_msgs/String/]
    PCMD[/chatbot/posture_command\nstd_msgs/String/]

    %% Robot-facing
    TTSA[[/tts_engine/tts\ntts_msgs/action/TTS]]
    SPEECH[/speech\nstd_msgs/String/]
    NAOQI[(NAOqi services\nALRobotPosture)]
    DRIVER[naoqi_driver / TTS stack\noptional]
    ROBOT[(NAO robot)]

    %% Input fan-in
    UI --> LIVE
    ASR --> LIVE
    LIVE --> BRIDGE
    BRIDGE --> UTXT

    %% Mission controller fan-out
    UTXT --> MC
    MC --> INTENT
    MC --> ATXT
    MC --> BTREQ
    BTREQ --> OLLAMA
    OLLAMA --> BTRESP
    BTRESP --> MC
    MC --> PCMD
    PCMD --> POSTURE
    POSTURE --> NAOQI

    %% Speech output path
    ATXT --> BRIDGE
    BRIDGE --> TTSA
    BRIDGE --> SPEECH
    TTSA --> DRIVER
    SPEECH --> DRIVER
    DRIVER --> ROBOT
```

## Notes

- `rules` mode: `mission_controller_node` publishes `/chatbot/assistant_text` directly.
- `backend` mode: `mission_controller_node` uses `/chatbot/backend/request` and `/chatbot/backend/response` through `ollama_responder_node`.
- Posture commands (`stand`, `sit`, `kneel`) are published on `/chatbot/posture_command` and executed by `nao_posture_bridge_node` through NAOqi services.
- `rqt_chat` and `naoqi_driver` are launch-time optional components; `laptop_asr_node` is optional and can share the same `LiveSpeech` topic with UI input.
