# chatbot_llm Backend Architecture

Last updated: 2026-03-13

The old `ollama_chatbot` action server has been replaced by the migrated
`chatbot_llm` backend package. This document keeps the historical filename but
describes the current backend architecture.

## Scope

`chatbot_llm` is not a robot-control node. It only implements the backend
dialogue contract consumed by `dialogue_manager`:

- action: `<prefix>/start_dialogue` (`chatbot_msgs/action/Dialogue`)
- service: `<prefix>/dialogue_interaction` (`chatbot_msgs/srv/DialogueInteraction`)

Robot-side execution now happens downstream in `nao_orchestrator`.

## Internal Pipeline

```text
dialogue_manager request
  -> backend_config
  -> prompt_pack + prompt_builders
  -> skill_catalog
  -> chat_history
  -> ollama_transport
  -> intent_rules / intent_adapter
  -> DialogueInteraction response
```

## Main Modules

| Module | Purpose |
| --- | --- |
| `node_impl.py` | Lifecycle node, ROS contract, request handling |
| `turn_engine.py` | Two-stage response and intent production |
| `ollama_transport.py` | HTTP transport to the configured model server |
| `prompt_pack.py` | YAML prompt-pack loading |
| `prompt_builders.py` | Response and intent prompt construction |
| `skill_catalog.py` | Skill catalog extraction from installed packages |
| `chat_history.py` | History trimming and role/message translation |
| `intent_rules.py` | Rule fallback for deterministic intent extraction |
| `intent_adapter.py` | Conversion into the upstream `chatbot_msgs` contract |

## Runtime Contract

Steady-state flow:

1. `dialogue_manager` opens a dialogue via `start_dialogue`
2. `dialogue_manager` sends user turns through `dialogue_interaction`
3. `chatbot_llm` returns assistant text plus structured intent data
4. `dialogue_manager` publishes `/intents`
5. `nao_orchestrator` decides what robot-side skill to invoke

## Operational Notes

- the default backend is Ollama-compatible HTTP at
  `http://localhost:11434/api/chat`
- package naming stays `chatbot_llm` even though the local implementation uses
  the NAO prompt/history pipeline
- all ROS-facing behavior is kept aligned to the upstream backend contract so
  changes can be reviewed in the forked repo
