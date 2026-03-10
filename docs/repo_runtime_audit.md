# Repository Runtime Audit

Last updated: 2026-03-09

This file tracks the current runtime boundaries after the dialogue-manager cleanup and package rename.

## Completed Cleanup

### Legacy `dialogue_manager` runtime removed

Removed legacy files that no longer matched the active stack:

- `dialogue_manager/chatbot_client.py`
- `dialogue_manager/dialogue.py`
- `dialogue_manager/manager_node.py`
- `dialogue_manager/skill_servers.py`
- `dialogue_manager/speech_handler.py`
- `dialogue_manager/start_manager.py`
- `dialogue_manager/tts_client.py`
- their matching legacy tests

The package now ships one runtime only:

- `dialogue_manager/nao_dialogue_manager.py`
- entrypoint: `dialogue_manager_node`
- standalone launch: `dialogue_manager.launch.py`

### `nao_posture_bridge` package renamed to `nao_skill_servers`

Reason:

- the package was no longer posture-only
- it hosts posture, head-motion, say, and the posture fallback bridge
- `nao_skills` remains interface-only and should not absorb runtime servers

Current package split:

- `nao_skills`: action definitions only
- `nao_skill_servers`: execution servers and bridges
- `nao_chatbot`: orchestration and backend logic

## Current Runtime Packages

| Package | Role |
|---|---|
| `dialogue_manager` | speech-to-turn bridge and say-skill dispatch |
| `nao_chatbot` | mission controller, chat skill server, Ollama transport, operator tools |
| `nao_skill_servers` | posture/head/say execution and fallback topic bridge |
| `asr_vosk` | local ASR lifecycle node |
| `simple_audio_capture` | microphone capture |
| `nao_skills` | NAO action interfaces |
| `communication_skills` | canonical communication skill interfaces |

## Intentionally Untouched

These local/vendor overlays remain in the workspace and were not modified:

- `src/motions_skills/`
- `src/std_skills/`

## Remaining Known Debt

- Historical docs may still mention the old package name where they are explicitly marked historical.
- `nao_posture_bridge_node` keeps its executable and ROS node name because it is still the posture fallback bridge inside the broader `nao_skill_servers` package.
