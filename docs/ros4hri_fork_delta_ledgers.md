# ROS4HRI Fork Delta Ledgers

Last updated: 2026-03-12

This document records the local-vs-upstream deltas for the upstream packages
that are being used as the source of truth for the refactor.

## Source Baselines

| Package | Fork workspace | Upstream commit | Purpose |
| --- | --- | --- | --- |
| `dialogue_manager` | `ref_src/dialogue_manager` | `862a5207527db8936549ef2ad328000180df2495` | Canonical dialogue runtime and communication-skill ownership |
| `chatbot_llm` | `ref_src/chatbot_llm` | `688d8db3c5ece9ce558a11becaa57cac5425e421` | Canonical chatbot backend contract |
| `asr_vosk` | `ref_src/asr_vosk` | `7dab6b31e54aaeaa78b5e2b575e48a5e86bb25b0` | Canonical ASR lifecycle/runtime contract |

During the migration, these fork workspaces are intentionally kept out of the
default build graph unless explicitly selected, while the active runtime still
resides in `src/`.

## Verified Fork Remotes

As of 2026-03-12, the local fork workspaces are wired for a GitHub-based fork
workflow while preserving the original source remotes for provenance:

| Package | Active workspace | Writable remote (`origin`) | Baseline remote (`upstream`) | Provenance remote | Status |
| --- | --- | --- | --- | --- | --- |
| `dialogue_manager` | `ref_src/dialogue_manager` | `https://github.com/ieverythng/dialogue_manager.git` | `https://github.com/ros4hri/dialogue_manager.git` | `git@gitlab.iiia.csic.es:socialminds/ros4hri/dialogue_manager.git` | Push verified; `juan-feat-1` published |
| `asr_vosk` | `ref_src/asr_vosk` | `https://github.com/ieverythng/asr_vosk.git` | `https://github.com/ros4hri/asr_vosk.git` | `https://gitlab.iiia.csic.es/socialminds/ros4hri/asr_vosk.git` | Push verified; `humble-devel` in sync |
| `chatbot_llm` | `src/chatbot_llm` | `https://github.com/ieverythng/nao_chatbot_llm.git` | `https://gitlab.iiia.csic.es/socialminds/ros4hri/chatbot_llm.git` | n/a | GitHub `main` reseeded from upstream commit `688d8db` |

## `dialogue_manager`

Upstream owns:

- lifecycle node model
- `/skill/chat`, `/skill/ask`, `/skill/say`
- chatbot action/service integration
- diagnostics, captions, waiting state, and speech handling

Current local delta in `src/dialogue_manager`:

- replaced upstream runtime with `nao_dialogue_manager.py`
- added custom `/chatbot/user_text`, `/chatbot/assistant_text`, and
  `/chatbot/dialogue_state` transport topics
- added local ASR turn buffering, de-duplication, and busy-guard logic
- added direct `/speech` mirroring and local say-skill dispatch helpers

Migration rule:

- upstream runtime remains authoritative
- local `/chatbot/*` transport must be externalized into NAO-specific glue or
  removed
- robot-specific speech execution should move to `/nao/say`, not be baked into
  the canonical `dialogue_manager` contract

## `chatbot_llm`

Upstream owns:

- lifecycle chatbot backend
- `chatbot_msgs/Dialogue` action
- `chatbot_msgs/DialogueInteraction` service
- intent emission through chatbot response payloads

Current local delta in `src/nao_chatbot`:

- exposes `/skill/chat` action server instead of the upstream backend contract
- centralizes Ollama transport, prompt packs, skill catalog, and history inside
  the local skill server runtime
- current mission-controller flow depends on the local `/skill/chat` contract

Migration rule:

- keep package name `nao_chatbot`
- move current Ollama logic behind an upstream-compatible backend node
- keep chat-skill compatibility only as a temporary bridge until
  `dialogue_manager` is cut over

## `asr_vosk`

Upstream owns:

- lifecycle ASR node
- `default_locale`, locale-management service/action, and PAL-style launch
- upstream audio inputs (`audio/channel0`, `audio/voice_detected`)
- `/robot_speaking` suppression contract

Current local delta in `src/asr_vosk`:

- uses custom `microphone_topic` and `output_speech_topic` parameters
- adds push-to-talk, result filtering, and simplified model path handling
- is wired around `simple_audio_capture` instead of upstream audio capture

Migration rule:

- restore upstream locale and audio contracts
- keep local result filtering/push-to-talk only if they can live as additive
  extensions or adapter behavior
- preserve `simple_audio_capture` only through an adapter/remap path during the
  transition
