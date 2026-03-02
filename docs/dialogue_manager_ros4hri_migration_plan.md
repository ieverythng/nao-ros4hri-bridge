# Dialogue Manager ROS4HRI Migration Plan

Date: 2026-02-26
Owner: nao-ros4hri-bridge

## Summary

This document tracks the staged migration to a ROS4HRI-standard dialogue stack:

- standalone `dialogue_manager` package
- canonical communication actions from `communication_skills`
- NAO-specific posture skill remaining in `nao_skills`
- migration compatibility layers removed after cutover validation

## Progress Model

- `Planned`
- `In Progress`
- `Verified`
- `Deferred`

## Source Provenance

- `dialogue_manager` imported from:
  - repo: `https://gitlab.iiia.csic.es/socialminds/ros4hri/dialogue_manager.git`
  - tag: `0.3.1`
  - commit: `862a5207527db8936549ef2ad328000180df2495`
- `communication_skills` imported from:
  - repo: `https://gitlab.iiia.csic.es/socialminds/ros4hri/communication_skills.git`
  - tag: `1.5.0`
  - commit: `9f3b35122ff35a0f18942e9e1b95475b14af2428`

## Sections

### Section 1: Source Intake + Scaffold (Foundation)

Status: `Verified`

Goals:
- import upstream `dialogue_manager` into `src/dialogue_manager`
- import upstream `communication_skills` into `src/communication_skills`
- scaffold package structure with `rpk` where available
- no runtime behavior change yet

Completed:
- imported both upstream packages into `src/`
- pinned and recorded upstream commit hashes

Open items:
- `rpk` scaffolding step
  - current state: `rpk` not available in local shell nor container baseline image
  - temporary fallback: upstream package intake performed manually
  - follow-up: run equivalent `rpk` generation once tool is available in environment

### Section 2: Canonical Skill Interfaces

Status: `Verified`

Target:
- `/skill/say` canonical datatype -> `communication_skills/action/Say`
- `/skill/chat` canonical datatype -> `communication_skills/action/Chat`
- `/skill/do_posture` remains `nao_skills/action/DoPosture`

Tasks:
- update say/chat clients + servers to canonical action types
- remove temporary compatibility adapters for legacy `nao_skills` say/chat callers
- keep `nao_skills` focused on NAO-specific posture interface

Completed:
- `chat_skill_client` now sends `communication_skills/action/Chat` goals and
  preserves compatibility callback fields for mission controller.
- `chat_skill_server` now serves canonical `/skill/chat` only.
- `say_skill_server` now serves canonical `/skill/say` only.
- removed deprecated compatibility interfaces from `nao_skills` (`Chat.action`,
  `SayText.action`) and kept `DoPosture.action` as the only local NAO skill API.

Done criteria:
- runtime introspection resolves `/skill/say` and `/skill/chat` to canonical types

### Section 3: Dialogue Manager Cutover

Status: `Verified`

Tasks:
- launch standalone `dialogue_manager` package node
- merge NAO-specific behavior needed by current stack:
  - LiveSpeech input path
  - `/chatbot/user_text` forwarding
  - `/skill/say` dispatch
  - `/speech` compatibility publish/fallback
  - dialogue state publication

Completed:
- added `dialogue_manager/dialogue_manager/nao_dialogue_manager.py` with
  preserved NAO bridge turn-taking behavior.
- added `dialogue_manager/dialogue_manager/say_skill_client.py` and utilities.
- wired `dialogue_manager_node` entrypoint in standalone package setup.
- switched launch stack to run `dialogue_manager_node` from package `dialogue_manager`.
- removed `nao_chatbot.dialogue_manager` and `nao_rqt_bridge` alias wrappers.

Done criteria:
- turn-taking flow works with existing `mission_controller`

### Section 4: Mission + Compatibility Alignment

Status: `Verified`

Tasks:
- keep `mission_controller` in `nao_chatbot`
- validate orchestration with canonical say/chat + posture flows
- keep operational fallback only where execution reliability requires it

Completed:
- smoke launch validation passed for:
  - `nao_chatbot_skills.launch.py`
  - `nao_chatbot_skills_asr.launch.py`
- both maintained profiles show expected node readiness with canonical action endpoints:
  - `/skill/chat` (`communication_skills/action/Chat`)
  - `/skill/say` (`communication_skills/action/Say`)
  - `/skill/do_posture` (`nao_skills/action/DoPosture`)
- removed legacy backend-topic runtime path (`ollama_responder`) from launch stack;
  backend mode now uses canonical chat action with rules fallback only.

Done criteria:
- skills profiles run cleanly with canonical action APIs

### Section 5: Dead Code / Docs / Standard Sweep

Status: `Verified`

Tasks:
- remove/de-emphasize proven deprecated paths
- fix stale docs references
- refresh whiteboard architecture with:
  - old
  - transition
  - target

Completed:
- removed dead `nao_chatbot` nodes:
  - `ollama_responder.py`
  - `ollama_node.py`
- consolidated Ollama runtime on one executable:
  - `ollama_chatbot_node` (chat action server backend)
- added explicit node module entrypoint:
  - `nao_chatbot/ollama_chatbot.py`
- removed legacy backend-topic launch controls from stack and profile wrappers.
- removed compatibility wrappers and entrypoints:
  - `nao_chatbot/nao_chatbot/dialogue_manager.py`
  - `nao_chatbot/nao_chatbot/nao_rqt_bridge.py`
  - `nao_chatbot_legacy.launch.py`
  - `nao_chatbot` entrypoints `nao_rqt_bridge_node`, `dialogue_manager_node`,
    and `laptop_asr_node`
- removed deprecated ASR alias argument `laptop_asr_enabled` from launch files.

Done criteria:
- docs and launch/runtime graph are aligned and current

## Validation Matrix (Per Section)

1. Build checks:
- `colcon build` for touched packages each section

2. Runtime checks:
- `ros2 action list -t` for action names/types
- e2e: LiveSpeech -> user_text -> chat -> assistant_text -> say
- posture intent path in direct and fallback modes

3. Compatibility checks:
- skills profiles operational

Current validation limits in this execution environment:
- `colcon build` validation is complete for touched packages.
- direct `ros2 action list -t` introspection is constrained by CLI daemon socket
  permissions in this environment.
- runtime validation was performed via short launch smoke tests and startup logs
  confirming canonical endpoints/types and profile readiness.

## Default Decisions in This Phase

- implementation uses container network path for upstream intake
- phase scope limited to `dialogue_manager` + `communication_skills` (beyond existing packages)
- backward compatibility layers removed after final sweep verification
