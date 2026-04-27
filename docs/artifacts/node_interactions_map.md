# Node Interactions Map

Last updated: 2026-04-09

This is now the short architecture map for the active stack.

For detailed runtime contracts and examples, see
[demo_status_and_contracts.md](./demo_status_and_contracts.md).
For the thesis-facing next-stage architecture, see
[thesis_planning_handoff.md](./thesis_planning_handoff.md).
For launch behavior, see [launch_profiles.md](./launch_profiles.md).

## Active Nodes

| Node | Package | Role |
| --- | --- | --- |
| `dialogue_manager` | `dialogue_manager` | entry point for speech/text dialogue |
| `chatbot_llm` | `chatbot_llm` | grounded response generation and planner handoff |
| `planner_llm` | `planner_llm` | goal supervision, plan generation, dialogue-act emission, and replanning |
| `knowledge_core` | upstream package | symbolic world state |
| `nao_scene_grounding` | `nao_scene_grounding` | detector-to-KB bridge and `/scene/summary` publisher |
| `nao_orchestrator` | `nao_orchestrator` | deterministic intent execution |
| `nao_say_skill` | `nao_say_skill` | robot speech execution |
| `nao_replay_motion` | `nao_replay_motion` | motion and posture execution |
| `nao_look_at` | `nao_look_at` | upstream-style gaze execution |
| `asr_vosk` | `asr_vosk` | transitional ASR path |

## Main Runtime Graph

```mermaid
graph LR
    speech["/humans/voices/*/speech"] --> dm["dialogue_manager"]
    dm --> chatbot["chatbot_llm"]
    chatbot -->|"/planner/request"| planner["planner_llm"]
    planner -->|"/planner/dialogue_act"| chatbot
    detector["detector_backend"] --> grounding["nao_scene_grounding"]
    grounding -->|/kb/revise| kb["knowledge_core"]
    kb -->|/kb/query via kb_skills| chatbot
    grounding -->|/scene/summary| summary["scene_summary_consumers"]
    dm -. direct mode .->|/intents| orch["nao_orchestrator"]
    planner -->|/intents| orch
    orch -->|/planner/execution_feedback| planner
    orch --> say["/nao/say"]
    orch --> motion["/skill/replay_motion"]
    orch --> head["/skill/do_head_motion"]
    orch --> look["/skill/look_at"]
```

## Architecture Notes

- `chatbot_llm` stays detector-agnostic and only consumes grounded symbolic state.
- planner mode is optional; in planner mode `chatbot_llm` emits `/planner/request`
  for execution turns, while the direct `chatbot_llm -> /intents` path still
  exists for non-planner fallback.
- `planner_llm` owns goal supervision, structured plan generation, replanning,
  and planner-side communication decisions, but it does not execute robot
  actions directly.
- `nao_scene_grounding` is the semantic bridge from raw detections into the KB.
- `nao_orchestrator` remains downstream-only and should evolve into the
  deterministic validation and execution layer for planner work.
- planner-facing execution feedback and dialogue acts are published separately
  so planner components can react without taking over skill dispatch or speech
  lifecycle.
- `kb_skills` is the intended long-term boundary for both KB reads and future KB
  mutations.
