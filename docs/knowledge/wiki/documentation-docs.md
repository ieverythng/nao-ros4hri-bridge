# Documentation — docs

# Documentation Module

This module contains the reference documentation for the NAO ROS4HRI bridge project. The documents cover architecture, runtime contracts, launch profiles, integration scope, and planning direction for the LLM-based dialogue and planning system.

## Document Index

| Document | Purpose | When to Read |
|----------|---------|--------------|
| `demo_status_and_contracts.md` | Current runtime contracts, message schemas, and end-to-end data flow | Understanding what is live today and how components communicate |
| `launch_profiles.md` | Launch file arguments, profile matrix, and execution commands | Running the stack in different configurations |
| `thesis_planning_handoff.md` | High-level architecture summary and thesis direction | Planning next implementation stages |
| `current_workflow.md` | Compact runtime snapshot and responsibility split | Quick reference for active data paths |
| `node_interactions_map.md` | Node graph and architecture notes | Understanding component relationships |
| `asr_vosk_setup.md` | ASR pipeline setup, parameters, and debugging | Working with speech recognition |
| `knowledge_core_integration_scope.md` | Upstream package requirements and local changes | Understanding KB integration and migration scope |
| `ollama_chatbot_architecture.md` | chatbot_llm backend pipeline and modules | Working with the LLM backend |
| `planner_supervisor_phase_c_handoff.md` | Planner supervisor validation checklist | Testing planner integration |
| `ros4hri_fork_delta_ledgers.md` | Fork vs upstream deltas for migrated packages | Understanding what changed from upstream |
| `wme_planner_alignment.md` | WME responsibilities and planner boundaries | Avoiding overlap between WME and planner work |
| `nao_camera_vlm_research.md` | Camera and VLM/VLA integration path | Future vision-language work |

## Architecture Overview

The system implements a grounded dialogue and planning architecture where perception is converted into symbolic state before reaching the LLM:

```mermaid
flowchart LR
    subgraph Perception
        CAM[Camera] --> DET[Detector Backend]
        DET --> SG[nao_scene_grounding]
    end
    
    subgraph Knowledge
        SG -->|/kb/revise| KC[knowledge_core]
        KC -->|/kb/query| KBS[kb_skills]
    end
    
    subgraph Dialogue
        USER[User Speech/Text] --> DM[dialogue_manager]
        DM --> CL[chatbot_llm]
        KBS --> CL
    end
    
    subgraph Planning
        CL -->|/planner/request| PL[planner_llm]
        PL -->|/intents| ORCH[nao_orchestrator]
    end
    
    subgraph Execution
        ORCH --> SAY[/nao/say]
        ORCH --> MOT[/skill/replay_motion]
        ORCH --> LOOK[/skill/look_at]
    end
```

## Key Concepts

### Grounded Reasoning Path

Raw perception does not flow directly into the LLM. Instead:

1. **Detector backend** produces raw object detections
2. **`nao_scene_grounding`** normalizes detections into symbolic facts
3. **`knowledge_core`** stores transient symbolic state
4. **`chatbot_llm`** queries grounded state through `kb_skills` before response generation

This separation means the LLM reasons over a bounded, symbolic world model rather than raw sensor data.

### Responsibility Split

| Package | Owns |
|---------|------|
| `dialogue_manager` | Dialogue lifecycle and speaking ownership |
| `chatbot_llm` | Grounded response generation and planner routing |
| `planner_llm` | Goal supervision, plan generation, replanning |
| `knowledge_core` | Symbolic world-state store |
| `kb_skills` | KB query and mutation boundary |
| `nao_scene_grounding` | Detector-to-KB bridge and `/scene/summary` |
| `nao_orchestrator` | Deterministic downstream execution |

### Planner Mode

When `chatbot_planner_mode_enabled=true`:

- `chatbot_llm` publishes execution-oriented turns to `/planner/request`
- `planner_llm` generates structured plans and emits `/intents`
- `nao_orchestrator` executes and publishes feedback on `/planner/execution_feedback`
- Direct `chatbot_llm -> /intents` path remains for non-planner fallback

## Reading Guide by Task

### Running the Stack

1. Start with `launch_profiles.md` for the profile matrix
2. Reference `demo_status_and_contracts.md` for topic and service contracts
3. Use `asr_vosk_setup.md` if ASR is needed

### Understanding the Architecture

1. Read `thesis_planning_handoff.md` for the high-level direction
2. Reference `node_interactions_map.md` for the node graph
3. Consult `demo_status_and_contracts.md` for detailed contracts

### Modifying the Codebase

1. Check `ros4hri_fork_delta_ledgers.md` to understand upstream vs local changes
2. Reference `knowledge_core_integration_scope.md` for KB integration boundaries
3. Consult `wme_planner_alignment.md` to avoid duplicating planner work

### Adding New Features

1. Review `thesis_planning_handoff.md` for target architecture
2. Check `demo_status_and_contracts.md` for existing contracts
3. Reference `nao_camera_vlm_research.md` for VLM/VLA integration path

### Debugging Runtime Issues

1. Use `launch_profiles.md` for correct launch arguments
2. Reference `demo_status_and_contracts.md` for validation commands
3. Check `asr_vosk_setup.md` for ASR-specific debugging

## Document Relationships

```
thesis_planning_handoff.md
├── demo_status_and_contracts.md (current state)
│   ├── current_workflow.md (runtime summary)
│   ├── node_interactions_map.md (node graph)
│   └── launch_profiles.md (how to run)
│
├── knowledge_core_integration_scope.md (KB integration)
│   └── ros4hri_fork_delta_ledgers.md (upstream deltas)
│
├── ollama_chatbot_architecture.md (LLM backend)
├── planner_supervisor_phase_c_handoff.md (planner testing)
├── wme_planner_alignment.md (WME boundaries)
│
└── nao_camera_vlm_research.md (future VLM path)
    └── asr_vosk_setup.md (ASR pipeline)
```

## Maintenance Notes

### Document Update Rules

- If launch defaults change, update `launch_profiles.md` in the same commit
- If runtime contracts change, update `demo_status_and_contracts.md`
- If architecture direction changes, update `thesis_planning_handoff.md`
- If upstream packages are rebased, update `ros4hri_fork_delta_ledgers.md`

### Canonical References

For planning and implementation, these are the primary references:

- **Current state**: `demo_status_and_contracts.md`
- **How to run**: `launch_profiles.md`
- **Thesis direction**: `thesis_planning_handoff.md`
- **VLM/VLA path**: `nao_camera_vlm_research.md`
- **WME/planner boundaries**: `wme_planner_alignment.md`

### Documents That Can Be Consolidated

The following overlap and can be merged when appropriate:

- `current_workflow.md` — overlaps with `demo_status_and_contracts.md`
- `node_interactions_map.md` — overlaps with both workflow and demo docs
- `knowledge_core_integration_scope.md` — valuable for migration context but less useful for day-to-day planning