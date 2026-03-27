# Current Workflow

Last updated: 2026-03-25

This is now the compact runtime snapshot for the active migrated stack.

For the full grounded-runtime contract, see
[demo_status_and_contracts.md](./demo_status_and_contracts.md).
For the thesis-facing architecture and next-stage planning direction, see
[thesis_planning_handoff.md](./thesis_planning_handoff.md).
For launch commands and profile toggles, see
[launch_profiles.md](./launch_profiles.md).

## Runtime Summary

```text
/humans/voices/*/speech
  -> dialogue_manager
  -> chatbot_llm
  -> /intents
  -> nao_orchestrator
  -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

Grounded scene path:

```text
detector backend
  -> nao_scene_grounding
  -> /kb/revise
  -> knowledge_core
  -> /kb/query via kb_skills
  -> chatbot_llm
```

Transitional ASR path:

```text
simple_audio_capture -> asr_vosk -> /humans/voices/anonymous_speaker/speech
```

## Responsibility Split

| Package | Owns |
| --- | --- |
| `dialogue_manager` | dialogue lifecycle and speaking ownership |
| `chatbot_llm` | grounded response and intent generation |
| `knowledge_core` | symbolic world state |
| `kb_skills` | KnowledgeCore query and mutation boundary |
| `nao_scene_grounding` | detector-to-KB grounding and `/scene/summary` |
| `nao_orchestrator` | deterministic downstream execution |

## Transitional Interfaces Still Present

- `/chatbot/posture_command`
- `/joint_angles`
- optional legacy `/chatbot/intent` bridge in `nao_orchestrator`

## Minimal Verification

```bash
ros2 action list -t
ros2 service list -t
ros2 lifecycle get /dialogue_manager
ros2 lifecycle get /chatbot_llm
ros2 lifecycle get /nao_orchestrator
```
