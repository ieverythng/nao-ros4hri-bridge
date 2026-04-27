# planner_common

`planner_common` is the shared contract package for the planner stack. It owns
JSON normalization helpers for planner ingress, plan envelopes, execution
feedback, dialogue acts, scene summaries, and enriched world-model snapshots.

## Owns

- `PlannerRequest`
- `ExecutionFeedback`
- `PlannerDialogueAct`
- `SceneSummary`
- `EnrichedSnapshot`
- plan payload builders and normalizers
- skill manifest loading helpers

It does not run ROS nodes and does not call robot skills or KnowledgeCore.

## Contract Role

This package is the source of truth for:

- `/planner/request` payload parsing
- `/intents` plan envelope shape
- `/planner/execution_feedback`
- `/planner/dialogue_act`
- `grounded_context`
- `/scene/summary` parsing
- bounded world-model text formatting

See `../../docs/contracts.md` for copyable runtime examples.

## Current Planner Request Fields

Important normalized fields:

- `request_id`
- `goal_id`
- `request_kind`
- `goal_text`
- `normalized_intents`
- `ack_text`
- `ack_mode`
- `scene_targets`
- `dialogue_context`
- `requested_plan`
- `grounded_context`
- `planner_mode`
- `interaction_mode`
- `dialogue_turn_id`

`goal_text` is the concise planner-facing objective. `user_text` remains parsed
for legacy payloads, but it is not part of the normal planner prompt path.

## Tests

```bash
PYTHONPATH=src/planner_common python3 -m pytest -q src/planner_common/test
```

When running with planner/orchestrator tests:

```bash
PYTHONPATH=src/planner_common:src/planner_llm:src/nao_orchestrator:src/kb_skills \
python3 -m pytest -q src/planner_common/test/test_contracts.py
```
