# Planner Supervisor Phase C Handoff

Last updated: 2026-04-09

This handoff is for the live laptop/container validation of the planner-supervisor stack on:

- bridge repo branch: `feat/TFM-LLM_planner`
- companion chatbot repo branch: `feat/planner_llm_hooks`

This document is intentionally Phase C only. WME is out of scope for this handoff.

## What is implemented

The current branch covers the planner-supervisor foundation through Phases A-C:

- Phase A: supervisor-ready contracts
  - `goal_id`
  - `request_kind`
  - `plan_version`
  - `communication_policy`
  - richer execution feedback
- Phase B: planner/dialogue separation
  - `/planner/dialogue_act`
  - planner-owned dialogue-act contract
  - `planner_llm` publishes dialogue acts separately from executable plans
- Phase C: precondition-aware execution supervision
  - lifecycle-oriented `/planner/execution_feedback`
  - retry/replan vs clarification/help/failure decisions
  - supervisor-local cancellation

## Current node roles

- `chatbot_llm`
  - owns user-facing acknowledgement and planner ingress publication
  - now sends supervisor-shaped planner requests
- `planner_llm`
  - owns goal supervision, planning, replanning, cancellation policy, and planner dialogue acts
- `nao_orchestrator`
  - owns validation, execution, and lifecycle execution feedback
- `dialogue_manager`
  - still owns speech/TTS realization and now subscribes to `/planner/dialogue_act`
    so planner-side asynchronous updates can be spoken without moving speaking
    ownership into `planner_llm`

## Important current limitation

`/planner/dialogue_act` is now fully implemented on the planner side, and the
dialogue side is minimally integrated in `dialogue_manager`.

What that means in practice:

- `chatbot_llm` now sends the richer supervisor ingress payload correctly.
- `planner_llm` now emits `/planner/dialogue_act`.
- `dialogue_manager` realizes planner dialogue acts through its existing TTS
  ownership seam.
- `chatbot_llm` still does **not** asynchronously realize planner dialogue
  acts on its own, which keeps planner progress/failure speech out of the
  chatbot backend for now.

So tomorrow’s live test should treat planner dialogue acts as a **first-class observable runtime output**:

- inspect them with `ros2 topic echo /planner/dialogue_act`
- or inspect them in `rqt_console`

This is enough to validate Phase B/C behavior while keeping speaking ownership
inside `dialogue_manager`.

## Expected end-to-end path

```text
dialogue_manager
  -> chatbot_llm
  -> /planner/request
  -> planner_llm
  -> /intents
  -> nao_orchestrator
  -> /planner/execution_feedback
  -> planner_llm

planner_llm
  -> /planner/dialogue_act
```

## Planner ingress shape expected tomorrow

`chatbot_llm` now publishes:

```json
{
  "request_id": "role:turn",
  "goal_id": "goal_role_turn",
  "parent_goal_id": "",
  "supersedes_goal_id": "",
  "request_kind": "new_goal",
  "user_text": "bring me the cup",
  "normalized_intents": ["bring_object"],
  "ack_text": "I will bring the cup.",
  "ack_mode": "say",
  "scene_targets": ["cup"],
  "dialogue_context": [],
  "grounded_context": {
    "knowledge_snapshot": {
      "summary_text": "..."
    },
    "scene_summary": {},
    "world_model_snapshot": {},
    "world_model_text": ""
  },
  "planner_mode": "default",
  "interaction_mode": "speech",
  "dialogue_turn_id": "role:turn"
}
```

## Topics to inspect

Core:

- `/planner/request`
- `/intents`
- `/planner/execution_feedback`
- `/planner/dialogue_act`

Useful context:

- `/scene/summary`
- `/diagnostics`

## Good live checks

### 1. New execution goal

Prompt:

- `look left`
- `stand up`
- `look at the cup`

Expected:

- `chatbot_llm` publishes `/planner/request`
- planner assigns `goal_id`
- planner emits `/intents`
- orchestrator emits `plan_accepted`
- orchestrator emits `step_started`
- orchestrator emits `step_succeeded` or `plan_completed`

### 2. Retry / replan

Use a request that likely fails or inject feedback manually.

Expected:

- orchestrator emits `step_failed`
- planner either republishes a higher `plan_version`
- or emits `/planner/dialogue_act` if retry budget is exhausted or more user input is needed

### 3. Clarification path

Force or inject failure feedback with:

- `needs_user_input=true`
- or `blocking=true` plus `unmet_preconditions`

Expected:

- no new `/intents`
- planner emits `/planner/dialogue_act`
- `act` should be `ask_clarification` or `ask_for_help`

### 4. Cancellation path

For now, the cleanest way to test cancellation is by publishing a supervisor-shaped planner request manually with:

```json
{
  "goal_id": "<existing_goal>",
  "request_kind": "cancel_request"
}
```

Expected:

- no new executable plan
- planner emits `notify_cancellation` on `/planner/dialogue_act`

## Local validation already completed

Bridge repo:

```bash
PYTHONPATH=src/planner_common:src/planner_llm:src/nao_orchestrator:src/kb_skills \
python3 -m pytest -q \
  src/planner_common/test/test_contracts.py \
  src/planner_llm/test/test_planner_engine.py \
  src/planner_llm/test/test_supervisor.py \
  src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py
```

Companion chatbot repo:

```bash
python3 -m pytest -q test/test_planner_request_adapter.py
python3 -m py_compile chatbot_llm/planner_request_adapter.py chatbot_llm/node_impl.py
```

## Suggested live commands

Bridge stack:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

Observe:

```bash
ros2 topic echo /planner/request
ros2 topic echo /planner/execution_feedback
ros2 topic echo /planner/dialogue_act
ros2 topic echo /intents
```

Manual planner fixtures:

```bash
ros2 run planner_llm publish_fixture request
ros2 run planner_llm publish_fixture feedback
```

## Tomorrow’s decision point

If the live test shows that `/planner/dialogue_act` is behaving well, the next
follow-up should be:

1. decide whether planner dialogue text should stay planner-owned or move
   behind a chatbot phrasing adapter
2. keep `dialogue_manager` as the speaking/TTS owner either way
3. only after that, reintroduce WME as Phase D
