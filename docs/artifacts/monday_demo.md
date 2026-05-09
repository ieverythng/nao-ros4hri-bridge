# Monday Demo Checklist

Last updated: 2026-04-27

The Monday target is a visible, understandable planner loop. It is not a full
robot-motion demo.

## Success Criteria

- A clear planner request is visible on `/planner/request`.
- `planner_llm` emits an executable `/intents` plan.
- `nao_orchestrator` accepts the plan and publishes execution feedback.
- The feedback reaches `planner_llm`.
- Completion, failure, or clarification is visible through feedback or
`/planner/dialogue_act`.

## Baseline Commands

Planner-local:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

Observe in separate terminals:

```bash
ros2 topic echo /planner/request
ros2 topic echo /intents
ros2 topic echo /planner/execution_feedback
ros2 topic echo /planner/dialogue_act
```

Fixture path:

```bash
ros2 run planner_llm publish_fixture request
```

Optional injected feedback:

```bash
ros2 run planner_llm publish_fixture feedback
```

## What To Record

For each run, copy the important fields into `docs/planner_status.md`:

- `request_id`, `goal_id`, `goal_text` if present.
- `normalized_intents`.
- `requested_plan` length.
- emitted `plan.plan_id`, `plan_version`, and step list.
- feedback event sequence.
- final supervisor behavior.

## Expected Happy Path

```text
/planner/request
  -> planner_llm receives new goal
/intents
  -> nao_orchestrator validates plan
/planner/execution_feedback plan_accepted
/planner/execution_feedback step_started
/planner/execution_feedback step_succeeded
/planner/execution_feedback plan_completed
```

## Expected Failure Path

```text
/planner/execution_feedback step_failed
  -> planner_llm replans if retryable and budget remains
  -> or planner_llm emits /planner/dialogue_act
  -> or planner_llm marks goal failed
```

## Demo Talking Points

- The planner is not the executor.
- The chatbot declares intent and routes the goal.
- The planner supervises a goal and emits a plan.
- The orchestrator validates and executes deterministically.
- Execution feedback is the loop-closing contract.
- Deterministic skill backends are acceptable if they prove the loop without
  real robot risk; `scan` is a normal skill contract, not a demo-only gate.

## Current Limitation

Real head motion is not a reliable physical convergence proof in the current
live runtime: the planner can emit a valid `head_look_left` plan, but the action
may fail if `/joint_states` does not show the commanded change. Source now has
explicit simulator/demo fallback parameters for head motion:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  head_motion_allow_open_loop_without_joint_state:=true \
  head_motion_assume_success_on_convergence_timeout:=true
```

Use that as a software-loop demo fallback after rebuild/relaunch. Do not present
open-loop success as physical convergence.
