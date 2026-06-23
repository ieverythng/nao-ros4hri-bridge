# Runtime Validation Report (Live Stack) — 2026-05-26

## Context

- Container: `nao_ros2`
- Launch profile used (user-provided, exact):
  - `ros2 launch nao_chatbot nao_chatbot_sim.launch.py ... start_fake_skills:=true start_interaction_trace_viewer:=true ...`
- Date/time window: 2026-05-26 (local Europe/Madrid)

## Critical Launch Finding

### 1) Fake-skill launch arg type crash (fixed)

- Symptom before fix:
  - Launch aborted with:
    - `Got dict for "mode_overrides_json"...`
- Root cause:
  - `fake_skill_mode_overrides_json` was YAML-coerced to dict instead of string.
- Fix applied:
  - `src/fake_skills/launch/fake_skills.launch.py`
  - `mode_overrides_json` now wrapped with:
    - `ParameterValue(LaunchConfiguration(...), value_type=str)`
- Container verification:
  - Patched file copied into `/home/ubuntu/ws/src/fake_skills/launch/fake_skills.launch.py`
  - Rebuilt with `colcon build --packages-select fake_skills`
  - Relaunch with the exact user command succeeded past fake-skill startup.

## Runtime Probe Matrix

## A) Full stack health (with user launch profile)

- `planner_llm` ready:
  - `[STACK READY] planner_llm ready ... auto_replan=True`
- `chatbot_llm` ready:
  - `[STACK READY] chatbot_llm configured ... planner_mode=True`
- `nao_orchestrator` active:
  - `nao_orchestrator active`
- `interaction_trace_viewer` active and subscribed to:
  - `/planner/request`, `/intents`, `/planner/execution_feedback`, `/planner/dialogue_act`, `/chatbot_llm/turn_trace`, `/fake_skills/events`
- `fake_skill_server` active:
  - scenarios available: `ambiguous_cup`, `area_person_found`, `path_blocked`, `social_wave_unavailable`

## B) Speech ingress check

- Published:
  - `/nao_chatbot/humans/voices/tracked` with `anonymous_speaker`
  - `/nao_chatbot/humans/voices/anonymous_speaker/speech` with `"hey robot"`
- Observed:
  - `dialogue_manager` received speech and issued chatbot request.
- Concern:
  - No corresponding `chatbot_llm/turn_trace` event appeared in the observation window for this turn.
  - This is a runtime seam risk for user-turn responsiveness under load.

## C) Planner request admission behavior

- Fixture request `goal_probe` was processed:
  - `planner_llm request received ... goal_id=goal_probe`
  - Resulted in `PLANNER_ACT ask_clarification` (expected for ambiguous goal text).
- Subsequent fixture requests (`goal_nav_1`, `goal_exec_1`) were visible on `/planner/request` in trace output, but no `planner_llm request received` log was emitted for them in the observation window.
- High-priority finding:
  - Potential request-admission/backpressure issue after entering `waiting_user` state.

## D) Fake-skill scenario verification (action-level, deterministic)

- Action type verified: `nao_skills/action/ScanScene`

### 1) `find_object` with `active_scenario_id=ambiguous_cup`

- Goal: target=`cup`
- Result: `ABORTED`
- Failure code in payload: `ambiguous`
- `scenario_id`: `ambiguous_cup`
- Expected: pass

### 2) `navigate_to` with `active_scenario_id=path_blocked`

- Goal: target=`kitchen`
- Result: `ABORTED`
- Failure code in payload: `path_blocked`
- `scenario_id`: `path_blocked`
- Expected: pass

### 3) `wave_greet` with `active_scenario_id=social_wave_unavailable`

- Goal: target=`person`
- Result: `ABORTED`
- Failure code in payload: `motion_unavailable`
- `scenario_id`: `social_wave_unavailable`
- Expected: pass

## Outstanding Risks / Gaps

1. Planner request admission inconsistency after first processed fixture request (`goal_probe`) must be debugged before demo lock.
2. Speech ingress reached `dialogue_manager`, but user-turn completion trace from `chatbot_llm` was not observed in this run.
3. Head-motion open-loop behavior is still runtime-parameter controlled (`allow_open_loop_without_joint_state=true`, `assume_success_on_convergence_timeout=true`) and is not yet wired to fake scenario policy controls in this profile.

## Recommended Immediate Next Checks

1. Instrument `planner_llm` request callback and supervisor state transitions to confirm why later `/planner/request` messages are not admitted.
2. Add a focused integration test or probe script for:
   - first request accepted
   - subsequent `new_goal` while `waiting_user`
   - expected supersede/queue behavior.
3. Add a scenario-driven control seam for head-motion failure injection (orchestrator/head-motion skill) and validate planner replan from that failure path.
