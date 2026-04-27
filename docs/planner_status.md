# Planner Status

Last updated: 2026-04-27

## Current State

Implemented:

- `chatbot_llm` can route execution turns to `/planner/request`.
- `planner_llm` has a goal-keyed supervisor.
- `planner_llm` publishes executable `/intents` and planner dialogue acts.
- `nao_orchestrator` can execute structured `Intent.data.plan` steps in order.
- `nao_orchestrator` publishes `/planner/execution_feedback`.
- `requires` exists as step-level precondition metadata.
- `goal_text` is now carried by `chatbot_llm`, parsed by
  `planner_common.PlannerRequest`, and included in the `planner_llm` prompt
  payload.
- `user_text` has been removed from normal `chatbot_llm` planner requests and
  from the `planner_llm` prompt payload. `PlannerRequest` still parses it for
  legacy compatibility.
- Planner/chatbot Ollama calls now default to `qwen3.5:397b-cloud` with
  `think: false` in source and launch defaults.
- `/planner/request` still uses the ROS `Intent` envelope, so `priority` and
  `confidence` are visible in topic echoes. `chatbot_llm` now publishes a
  deterministic planner priority and a bounded confidence instead of leaving
  execution-routed requests at `0.0`.

Known weak spots:

- Planner ingress can still mix `goal_text`, `normalized_intents`,
  scene targets, and `requested_plan`; diagnostics should verify that
  `goal_text` is the primary objective and `requested_plan` remains optional.
- Multi-step completeness needs direct testing; unsupported-step filtering may
  hide partial-plan failures.
- Head motion can now be launched with explicit simulator/demo fallback
  parameters, but a rebuilt runtime is required before this affects the live
  graph.
- Pre/post condition validation should remain lightweight until the simple loop
  is proven.

## Diagnostic Goal

Before adding full demo nodes, run one minimal planner diagnostic:

1. Give `chatbot_llm` or a fixture a clear goal.
2. Avoid force-feeding a complete `requested_plan` unless testing the hint path.
3. Let `planner_llm` generate the plan from goal/intents/skill registry.
4. Let `nao_orchestrator` validate and execute a safe target.
5. Observe `/planner/execution_feedback`.

## Failure Classification

Use this table when recording diagnostic output.

| Failure class | Meaning | Example |
| --- | --- | --- |
| Routing | Topic/action/server wiring is wrong | no `/planner/request` subscriber |
| Contract/schema | Payload shape mismatches code expectations | missing `plan.steps`, unsupported step silently filtered |
| Planner/model | Model cannot form a valid plan from clear inputs | invalid JSON, wrong skill name |
| Execution/mock | Executor or skill result path fails | action server unavailable, mock returns failure |

## Minimal Refinement Completed In This Pass

Added `goal_text` through:

- `chatbot_llm.planner_request_adapter`
- `planner_common.PlannerRequest`
- `planner_llm` prompt payload
- focused tests and docs

Keep policy:

- `chatbot_llm`: intent declaration and planner routing.
- `planner_llm`: planning and supervision.
- `nao_orchestrator`: deterministic execution and feedback.

## Preconditions And Postconditions

Current:

- `requires` is normalized as a list of precondition labels.
- `nao_orchestrator` can report unmet preconditions through feedback.

Near-term:

- prove action success/failure and step ordering first.
- avoid building heavy world-state validation until the simple loop closes.

Future:

- add lightweight `expected_effects` only after the planner loop is stable.
- compare skill-owned postcondition checks vs orchestrator/world-model checks as
  a thesis design axis.

## Diagnostic Notes

Source-level validation completed on 2026-04-24:

- `planner_common` parses `goal_text` and legacy `goal` alias.
- `chatbot_llm` emits explicit `goal_text` when present and otherwise falls
  back to the cleaned user utterance.
- `planner_llm` includes `goal_text` in the provider prompt payload.
- `planner_llm` no longer includes `user_text` in the provider prompt payload.
- Minimal no-`requested_plan` diagnostic:
  - input goal: `inspect the cup and report completion`
  - `requested_plan_len`: `0`
  - planner result mode: `plan`
  - generated steps: `look_at` then `say`
  - classification: source-level contract path passed; live ROS routing and
    execution are still unverified in this shell.
- Focused tests passed:
  - `47 passed`: planner contracts, planner engine, supervisor, orchestrator
    intent rules.
  - `13 passed`: `chatbot_llm` planner request adapter.
- `python3 -m py_compile` passed for touched planner/chatbot/orchestrator
  Python entrypoints.

Live ROS diagnostic was not run in this shell because `ros2` is not available.
The GitNexus status check is also blocked here because `node` is missing.

Live container diagnostic completed after rebuild on 2026-04-24:

- Graph wiring confirmed:
  - `chatbot_llm` publishes `/planner/request`.
  - `planner_llm` subscribes `/planner/request` and publishes `/intents`.
  - `nao_orchestrator` subscribes `/intents` and publishes
    `/planner/execution_feedback`.
  - `planner_llm` publishes `/planner/dialogue_act`.
  - `dialogue_manager` subscribes `/planner/dialogue_act`.
  - `nao_scene_grounding` publishes `/scene/summary`; `chatbot_llm`
    subscribes it.
- Request probe with `goal_text`, `normalized_intents`, `scene_targets`,
  `grounded_context`, and no `user_text` reached `planner_llm`.
- Model-backed `inspect_scene` generated a two-step plan, but the first step
  used `look_at` args `{"target": "blueberry"}`. `nao_orchestrator` rejected it
  as `plan_invalid` because `look_at` currently requires `target_frame` or a
  reset policy. Classification: contract/schema failure in planner output.
- Rule-backed `head_look_left` generated `perform_motion` without
  `requested_plan`; orchestrator emitted `plan_accepted`, `step_started`, and
  then either `step_succeeded`/`plan_completed` or `step_failed` depending on
  whether `/joint_states` was fresh. Classification: planner contract passed;
  remaining instability is execution/sensor freshness.
- TF/log emergency check: simulator TF only had `base_link -> sellion_link ->
  camera`, while `hri_person_manager` defaulted to `reference_frame: map`.
  Live parameter override to `base_link` stopped the sampled lookupTransform log
  flood. Launch fix added in `nao_chatbot.interaction_sim_support`.

Live container diagnostic on 2026-04-27:

- Runtime model check:
  - `chatbot_llm` model: `qwen3.5:397b-cloud`.
  - `planner_llm` was still running `gpt-oss:120b-cloud` until a live parameter
    override set it to `qwen3.5:397b-cloud`; source/launch defaults now use
    qwen for both.
  - current live container did not yet expose the new `think` parameters because
    those source changes require rebuild/relaunch.
- Rule-backed single-head-motion probe:
  - input goal: `look left`
  - request payload had `goal_text`, `normalized_intents: ["head_look_left"]`,
    empty `requested_plan`, and no `user_text`.
  - planner emitted a valid `perform_motion` step:
    `{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"}}`.
  - orchestrator/skill feedback ended in `step_failed` because head joint state
    did not change after publishing to `/joint_angles`.
  - classification: planner contract/routing passed; execution convergence
    failed in the live runtime.
- Composite qwen probe:
  - input goal: `look around to see if you find anyone, then report what you know`
  - `normalized_intents: ["inspect_people"]`, `scene_targets: ["person"]`,
    empty `requested_plan`.
  - no `/intents` plan was emitted; `planner_llm` emitted
    `/planner/dialogue_act` with `act: ask_clarification` and reason
    `planner output did not contain a valid executable plan`.
  - classification: planner/model insufficiency or prompt/schema constraint
    weakness for composite abstraction.
- Source-level validation on 2026-04-27:
  - `48 passed`: planner contracts, planner engine, supervisor, orchestrator
    intent rules.
  - `28 passed`: chatbot planner request adapter, turn engine, knowledge
    snapshot tests.
  - `6 passed`: `nao_replay_motion` unit tests in the ROS container.
  - `3 passed`: `nao_chatbot` launch-profile tests in the ROS container.
  - `python3 -m py_compile` passed for touched Python entrypoints.

Next ROS topic observations should be pasted here with:

- request payload
- planner output payload
- execution feedback payload
- dialogue act payload, if any
- failure classification
