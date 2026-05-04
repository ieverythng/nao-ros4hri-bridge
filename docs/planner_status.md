# Planner Status

Last updated: 2026-05-04

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
- Planner launch profiles now default to `qwen3-coder:480b-cloud`, while
  chatbot response generation defaults to `gemma4:31b-cloud`; both keep
  `think: false` in source and launch defaults.
- `/planner/request` still uses the ROS `Intent` envelope, so `priority` and
  `confidence` are visible in topic echoes. `chatbot_llm` now publishes a
  deterministic planner priority and a bounded confidence instead of leaving
  execution-routed requests at `0.0`.
- `scan` is now the planner-visible perception/composite skill contract. The
  previous demo-only `mock_scan_scene` naming has been removed from the planner
  registry.
- Planner execution feedback carries `result_summary`, which lets scan-style
  success summaries reach planner supervision without executor-authored speech.
- `step_succeeded` feedback now uses `status=succeeded` with
  `event_type=step_succeeded`.
- Planner skill prompt summaries now include params, aliases, observable
  success, safety flags, and adapter mapping.
- Unsupported model-generated planner steps no longer produce silent partial
  plans; if any generated step is rejected, the model output is treated as
  invalid and routed to clarification/fallback.
- `nao_chatbot` sim/robot/demo launch profiles share launch-native lifecycle
  events for chatbot/dialogue startup.

Known weak spots:

- Current demo path still has `chatbot_llm` publishing `/planner/request`
  directly. Supervisor feedback recommends moving this planner-gate ownership
  into `nao_orchestrator`; that is the next architectural migration, not a
  mixed-in demo hotfix.
- Dialogue-only intent cleanup is incomplete. `chatbot_llm` should avoid
  publishing greet/identity/wellbeing/help as executable intents, and
  `nao_orchestrator` should only keep a temporary ignore shim.
- Multi-step completeness still needs live ROS scenario testing after the
  stricter unsupported-step behavior.
- Ollama cloud model availability has shifted; `qwen3.5:*cloud` may require a
  paid tier or quota headroom depending on the account.
- Pre/post condition validation should remain lightweight until the simple loop
  is proven.

## Current Architecture Reference

See `docs/planner_architecture_current.md` for the consolidated architecture
status, remaining gaps, and model benchmark procedure.

## Model Candidate Probe

Use:

```bash
python3 scripts/benchmark_ollama_models.py --markdown \
  qwen3-coder:480b-cloud \
  gemma4:31b-cloud \
  glm-5.1:cloud \
  kimi-k2.6:cloud \
  deepseek-v4-flash:cloud \
  qwen3.5:cloud
```

Interpretation:

- prefer valid compact JSON over conversational quality;
- prefer supported skill names (`perform_motion`, `look_at`, `scan`);
- use latency only after JSON validity is acceptable;
- fall back to llama.cpp through the OpenAI-compatible planner provider if
  cloud quota/model access blocks the demo.

Latest quick result, 2026-05-04:

- preferred planner candidate: `qwen3-coder:480b-cloud`;
- fallback cloud candidate: `gemma4:31b-cloud`;
- launch defaults now use `planner_llm_model=qwen3-coder:480b-cloud` and
  `ollama_model=gemma4:31b-cloud`;
- blocked by subscription/quota: `qwen3.5:*cloud`, `glm-5.1:cloud`,
  `kimi-k2.6:cloud`, `deepseek-v4-*cloud`;
- not recommended despite shallow JSON pass: `gpt-oss:120b-cloud`;
- weak fallback only: `gpt-oss:20b-cloud` and local `llama3.2:1b`.

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
  - `planner_llm` model: `qwen3.5:397b-cloud`.
  - both nodes use `think: false`.
  - chatbot timeout failures were reproduced with uncapped Qwen generation: a
    short greeting prompt ran to hundreds of generated tokens and exceeded the
    runtime timeout.
  - bounded Qwen probes with `num_predict: 64` returned in the low-single-digit
    seconds, so the fix is generation control rather than increasing inherent
    request latency.
  - source defaults now expose `response_max_tokens: 64` and
    `intent_max_tokens: 64`; launch also exposes
    `chatbot_response_max_tokens` and `chatbot_intent_max_tokens`.
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

Head/replay motion probe on 2026-04-27:

- Direct `/joint_angles` driver probe moved `HeadYaw` from near center to
  about `-0.44` rad and back to near zero. Classification: `naoqi_driver`
  command path works.
- Direct `/skill/do_head_motion` action succeeded twice and reported
  convergence in about `1.3-1.4s`. Classification: head-motion action server
  works with current fallback/convergence settings.
- Planner/orchestrator `head_look_left` probe produced `plan_accepted`,
  `step_started`, `step_succeeded`, and `plan_completed`. Classification:
  full head-motion software loop is closed.
- Direct `/skill/replay_motion` with `motion_name: sit` succeeded through the
  posture topic fallback. Previous replay failures were caused by result
  timeout pressure, not by planner policy.
- Source defaults and live params were adjusted to `20.0s` for replay/posture
  result waits, and direct motion dispatch in `nao_orchestrator` now waits on
  action results instead of treating goal submission as completion.

Next ROS topic observations should be pasted here with:

- request payload
- planner output payload
- execution feedback payload
- dialogue act payload, if any
- failure classification

Demo readiness update on 2026-05-05:

- Current recommended model is `gemma4:31b-cloud` for both `chatbot_llm` and
  `planner_llm`; `qwen3.5:cloud` is subscription-gated and
  `qwen3-coder:480b-cloud` timed out in live stack probes.
- Demo launch profiles now require LLM preflight and log `[STACK]`,
  `[LLM PREFLIGHT]`, and `[STACK READY]` markers so rqt shows selected models,
  subsystem enablement, lifecycle order, and readiness.
- Chatbot LLM timeout on execution-looking turns now preserves the planner seam:
  it speaks a short acknowledgement and publishes the original goal text to the
  planner instead of falling back to dialogue-only mode.
- Planner provider timeout now becomes a backend-unavailable failure dialogue
  act, not an `ask_clarification` act.
- `nao_orchestrator` now owns an optional planner gate on
  `/nao_orchestrator/planner_request`, forwarding accepted requests to
  `/planner/request` while rejecting duplicate or unsuperseded active goals.
- See `docs/demo_ready_handoff_2026-05-05.md` for the meeting/demo narrative
  and `docs/nao_orchestrator_planner_gate_handoff.md` for the deferred
  orchestrator planner-gate migration notes.
