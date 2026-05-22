# Codex Handoff — Fake Skills Action Server for Base Planner + Neural Workbench Validation

> Archived reference notice: active execution tracking now lives in `docs/plans/ros4hri_integration_master_plan_2026-05-18.html`.
> Preserved archive copies: `docs/artifacts/plan_archive/fake_skills_codex_handoff.md` and `.html`.

**Target branch:** `feat(R)/Neural-Workbench` for research integration; backport-compatible with `feat/TFM-LLM_planner` where appropriate.  
**Primary repo:** `ieverythng/nao-ros4hri-bridge`  
**Purpose:** Implement a fake skill execution substrate that can be used by both the current planner stack and the Neural Workbench validation path.

## 1. Executive intent

The fake skills should not be treated as throwaway demo stubs. They are a core part of the validation and simulation layer.

They should allow the stack to test:

- abstract skill routing;
- ordered multi-step execution;
- controlled success/failure;
- replanning hooks;
- result payload propagation;
- AB registry mappings;
- dashboard/TUI traces;
- Neural Workbench candidate selection and energy/entropy scoring.

The current AB registry already exposes planner-visible AB=1 skills such as `scan`, `find_object`, `navigate_to`, `walk_to`, `perform_motion`, `look_at`, and `report_result`, and it includes AB=0 ROS/runtime seams such as `/planner/request`, `/intents`, `/planner/execution_feedback`, `/planner/dialogue_act`, `/scene/summary`, `query_kb`, `verify_evidence_payload`, `store_trace`, and `retrieve_memory`.

The fake skill substrate should therefore be implemented as a **real ROS2 package with deterministic action/service behaviour**, not as prompt-only logic.

## 2. Key architectural decision

Create one package and one shared fake skill engine:

```text
src/fake_skills/
  package.xml
  setup.py
  fake_skills/
    __init__.py
    engine.py
    contracts.py
    action_server.py
    scenario_store.py
    result_builders.py
    skills/
      navigate_to.py
      find_object.py
      wave_greet.py
      wave_at.py
      inspect_area.py
  config/
    fake_skill_scenarios.yaml
  launch/
    fake_skills.launch.py
  test/
    test_engine.py
    test_result_builders.py
    test_scenario_store.py
```

Expose either:

### Preferred: one generic action server plus compatibility wrappers

```text
/skill/fake/execute        # generic fake skill endpoint
/skill/fake/navigate_to    # optional wrapper/alias
/skill/fake/find_object    # optional wrapper/alias
/skill/fake/wave_greet     # optional wrapper/alias
```

The generic endpoint executes payloads of the form:

```json
{
  "skill": "navigate_to",
  "args": {"target": "kitchen"},
  "scenario": {"result_mode": "path_blocked"}
}
```

### Alternative: multiple action servers, one shared engine

```text
/skill/fake/navigate_to
/skill/fake/find_object
/skill/fake/wave_greet
/skill/fake/inspect_area
```

This is closer to classic ROS action-server semantics, but duplicates wiring. Use this only if the existing `nao_orchestrator` dispatch layer strongly prefers one endpoint per skill.

## 3. Why one fake skill engine

The goal is to avoid writing isolated fake skills that drift from each other. A single engine should own:

- result mode handling;
- delay/progress simulation;
- failure injection;
- typed result payload creation;
- trace labels;
- AB object lookup;
- scenario reset;
- deterministic seeding;
- dashboard event emission.

This makes it useful for both:

```text
base planner validation
Neural Workbench candidate/trace validation
```

## 4. Skill set to implement

### 4.1 `navigate_to`

**Registry role:** AB=1 executable semantic navigation skill now; supports AB=2/AB=3 strategy objects later.

Planner-visible examples:

```text
"go to the kitchen"
"move to the table"
"navigate to the charging station"
```

Input args:

```json
{
  "target": "kitchen",
  "location": "kitchen",
  "result_mode": "success"
}
```

Result modes:

```text
success
path_blocked
unknown_location
safety_disabled
timeout
fail_once
always_fail
unexpected_payload
```

Canonical result payload:

```json
{
  "skill": "navigate_to",
  "status": "failed",
  "target": "kitchen",
  "target_kind": "location",
  "target_found": false,
  "summary_text": "I could not navigate to the kitchen because the path is blocked.",
  "evidence": {
    "location": "kitchen",
    "simulated": true
  },
  "failure": {
    "code": "path_blocked",
    "message": "The simulated path to the kitchen is blocked.",
    "recoverable": true,
    "suggested_recovery": "ask_user_for_alternative_route"
  },
  "metadata": {
    "ab_object_id": "navigate_to",
    "fake": true,
    "duration_sec": 1.2
  }
}
```

AB interpretation:

```text
navigate_to as currently registered = AB=1 executable semantic navigation skill.
navigate_with_recovery = future AB=2/AB=3 macro/strategy candidate.
```

Do not prematurely register `navigate_to` itself as AB=3. Keep the executable skill AB=1 and allow Workbench traces to propose higher objects.

### 4.2 `find_object`

**Registry role:** AB=1 fake perception/search skill.

Planner-visible examples:

```text
"find the cup"
"look for the ball"
"find a person"
```

Input args:

```json
{
  "target": "cup",
  "target_kind": "object",
  "result_mode": "found"
}
```

Result modes:

```text
found
not_found
ambiguous
backend_unavailable
fail_once
always_fail
unexpected_payload
```

Canonical result payload:

```json
{
  "skill": "find_object",
  "status": "succeeded",
  "target": "cup",
  "target_kind": "object",
  "target_found": true,
  "summary_text": "I found one cup.",
  "evidence": {
    "objects": [
      {
        "id": "fake_cup_1",
        "label": "cup",
        "confidence": 0.91,
        "source": "fake_find_object"
      }
    ]
  },
  "failure": {},
  "metadata": {
    "ab_object_id": "find_object",
    "fake": true,
    "duration_sec": 0.8
  }
}
```

For `ambiguous`, return multiple candidates and mark recoverable:

```json
"failure": {
  "code": "ambiguous",
  "recoverable": true,
  "suggested_recovery": "ask_user_to_disambiguate"
}
```

### 4.3 `wave_greet`

**Registry role:** AB=1 executable social/motion skill.

This is the low-level reusable fake/real social gesture primitive. It can be fake initially and later mapped to an actual NAO motion/replay action.

Planner-visible examples:

```text
"wave"
"say hello with a wave"
"greet them"
```

Input args:

```json
{
  "style": "friendly",
  "hand": "right",
  "duration_sec": 2.0,
  "dry_run": true
}
```

Result modes:

```text
success
motion_unavailable
safety_disabled
fail_once
unexpected_payload
```

Canonical result payload:

```json
{
  "skill": "wave_greet",
  "status": "succeeded",
  "target": "",
  "target_kind": "social_gesture",
  "target_found": null,
  "summary_text": "I performed a friendly wave.",
  "evidence": {
    "gesture": "wave",
    "style": "friendly",
    "dry_run": true
  },
  "metadata": {
    "ab_object_id": "wave_greet",
    "fake": true,
    "duration_sec": 2.0
  }
}
```

Recommended AB entry:

```yaml
object_id: wave_greet
ab_level: 1
kind: skill
category: social_motion
aliases: [wave, greet_wave, wave_hello]
params: [style, hand, duration_sec, dry_run, result_mode]
required_params: []
implementation_status: fake
supports_failure_injection: true
robot_adapter_mapping: fake_skills.wave_greet
decomposes_to:
  - perform_motion
```

### 4.4 `wave_at`

**Registry role:** AB=2 macro candidate.

`wave_at` should be represented as a macro over AB=1 objects:

```text
look_at(target) -> wave_greet(style=friendly)
```

It should initially be **proposal-only** in the AB registry unless Codex implements a reviewed macro execution path.

Planner-visible examples:

```text
"wave at the person"
"look at them and wave"
"greet the person with a wave"
```

Canonical macro composition:

```yaml
object_id: wave_at
ab_level: 2
kind: macro_skill
category: social_hri
implementation_status: proposal
decomposes_to:
  - look_at
  - wave_greet
result_schema:
  required: [skill, status, summary_text]
```

Execution options:

1. **Base stack:** planner emits two AB=1 steps: `look_at`, `wave_greet`.
2. **Workbench mode:** Workbench may propose AB=2 `wave_at`, then expand to AB=1 before sending to `nao_orchestrator`.

Acceptance:

- `wave_at` must not bypass `nao_orchestrator`.
- If no macro executor exists yet, keep `wave_at` as a registry proposal and expand to AB=1 steps.

### 4.5 `inspect_area`

**Registry role:** AB=1 fake skill initially; AB=2/AB=3 candidate later.

Recommended as the third fake skill family because it sits naturally between perception and navigation.

Planner-visible examples:

```text
"inspect the table"
"check the area near the sofa"
"look at the desk and tell me if anything is there"
```

Simple AB=1 fake form:

```text
inspect_area(target_area)
```

AB=2 macro form:

```text
navigate_to(area) -> scan(target_kind=scene) -> report_result
```

For now, implement as a **fake AB=1 executable skill** with a decomposition hint to future AB=2 macro variants.

Result modes:

```text
clear
object_found
person_found
ambiguous
area_unknown
backend_unavailable
```

Canonical result payload:

```json
{
  "skill": "inspect_area",
  "status": "succeeded",
  "target": "table",
  "target_kind": "area",
  "target_found": true,
  "summary_text": "The table area appears clear.",
  "evidence": {
    "area": "table",
    "objects": [],
    "people": [],
    "simulated": true
  },
  "metadata": {
    "ab_object_id": "inspect_area",
    "fake": true
  }
}
```

Why `inspect_area`:

- It gives the planner a richer fake perception task.
- It can later become an AB=2 or AB=3 strategy over `navigate_to`, `scan`, `find_object`, and `report_result`.
- It supports validation scenarios beyond simple object lookup.

## 5. Generic fake skill result schema

All fake skills must return the same outer payload shape:

```json
{
  "skill": "string",
  "status": "succeeded | failed | partial | unavailable",
  "target": "string",
  "target_kind": "string",
  "target_found": true,
  "summary_text": "string",
  "evidence": {},
  "failure": {
    "code": "string",
    "message": "string",
    "recoverable": true,
    "suggested_recovery": "string"
  },
  "metadata": {
    "ab_object_id": "string",
    "fake": true,
    "result_mode": "string",
    "duration_sec": 0.0,
    "scenario_id": "string"
  }
}
```

The current `SkillResultPayload` contract should be reused. Do not invent a second payload schema.

## 6. Scenario configuration

Create:

```text
src/fake_skills/config/fake_skill_scenarios.yaml
```

Example:

```yaml
default:
  navigate_to:
    result_mode: success
    delay_sec: 1.0
  find_object:
    result_mode: found
    target_found: true
  wave_greet:
    result_mode: success
    dry_run: true
  inspect_area:
    result_mode: clear

scenarios:
  path_blocked:
    navigate_to:
      result_mode: path_blocked
      recoverable: true
  ambiguous_cup:
    find_object:
      result_mode: ambiguous
      candidates:
        - {id: fake_cup_1, label: cup, confidence: 0.72}
        - {id: fake_cup_2, label: cup, confidence: 0.69}
  social_wave_unavailable:
    wave_greet:
      result_mode: motion_unavailable
  area_person_found:
    inspect_area:
      result_mode: person_found
```

The engine should allow scenario override per request:

```json
{
  "skill": "navigate_to",
  "args": {"target": "kitchen"},
  "scenario": {"result_mode": "path_blocked"}
}
```

## 7. ROS interfaces

Codex should inspect the current action/message seams before finalizing exact types. If no suitable custom action exists, use JSON payloads over a simple action or service wrapper consistent with existing repo patterns.

Preferred conceptual interface:

```text
Action: /skill/fake/execute

Goal:
  skill_name: string
  args_json: string
  scenario_json: string

Feedback:
  progress: float
  phase: string
  message: string

Result:
  ok: bool
  status: string
  result_payload_json: string
```

If action definition changes are too heavy, implement an internal Python dispatcher first and expose integration through `nao_orchestrator` using the existing Intent/JSON pathway.

## 8. Integration with `nao_orchestrator`

The orchestrator should dispatch fake skills using the canonical AB registry executor mapping:

```text
fake_skills.navigate_to
fake_skills.find_object
fake_skills.wave_greet
fake_skills.inspect_area
```

Rules:

- Orchestrator validates plan step.
- Orchestrator dispatches fake skill.
- Fake skill returns `SkillResultPayload`.
- Orchestrator wraps it in `ExecutionFeedback`.
- Planner receives `/planner/execution_feedback`.
- `stack_observer` displays active skill + payload.

Do not allow fake skills to publish final speech directly. Speech remains through planner dialogue act and `dialogue_manager`.

## 9. Integration with Neural Workbench

The fake skills are required for Workbench validation.

Workbench use cases:

```text
candidate program: navigate_to(kitchen)
fake result: path_blocked
expected workbench/planner response: ask user or recover

candidate program: find_object(cup)
fake result: ambiguous
expected response: ask clarification

candidate program: wave_at(person)
macro expansion: look_at(person) -> wave_greet
expected response: grounded completion
```

Fake skills should provide enough metadata for trace memory:

```json
{
  "ab_object_id": "navigate_to",
  "result_mode": "path_blocked",
  "recoverable": true,
  "delta_entropy_proxy": 0.21,
  "energy_observation": {
    "risk_observed": 0.15,
    "cost_observed": 0.30
  }
}
```

Entropy fields can be optional for now.

## 10. AB registry updates

Add or update entries for:

```text
wave_greet
inspect_area
wave_at
navigate_with_recovery
```

Do not remove existing entries:

```text
navigate_to
find_object
walk_to
scan
perform_motion
look_at
report_result
```

Example macro entry:

```yaml
object_id: wave_at
ab_level: 2
kind: macro_skill
category: social_hri
implementation_status: proposal
decomposes_to:
  - look_at
  - wave_greet
expected_effects:
  - robot attends to target
  - robot performs greeting wave
observable_success:
  - look_at.status
  - wave_greet.status
safety_flags:
  - attention
  - motion
```

## 11. Tests

Add unit tests:

```text
src/fake_skills/test/test_result_builders.py
src/fake_skills/test/test_scenario_store.py
src/fake_skills/test/test_engine.py
src/fake_skills/test/test_action_server_smoke.py
```

Add integration tests if current test harness allows:

```text
src/nao_orchestrator/test/test_fake_skill_dispatch.py
src/neural_workbench/test/test_fake_skill_candidate_feedback.py
src/stack_observer/test/test_fake_skill_event_render.py
```

Test cases:

| Test | Expected |
|---|---|
| navigate_to success | `status=succeeded`, no failure |
| navigate_to path_blocked | recoverable failure payload |
| find_object found | evidence contains object |
| find_object ambiguous | multiple candidates + clarification recovery |
| wave_greet success | dry-run gesture payload |
| wave_at expansion | expands to `look_at`, `wave_greet` if no macro executor |
| inspect_area person_found | evidence.people populated |
| unexpected_payload | verifier rejects or marks failed |
| fail_once | first call fails, second call succeeds |
| scenario override | per-request scenario wins over default |

## 12. Launch

Create:

```text
src/fake_skills/launch/fake_skills.launch.py
```

Parameters:

```yaml
fake_skills_enabled: true
fake_skill_scenario_file: config/fake_skill_scenarios.yaml
fake_skill_default_delay_sec: 0.75
fake_skill_publish_events: true
fake_skill_event_topic: /fake_skills/events
fake_skill_deterministic_seed: 42
```

Include this launch in the demo stack only when fake validation is needed.

## 13. Dashboard/TUI requirements

`stack_observer` should display:

```text
active fake skill
scenario/result_mode
result_payload.status
result_payload.failure.code
recoverable flag
suggested recovery
trace_id if available
```

The fake skill package should optionally publish:

```text
/fake_skills/events
```

Event payload:

```json
{
  "event_type": "fake_skill_started | fake_skill_feedback | fake_skill_completed",
  "skill": "navigate_to",
  "scenario_id": "path_blocked",
  "result_mode": "path_blocked",
  "goal_id": "goal_123",
  "payload": {}
}
```

## 14. Acceptance checklist

- `fake_skills` package builds.
- All fake skills return `SkillResultPayload`.
- `navigate_to`, `find_object`, `wave_greet`, and `inspect_area` are callable.
- `wave_at` is represented as AB=2 proposal or macro expansion.
- Failure modes are deterministic and configurable.
- Orchestrator can dispatch fake skills through registry mapping.
- ExecutionFeedback includes fake result payload.
- Planner receives feedback and can ask/replan/fail.
- Stack observer displays fake skill lifecycle.
- Neural Workbench can use fake skills for scenario validation.

## 15. First implementation order

1. Create `fake_skills` package skeleton.
2. Implement result payload builders.
3. Implement scenario store.
4. Implement fake engine with synchronous Python call surface.
5. Add ROS action/server wrapper.
6. Wire `nao_orchestrator` dispatch for `navigate_to` and `find_object`.
7. Add `wave_greet`.
8. Add `inspect_area`.
9. Add AB registry entries for `wave_greet`, `inspect_area`, and `wave_at`.
10. Add dashboard event emission.
11. Add integration tests.

## 16. Safety and scope

Do not:

- bypass `nao_orchestrator`;
- let fake skills speak directly;
- confuse fake success with real robot capability;
- enable real walking under fake `walk_to`;
- auto-register AB=2 macros as live executable skills;
- use prompt-only simulation for validation when a fake skill server can return typed payloads.

The fake skills are a validation substrate. They should be deterministic, inspectable, and boringly reliable.
