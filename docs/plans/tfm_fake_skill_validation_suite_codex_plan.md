# Codex Handoff — Reproducible Fake-Skill Validation Suite for TFM Evidence

**Target branch:** `refactor/deslop_repo`  
**Primary repo:** `ieverythng/nao-ros4hri-bridge`  
**Purpose:** build a reproducible validation harness that runs fake-skill scenarios through the live planner stack, captures traces, and produces metrics/datasets for the TFM.  
**Priority:** P0/P1. Start tomorrow; expand over the next 11 days.

---

## 1. Goal

Create a validation suite that can run controlled HRI/planner scenarios repeatedly and save evidence.

The suite should answer:

```text
Given a user utterance and a fake-skill outcome policy, does the stack:
1. route correctly?
2. plan the right skills?
3. dispatch through nao_orchestrator?
4. handle success/failure/ambiguity safely?
5. replan or communicate when appropriate?
6. produce a readable final response?
7. leave a trace that can be analyzed for the TFM?
```

This should use existing fake-skill seams rather than building a parallel simulator.

---

## 2. Current available seams

The repo already has the key components needed:

```text
fake_skills package
fake_skill_server
ScenarioStore
FakeSkillEngine
global modes: scenario, always_success, always_fail, every_other, random_seeded
active_scenario_id parameter
mode_overrides_json parameter
/fake_skills/events
interaction_trace_viewer
JSONL trace writer
HTML trace report support
run_live_fake_skill_scenario_probe.py
```

Therefore, the work should be incremental, not a rewrite.

---

## 3. Validation suite architecture

Add a higher-level runner:

```text
scripts/run_tfm_validation_suite.py
```

or package form:

```text
src/tfm_validation/
  tfm_validation/
    suite_runner.py
    case_loader.py
    metrics.py
    report_writer.py
    fixture_builder.py
    baseline.py
  config/
    tfm_validation_cases.yaml
```

Initial recommendation: script first, package later.

The runner should:

```text
1. set fake_skill_server policy;
2. publish a controlled planner request or user utterance;
3. record interaction_trace_viewer JSONL;
4. wait for completion/failure/timeout;
5. collect events;
6. compute metrics;
7. write run bundle.
```

---

## 4. Case definition format

Create:

```text
docs/evaluation/tfm_validation_cases.yaml
```

Example:

```yaml
suite:
  name: planner_fake_skill_validation_v1
  default_wait_sec: 12.0
  repetitions: 1
  seeds: [42]

cases:
  - id: find_cup_success
    utterance: "find the cup"
    route_mode: planner_request
    goal_text: "find the cup"
    normalized_intents: ["find_object"]
    scene_targets: ["cup"]
    fake_policy:
      global_mode: scenario
      active_scenario_id: ""
      mode_overrides_json: "{}"
    expected:
      plan_contains: ["find_object", "report_result"]
      final_status_any: ["succeeded", "completed"]
      should_replan: false

  - id: find_cup_ambiguous
    utterance: "find the cup"
    route_mode: planner_request
    goal_text: "find the cup"
    normalized_intents: ["find_object"]
    scene_targets: ["cup"]
    fake_policy:
      global_mode: scenario
      active_scenario_id: "ambiguous_cup"
    expected:
      plan_contains: ["find_object"]
      should_ask_user: true
      acceptable_outcomes: ["clarification", "blocked"]

  - id: navigate_path_blocked
    utterance: "go to the cup"
    route_mode: planner_request
    goal_text: "navigate to the cup"
    normalized_intents: ["navigate_to"]
    scene_targets: ["cup"]
    fake_policy:
      global_mode: scenario
      active_scenario_id: "path_blocked"
    expected:
      plan_contains: ["navigate_to"]
      should_replan_or_fail_safely: true

  - id: scan_report_success
    utterance: "look around and tell me what you see"
    route_mode: planner_request
    goal_text: "look around and tell me what you see"
    normalized_intents: ["scan", "report_result"]
    scene_targets: []
    fake_policy:
      global_mode: always_success
    expected:
      plan_contains: ["scan", "report_result"]
      report_result_uses_live_payload: true

  - id: navigate_all_objects_report_each
    utterance: "Now walk to every object, let me know when you are there and then walk to the next!"
    route_mode: user_turn
    goal_text: "walk to every grounded object, report each arrival, and then continue to the next object"
    normalized_intents: ["navigate_to", "report_result"]
    scene_targets: ["apple", "book", "phone"]
    fake_policy:
      global_mode: always_success
    expected:
      plan_contains: ["navigate_to", "report_result"]
      report_after_each_navigation: true
      final_report_mentions_all_targets: true
```

Keep `route_mode=planner_request` for fast stability. Add true user-speech injection later.

---

## 5. Two execution modes

### Mode A — Planner-request injection

Publish directly to `/planner/request` or the orchestrator planner-gate topic.

Pros:

```text
fast
controlled
isolates planner/orchestrator/fake skill behavior
less ASR/dialogue noise
```

Cons:

```text
does not test full chatbot routing
```

Use this for primary TFM planner validation.

### Mode B — Full user-turn injection

Inject utterance through the dialogue/chatbot path.

Pros:

```text
tests HRI routing and chatbot prompt behavior
```

Cons:

```text
slower, more flaky, harder to isolate
```

Use this for a smaller secondary validation set.

Recommended for 11-day timeline:

```text
Primary dataset: Mode A
Supplementary dataset: Mode B for 8-12 cases
```

---

## 6. Scenario matrix

Use fake-skill global modes already available:

```text
scenario
always_success
always_fail
every_other
random_seeded
```

Use named scenarios already available:

```text
ambiguous_cup
area_person_found
head_motion_strict
path_blocked
social_wave_unavailable
```

Minimum tomorrow matrix:

| Case family | Modes |
|---|---|
| find_object | success, ambiguous, always_fail |
| navigate_to | success, path_blocked |
| navigate_to plus report_result | all objects success, one target missing, path_blocked |
| scan/report_result | always_success, always_fail |
| wave_greet | success, social_wave_unavailable |
| inspect_area | clear, area_person_found |

Recommended P0 run count:

```text
10-15 cases x 1 repetition = fast smoke dataset
```

Recommended TFM dataset:

```text
8 utterance families x 5 outcome policies x 3 repetitions = 120 runs
```

If runtime is too long:

```text
8 x 5 x 2 = 80 runs
```

This is enough for descriptive metrics without overclaiming statistical significance.

---

## 7. Fixtures: objects and scene context

The current probe already publishes a controlled planner request and can include `grounded_context`.

Add a fixture builder:

```python
def build_grounded_context(profile: str) -> dict:
    ...
```

Profiles:

```text
empty_scene
cup_visible
two_cups_ambiguous
person_visible
person_and_cup_visible
area_table_clear
area_table_person
```

Example:

```json
{
  "entities": [
    {
      "id": "cup_1",
      "label": "cup",
      "kind": "object",
      "class": "Cup",
      "visible": true,
      "relations": [
        {"predicate": "oro:isOn", "object": "table_1"}
      ]
    }
  ]
}
```

This does not need to fully simulate the scene graph initially. It only needs to provide deterministic planner context.

Later, optionally publish simulated `/scene/summary` or KB snapshots to align the whole stack.

---

## 8. Metrics to compute

Per run:

```text
case_id
utterance
route_mode
global_mode
scenario_id
seed
goal_id
event_count
latency_total_sec
planner_called
plan_steps
skills_dispatched
skill_result_statuses
replan_requested
clarification_requested
dialogue_act_type
final_status
final_response_text
timeout
errors_count
warnings_count
```

Planner metrics:

```text
plan_valid_json
plan_contains_expected_skills
forbidden_say_skill_present
report_result_terminal
report_result_summary_text_prefilled_after_scan
on_failure_values_valid
```

Execution metrics:

```text
all_steps_attempted
first_failure_step
safe_stop_after_failure
feedback_published
fake_skill_event_seen
```

HRI metrics:

```text
final_response_present
response_not_duplicate
clarification_when_ambiguous
failure_explained_when_blocked
```

Aggregate metrics:

```text
success_rate
route_accuracy
plan_correctness_rate
safe_failure_rate
clarification_rate_when_expected
mean_latency
timeout_rate
forbidden_plan_rate
```

---

## 9. Baselines

A perfect historical baseline may be hard to reconstruct. Do not lose time chasing it unless a stable commit is obvious.

Use tiered baselines:

### Baseline 1 — Static expected-plan baseline

For each case, define the expected skill sequence manually.

Measure:

```text
planner output vs expected sequence
```

This is not a runtime robot baseline, but it is a valid planning-correctness reference.

### Baseline 2 — No-replanning baseline

Run same cases with recovery disabled or with `on_failure=fail` normalization.

Measure:

```text
Does planner supervision improve safe recovery / clarification compared with immediate failure?
```

This is useful and feasible.

### Baseline 3 — Earlier non-planner stack

Only use if a known branch/commit can be run quickly.

Measure:

```text
multi-intent handling
failure handling
dialogue quality
```

Do not make the TFM depend on this baseline unless it is stable within one day.

Recommended TFM framing:

```text
Primary evaluation is scenario-based validation of the planner architecture under controlled outcomes.
Baseline comparisons are used where reproducible, but the central evidence is correct behavior across success, ambiguity, failure, and recovery scenarios.
```

---

## 10. Dataset size and timeline

### Tomorrow / demo smoke

```text
12 runs
```

Suggested:

```text
find_cup_success
find_cup_ambiguous
find_cup_always_fail
navigate_success
navigate_path_blocked
scan_success
scan_fail
wave_success
wave_unavailable
inspect_clear
inspect_person_found
random_seeded_find
```

### TFM minimum

```text
40-60 runs
```

Enough for tables, qualitative trace examples, and basic rates.

### TFM strong version

```text
80-120 runs
```

Allows scenario matrix plots and more confidence.

---

## 11. Report outputs

Each suite run should write:

```text
docs/evaluation/runs/<timestamp>/
  suite_config.yaml
  trace.jsonl
  per_case_metrics.csv
  aggregate_metrics.json
  report.md
  report.html
  selected_traces/
    case_id_trace.json
```

Optional:

```text
plots/
  success_rate_by_mode.png
  plan_correctness_by_case.png
  latency_by_case.png
  outcome_confusion_table.csv
```

---

## 12. Implementation steps

## Phase 1 — Extend existing probe, do not rewrite

Copy/refactor from:

```text
scripts/run_live_fake_skill_scenario_probe.py
```

Add:

```text
--cases-file docs/evaluation/tfm_validation_cases.yaml
--output-dir docs/evaluation/runs
--mode planner_request|user_turn
--repetitions N
--seeds 42,43,44
--skip-relaunch
--skip-sync-rebuild
```

Keep the existing Docker/container pattern for now.

## Phase 2 — Add metrics extraction

Implement:

```python
extract_plan_steps(events)
extract_skill_results(events)
extract_dialogue_act(events)
extract_errors(events)
score_case(case, events)
```

## Phase 3 — Add fixture builder

Implement scene profiles in the planner request payload.

## Phase 4 — Add report writer

Write:

```text
report.md
report.html
per_case_metrics.csv
aggregate_metrics.json
```

## Phase 5 — Add full user-turn mode

Only after planner-request mode is stable.

## Phase 6 — Add plots

Use CSV output. Do not block validation on plots.

---

## 13. Acceptance checklist

- Can run at least 10 cases end-to-end.
- Can switch fake-skill global modes without relaunch.
- Can use named scenarios.
- Captures JSONL trace for every case.
- Produces per-case metrics CSV.
- Produces aggregate summary.
- Detects expected plan steps.
- Detects forbidden planner outputs like executable `say`.
- Detects whether `report_result` follows scan/find when expected.
- Detects failure/clarification/replan behavior.
- Produces at least one HTML report for TFM evidence.
- Does not require the physical robot for fake-skill validation.

---

## 14. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Full user-turn mode is flaky | Use planner-request mode as primary dataset |
| Too many variables | Fix seeds, scenario modes, fixture profiles |
| Dataset takes too long | Start with 12-run smoke, then expand overnight |
| Baseline branch is unstable | Use static expected-plan and no-replanning baselines |
| Fake skills over-simplify reality | Frame as controlled scenario validation, not full real-world performance |
| Trace matching misses events | Use `goal_id`, `turn_id`, and `trace_id` consistently |

---

## 15. Suggested first commit names

```text
docs: add TFM validation suite plan
feat(validation): add case-driven fake skill suite runner
feat(validation): add metrics extraction and HTML reports
test(validation): add smoke scenario config
```
