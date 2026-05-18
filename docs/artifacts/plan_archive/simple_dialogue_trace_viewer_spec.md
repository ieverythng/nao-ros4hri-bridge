# NAO ROS4HRI Observability — Simple Dialogue Trace Viewer Spec

**Target branch:** `feat(R)/Neural-Workbench` or `feat/observability-dashboard`  
**Backport relevance:** high for `feat/TFM-LLM_planner`  
**Priority:** P0  
**Purpose:** provide the simple, supervisor-friendly view of the real runtime flow.

## 1. Goal

Build a lightweight trace viewer that shows the full flow:

```text
User -> dialogue_manager -> chatbot_llm -> planner_llm -> nao_orchestrator -> skill/action server -> execution feedback -> planner dialogue act -> robot speech
```

It should be simple enough to run during demos and debugging. It must display the actual payloads at every step, but in a compact readable way.

## 2. Recommended package

```text
src/interaction_trace_viewer/
  package.xml
  setup.py
  interaction_trace_viewer/
    trace_node.py
    trace_model.py
    payload_normalizer.py
    render_tui.py
    render_html.py
  launch/
    interaction_trace_viewer.launch.py
  test/
    test_trace_model.py
    test_payload_normalizer.py
```

A single script is acceptable for the first pass:

```text
scripts/interaction_trace_viewer.py
scripts/render_interaction_trace_html.py
```

but the event model should be clean enough to migrate into `stack_observer`.

## 3. Real ROS artifacts to observe

Subscribe directly to actual ROS artifacts wherever possible:

```text
/humans/voices/*/speech
/planner/request
/intents
/planner/execution_feedback
/planner/dialogue_act
/scene/summary
/rosout
```

Optional / future:

```text
/fake_skills/events
/neural_workbench/candidates
/neural_workbench/selection
/neural_workbench/trace
/stack_observer/events
```

If a topic is missing, show `source unavailable` rather than crashing.

## 4. Normalized event model

```python
@dataclass
class InteractionEvent:
    timestamp: float
    trace_id: str | None
    source_node: str
    channel: str
    event_type: str
    ab_object_id: str | None
    ab_level: int | None
    summary: str
    payload: dict
    raw: str | None = None
```

Event types:

```text
user_utterance
chatbot_response
planner_request
planner_output
orchestrator_dispatch
skill_started
skill_feedback
skill_result
execution_feedback
planner_dialogue_act
robot_speech
scene_update
error
warning
```

## 5. Trace grouping

Preferred:

```text
new user utterance starts a trace
final /planner/dialogue_act closes the trace
```

Fallback:

```text
/planner/request starts a trace when user speech is not available
```

If planner/common already carries `trace_id`, `turn_id`, or `correlation_id`, reuse it.

## 6. TUI output

Example compact output:

```text
TRACE trace_024 | user: "find the cup and tell me where it is"

[00.000] USER
  find the cup and tell me where it is

[00.142] CHATBOT -> PLANNER REQUEST
  goal: find object and report result
  target: cup

[00.412] PLANNER
  steps:
    1. find_object(target="cup")
    2. report_result(source="step_1")

[00.620] ORCHESTRATOR
  dispatch: find_object {"target": "cup"}

[01.220] SKILL RESULT
  status: succeeded
  target_found: true
  evidence: fake_cup_1 confidence=0.91

[01.360] DIALOGUE ACT
  say: "I found one cup."
```

CLI:

```bash
ros2 run interaction_trace_viewer trace_node --compact
ros2 run interaction_trace_viewer trace_node --verbose
```

## 7. Static HTML report

Support:

```bash
ros2 run interaction_trace_viewer render_html \
  --input traces/latest.jsonl \
  --output traces/latest.html
```

HTML sections:

```text
trace summary
timeline
planner request
planner output
skill dispatches
skill result payloads
final dialogue act
errors/warnings
raw JSON appendix
```

## 8. Payload display rules

Compact mode should show:

- user text;
- chatbot intent/routing summary;
- planner goal and plan steps;
- orchestrator dispatched skill;
- skill result status;
- `target_found`;
- `failure.code`;
- `failure.recoverable`;
- `suggested_recovery`;
- final dialogue act.

Verbose mode shows full JSON.

## 9. Launch parameters

```yaml
trace_viewer_enabled: true
compact_mode: true
write_jsonl: true
jsonl_output_dir: ~/.ros/nao_ros4hri_traces
write_html_on_shutdown: true
html_output_dir: ~/.ros/nao_ros4hri_trace_reports
include_raw_payloads: true
max_payload_chars: 4000
```

## 10. Acceptance checklist

- Launches with the current stack.
- Does not interfere with execution.
- Subscribes to real ROS topics.
- Shows user → chatbot → planner → orchestrator → skill → feedback → dialogue act.
- Shows actual payloads.
- Writes JSONL traces.
- Renders simple HTML reports.
- Handles missing optional topics gracefully.
- Works with fake skill result payloads.
- Works without Neural Workbench enabled.

## 11. Non-goals

This simple viewer should not manage node restarts, launch profiles, AB-space visualization, registry editing, or Workbench macro crystallization. It is a clear trace viewer.
