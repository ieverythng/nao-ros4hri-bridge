# Codex Handoff — NAO Observability Dashboard Implementation Roadmap

## 1. Strategy

Implement two versions:

| Version | Purpose | Lead time |
|---|---|---|
| Simple trace viewer | supervisor-readable payload flow | short |
| Full NAO dashboard | ROS graph + AB + Workbench visualization | long |

Do not block the simple viewer on the full dashboard.

## 2. PR sequence

```text
PR 1: interaction_trace_viewer event model + terminal timeline
PR 2: JSONL trace writer + simple HTML report
PR 3: ROS graph snapshot utility
PR 4: web dashboard skeleton with live timeline
PR 5: AB registry graph panel
PR 6: skill/action server panel + fake skill events
PR 7: Neural Workbench candidate/energy/entropy panel
PR 8: report/replay/export bundle
PR 9: launch profile health view
PR 10: guarded restart/scenario controls
```

## 3. Shared event compatibility

Both versions should share the same event format:

```json
{
  "timestamp": 0.0,
  "run_id": "run_001",
  "trace_id": "trace_001",
  "source": "planner_llm",
  "event_type": "planner_output",
  "channel": "/intents",
  "ab_object_id": "find_object",
  "ab_level": 1,
  "payload_summary": "find_object(target=cup)",
  "payload": {}
}
```

## 4. Expected repo locations

```text
src/interaction_trace_viewer/
src/nao_dashboard/
```

Shared models can live in `stack_observer` or a small `observability_common` package if necessary.

## 5. Final acceptance

The supervisor should be able to ask:

```text
What happened when I gave this command?
```

and the system should show:

```text
exact user input
chatbot interpretation
planner request
plan
orchestrator dispatch
skill/action result
final robot speech
errors/warnings
```

The future dashboard should additionally show:

```text
which AB object was used
what it decomposed into
whether it was fake or real
candidate programs rejected by Workbench
selected candidate and score
trace stored
nodes/actions/services to debug or restart
```
