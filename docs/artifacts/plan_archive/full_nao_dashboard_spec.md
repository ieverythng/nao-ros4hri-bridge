# NAO ROS4HRI Observability — Full Integrated Dashboard Specification

**Target branch:** `feat(R)/Neural-Workbench` or `feat/nao-observability-dashboard`  
**Priority:** P1/P2, long-lead implementation  
**Purpose:** the full “wow version” for demos, debugging, Neural Workbench visualization, and future PhD-grade research tooling.

## 1. Vision

Build a browser-based dashboard that externalizes the whole NAO cognitive/execution stack:

```text
user input
dialogue_manager
chatbot_llm
planner_llm
Neural Workbench
nao_orchestrator
skill/action servers
scene grounding
KB / WME-like state
AB registry
trace memory
ROS graph health
```

The dashboard should answer:

```text
What did the user say?
What did chatbot_llm infer?
What did planner_llm plan?
What candidates did the Workbench consider?
What did the verifier reject?
What did the orchestrator dispatch?
What skill/action ran?
What result payload came back?
Why did the robot say what it said?
What node/topic/service/action is failing?
```

## 2. Two modes

### Base stack mode

For `feat/TFM-LLM_planner` or Workbench disabled:

```text
dialogue_manager -> chatbot_llm -> planner_llm -> nao_orchestrator -> skills -> feedback -> dialogue_act
```

### Neural Workbench mode

Adds:

```text
candidate pulse programs
AB registry verification
energy/entropy scores
selected program
trace retrieval
macro candidates
AB-space visualizations
```

Base mode must work first.

## 3. Direct ROS artifact rule

The dashboard should use `skill_common`, AB registry, and `stack_observer`, but it should also wire directly to real ROS artifacts:

```text
nodes
topics
services
actions
message types
publisher/subscriber counts
action server availability
/rosout
```

This is central because the supervisor has repeatedly requested visibility into the actual ROS interactions.

## 4. Proposed architecture

```text
dashboard_web_ui
  <-> websocket/http
dashboard_backend_node
  -> ROS graph introspection
  -> topic subscriptions
  -> service/action introspection
  -> skill_common / AB registry loader
  -> stack_observer event stream
  -> trace memory reader
  -> launch profile manager
```

Suggested package:

```text
src/nao_dashboard/
  package.xml
  setup.py
  nao_dashboard/
    backend_node.py
    ros_graph_introspector.py
    topic_tap.py
    action_introspector.py
    service_introspector.py
    registry_adapter.py
    ab_graph_adapter.py
    trace_store.py
    launch_profiles.py
    node_health.py
    websocket_server.py
    models.py
  web/
    index.html
    app.js
    styles.css
  launch/
    nao_dashboard.launch.py
  test/
```

Simpler alternative: place early version inside `stack_observer`.

## 5. Main screens

### 5.1 Live flow view

Visual graph/timeline:

```text
User -> DialogueManager -> ChatbotLLM -> PlannerLLM -> Workbench? -> Orchestrator -> Skill -> Feedback -> DialogueAct -> Speech
```

Each edge shows:

```text
topic/service/action name
last timestamp
message count
latency
payload summary
```

Clicking an edge opens the latest payload.

### 5.2 ROS graph view

Show actual ROS graph:

```text
nodes as boxes
topics as blue edges
services as purple edges
actions as orange edges
parameters as metadata
```

Highlight:

```text
missing expected nodes
inactive action servers
orphan topics
high-error nodes
stale publishers
```

### 5.3 Plan execution view

Show current plan:

```text
Plan ID: plan_023
Goal: "find the cup and tell me where it is"

1. find_object(target=cup)       running/succeeded/failed
2. report_result(source=step_1)  pending/succeeded
```

For each step show:

```text
AB object ID
skill/action endpoint
args
expected effects
result payload
failure code
recovery suggestion
```

### 5.4 NAO skill/action view

For each skill:

```text
name
AB level
real/fake
owner package
action server status
last call
success count
failure count
latency
result schema
```

NAO-specific panels:

```text
head/gaze status
speech status
posture/motion status
camera/perception status
NAOqi connection status
fake skill scenario mode
```

### 5.5 Scene / KB / WME-like state

Show:

```text
/scene/summary
visible people
visible objects
person IDs
object labels/confidence
knowledge snapshot
KB query/revise activity
latest skill payloads
active uncertainties
```

### 5.6 AB registry graph

Visualize:

```text
AB=0 primitives
AB=1 executable skills
AB=2 macro proposals
AB=3 strategy proposals
```

Color by:

```text
implemented
fake
proposal
deprecated
failed validation
```

Example decomposition:

```text
perform_motion -> wave_greet
look_at + wave_greet -> wave_at
find_object + report_result -> find_object_and_report
navigate_to + recovery branch -> navigate_with_recovery
```

### 5.7 Neural Workbench search view

When enabled, show:

```text
Goal: "wave at the person"

Candidate A:
  look_at -> wave_greet
  valid: true
  energy: 0.21
  entropy_after: 0.18

Candidate B:
  report_result only
  valid: false
  reason: missing social gesture

Selected:
  wave_at expanded to look_at + wave_greet
```

Even if selection is fast, capture events and allow replay.

### 5.8 Energy / entropy / AB maturity view

Future research visualizations:

```text
selected program energy over traces
entropy_before / entropy_after
delta_entropy
AB maturity A1/A2/A3
success/failure by AB object
probability shift after traces
```

### 5.9 Launch profile and health view

Profiles:

```text
base_demo
fake_skill_validation
robot_live
neural_workbench
dashboard_only
```

For each profile show:

```text
required nodes
optional nodes
expected topics
expected actions
expected services
status
start/restart command
```

Start read-only. Add guarded restart controls later.

## 6. Backend data model

```python
@dataclass
class DashboardEvent:
    timestamp: float
    event_id: str
    run_id: str
    trace_id: str | None
    source: str
    event_type: str
    channel: str
    ab_object_id: str | None
    ab_level: int | None
    payload_summary: str
    payload: dict
```

```python
@dataclass
class RosGraphSnapshot:
    timestamp: float
    nodes: list
    topics: list
    services: list
    actions: list
    edges: list
```

```python
@dataclass
class ABRegistrySnapshot:
    timestamp: float
    objects: list
    edges: list
    validation_errors: list[str]
```

## 7. ROS discovery

Use `rclpy` introspection where possible:

```python
get_node_names()
get_topic_names_and_types()
get_service_names_and_types()
get_publishers_info_by_topic()
get_subscriptions_info_by_topic()
```

For actions, use ROS2 APIs if available; otherwise shell fallback with timeout protection:

```text
ros2 action list
ros2 action info /action_name
```

## 8. Event sources

Minimum:

```text
/planner/request
/intents
/planner/execution_feedback
/planner/dialogue_act
/scene/summary
/rosout
```

Recommended:

```text
/fake_skills/events
/stack_observer/events
/neural_workbench/candidates
/neural_workbench/selection
/neural_workbench/trace
/skill_common/registry_snapshot
/ab_registry/snapshot
```

## 9. Reports and exports

Export:

```text
current trace JSON
trace HTML report
ROS graph snapshot JSON
AB registry snapshot JSON
dashboard snapshot HTML
experiment run bundle
```

Experiment bundle:

```text
run_id/
  trace.jsonl
  ros_graph_snapshot.json
  ab_registry_snapshot.json
  errors.jsonl
  report.html
```

## 10. Implementation phases

### Phase 0 — Spec and mock

Add this spec and optionally a static mock.

### Phase 1 — Web live timeline

Reuse simple trace viewer event model. Show live timeline and payload inspector.

### Phase 2 — ROS graph introspection

Add nodes/topics/services/actions snapshot and expected-vs-missing artifacts.

### Phase 3 — AB registry visualization

Load `skill_common`/AB registry and render decomposition graph.

### Phase 4 — Skill/action server panel

Show skill endpoints, fake scenarios, payloads, counters.

### Phase 5 — Neural Workbench panel

Show candidates, verification, energy/entropy, selected program.

### Phase 6 — Reports and replay

Save and replay trace bundles.

### Phase 7 — Launch profiles and safe controls

Read launch profiles, show health, then add guarded restart/scenario controls.

## 11. Safety rules

Do not allow early dashboard controls to:

```text
bypass nao_orchestrator
publish arbitrary robot speech
call robot actions directly
enable movement without explicit operator guard
restart nodes without confirmation
mutate AB registry live without review
```

First dashboard should be read-only plus export.

## 12. Why this matters

This is not only a developer tool. For Neural Workbench it becomes a visual microscope for:

```text
AB object usage
candidate pulse search
symbolic verification
energy/entropy scoring
trace memory
macro crystallization
probability landscape shifts
```

The future dashboard can show the agent searching capability space in replay, even when runtime planning is fast.
