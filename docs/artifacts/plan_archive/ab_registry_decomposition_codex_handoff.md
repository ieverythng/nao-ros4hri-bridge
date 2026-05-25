# Codex Handoff — Canonical AB Registry and Decomposition Layer

**Target branch:** `feat(R)/Neural-Workbench`  
**Backport relevance:** selected registry-view utilities may later be backported to `feat/TFM-LLM_planner`  
**Primary repo:** `ieverythng/nao-ros4hri-bridge`  
**Purpose:** turn the current generated AB registry into the canonical symbolic decomposition graph for skills, fake skills, macro-skills, strategies, and future Neural Workbench validation.

---

## 1. Executive intent

This handoff documents the design pass where we made the AB registry explicit.

The final architecture should have:

```text
skill_common      = what can be called/executed
ab_registry       = what each abstraction is made of
trace_memory      = what happened when it was used
embedding_index   = what it is similar to
verifier          = whether it is valid/safe/grounded
crystallizer      = when repeated traces become new AB objects
```

The important addition is the second line: `ab_registry`.

The registry should not be only a list of available skills. It should become a symbolic abstraction graph where every AB object has a compositional lineage.

Core rule:

```text
AB_k_object = composition(AB_{k-1,1}, AB_{k-1,2}, ..., AB_{k-1,n})
```

However, this should be a structured composition, not a flat list. It must include order, control policy, preconditions, expected effects, result schemas, failure transitions, safety flags, trace support, entropy profile, and implementation status.

---

## 2. Why this matters now

The fake skills package makes this urgent.

We are adding executable/fake AB=1 skills such as:

```text
navigate_to
find_object
wave_greet
inspect_area
```

and macro candidates such as:

```text
wave_at = look_at -> wave_greet
find_object_and_report = find_object -> report_result
inspect_area_and_report = inspect_area -> report_result
navigate_with_recovery = navigate_to + recovery/clarification branch
```

If these are represented only as isolated YAML skill entries, the Workbench will not know how high-level behaviours relate to lower-level components. That weakens:

- symbolic validation;
- fake-skill scenario testing;
- Neural Workbench candidate verification;
- macro proposal and crystallization;
- entropy/energy scoring;
- trace memory interpretation;
- dashboard visualization;
- future AB=2/AB=3 strategy formation.

The AB registry should therefore become the canonical symbolic skeleton of the system.

---

## 3. Required conceptual split

Do not collapse everything into one skill list.

| Component | Role |
|---|---|
| `skill_common` | executable skill interface and compact views |
| `ab_registry` | symbolic abstraction graph and decomposition lineage |
| `trace_memory` | empirical outcomes from full interaction traces |
| `embedding_index` | geometric/semantic similarity over objects and traces |
| `verifier` | symbolic validity, safety, and grounding checks |
| `crystallizer` | proposes new AB objects from repeated successful traces |
| `stack_observer` | visualizes runtime events, AB lineage, traces, and payloads |

`skill_common` can host the first implementation for practical reasons, but architecturally the AB registry is a separate responsibility.

---

## 4. Current state assumption

Codex should inspect the current generated registry JSON/YAML before editing.

Expected current state includes AB=0 ROS/runtime affordances such as:

```text
/planner/request
/intents
/planner/execution_feedback
/planner/dialogue_act
/scene/summary
query_kb
revise_kb
verify_evidence_payload
store_trace
retrieve_memory
```

and AB=1 planner-visible skills such as:

```text
scan
find_object
navigate_to
walk_to
perform_motion
look_at
report_result
```

The target is to support the following long-term range:

```text
AB=0 primitive vectors / ROS topics / services / actions / cognitive affordances
AB=1 executable skills
AB=2 macro-skills and adaptive skill surfaces
AB=3 strategies
AB=4 orchestration policies
AB=5 policy surfaces / meta-workbench proposals
```

Only AB=0, AB=1, and a few AB=2 proposal objects need to be practical immediately.

---

## 5. Proposed package/module location

Preferred initial implementation inside `skill_common`:

```text
src/skill_common/
  skill_common/
    contracts.py
    registry.py
    loader.py
    ab_objects.py
    ab_registry.py
    ab_loader.py
    ab_expansion.py
    ab_validation.py
    defaults/
      skill_registry.yaml
      ab_registry.yaml
  test/
    test_ab_registry_loader.py
    test_ab_expansion.py
    test_ab_validation.py
    test_ab_dag_integrity.py
```

If the package becomes too large later, split into:

```text
src/ab_registry/
```

Do not split too early if it complicates dependencies.

---

## 6. `ABObjectSpec` schema

Add a general object specification.

```python
@dataclass(frozen=True)
class ABObjectSpec:
    object_id: str
    name: str
    ab_level: int
    kind: str
    category: str = ""
    implementation_status: str = "unknown"  # implemented | fake | proposal | deprecated
    owner_package: str = ""
    aliases: list[str] = field(default_factory=list)

    interface: dict[str, Any] = field(default_factory=dict)
    transport: dict[str, Any] = field(default_factory=dict)
    decomposition: dict[str, Any] = field(default_factory=dict)
    validation: dict[str, Any] = field(default_factory=dict)
    safety_flags: list[str] = field(default_factory=list)

    expected_effects: list[str] = field(default_factory=list)
    observable_success: list[str] = field(default_factory=list)
    failure_modes: list[str] = field(default_factory=list)

    trace_support: dict[str, Any] = field(default_factory=dict)
    entropy_profile: dict[str, Any] = field(default_factory=dict)
    embedding: dict[str, Any] = field(default_factory=dict)
    notes: str = ""
```

Minimum YAML object shape:

```yaml
objects:
  - object_id: wave_at
    name: Wave At Target
    ab_level: 2
    kind: macro_skill
    category: social_hri
    implementation_status: proposal
    owner_package: neural_workbench
    aliases: [greet_target, wave_to_person]
    interface:
      inputs:
        target: string
      outputs:
        result_payload: SkillResultPayload
    decomposition:
      lower_level: 1
      expansion_type: sequence
      nodes:
        - {id: s1, ref: look_at}
        - {id: s2, ref: wave_greet}
      edges:
        - [s1, s2]
    validation:
      executable_if_expanded_to_level: 1
      result_schema: SkillResultPayload
    safety_flags: [attention, social_motion]
```

---

## 7. Composition graph requirements

The AB registry should behave like a typed DAG or hypergraph.

Object kinds:

```text
primitive_affordance
ros_topic
ros_service
ros_action
runtime_contract
skill
fake_skill
macro_skill
strategy
orchestration_policy
policy_surface
```

Edge/relation types:

```text
composes
requires
supports
inhibits
recovers_from
generalizes
specializes
promoted_from_trace
```

Required utilities:

```python
get_object(object_id)
children(object_id, depth=1)
parents(object_id)
expand(object_id, target_level=None, max_depth=1)
validate_dag()
validate_references()
compute_depth(object_id)
compute_compression_ratio(object_id)
find_objects_by_ab_level(k)
find_executable_expansion(object_id)
```

Important rule:

> Expansion is depth-controlled. Do not expand AB=4 or AB=3 objects all the way to AB=0 by default.

Example:

```python
expand("try_recover_clarify", max_depth=1)
# returns AB=2 components

expand("try_recover_clarify", target_level=1)
# returns AB=1 executable skills

expand("try_recover_clarify", target_level=0)
# only for deep debugging, validation, or simulation
```

---

## 8. Symbolic validation requirements

The Neural Workbench verifier should use the AB registry to check candidate programs.

For any candidate pulse program or macro object, validate:

### 8.1 Object existence

```text
Does every referenced object_id exist?
```

### 8.2 Level validity

```text
Is AB=k composed from approved lower-level objects?
Is the candidate unnecessarily decomposed?
Is the candidate too abstract for the requested task?
```

### 8.3 Interface compatibility

```text
Do outputs of one node satisfy inputs of the next?
Are required parameters provided?
Are result schemas known?
```

### 8.4 Safety

```text
Does expansion include unsafe movement?
Does walking require an explicit operator flag?
Does speech bypass dialogue_manager?
Does anything bypass nao_orchestrator?
```

### 8.5 Implementation status

```text
implemented = executable
fake = executable only in validation/simulation mode
proposal = expand first or keep non-executable
deprecated = reject
```

### 8.6 Trace support

```text
Does this macro have enough empirical support?
Is failure rate acceptable?
Is mean energy low enough?
Is entropy reduction positive?
```

---

## 9. Relationship to embeddings

The AB registry is not redundant with embeddings.

Embeddings answer:

```text
What is similar?
What worked in related contexts?
Which trace cluster resembles this task?
```

AB registry answers:

```text
What is this abstraction made of?
Can it be expanded safely?
Can its components execute?
Can it be explained?
Can it be verified?
Can it be promoted?
```

Correct architecture:

```text
AB registry     = symbolic skeleton
trace memory    = lived experience
embedding index = geometric similarity
verifier        = immune system
crystallizer    = abstraction formation mechanism
```

Use embeddings to propose or retrieve candidates. Use the AB registry to validate them.

---

## 10. Relationship to Entropy Machines

Entropy-machine metadata should attach to AB objects, but it should be optional at first.

Suggested metadata:

```yaml
entropy_profile:
  observable_variables:
    - target_found
    - ambiguity_count
    - confidence
    - remaining_uncertainty
  expected_entropy_reduction:
    empirical_mean: null
    empirical_std: null
    sample_count: 0
  entropy_machine_status: unmeasured  # unmeasured | candidate | verified | deprecated
```

After full interaction traces, update:

```text
trace -> selected AB object -> result payload -> entropy delta -> AB object profile
```

An object becomes an entropy-machine candidate when:

```text
E[ΔH(object)] > 0
```

and later verified when:

```text
sample_count >= N
mean_delta_entropy >= threshold
failure_rate <= threshold
safety_status == approved
```

Do not block the first registry implementation on entropy fields. Add the fields now so future metrics can use them.

---

## 11. Concrete objects to add or update

### 11.1 AB=0 primitives

Examples:

```yaml
- object_id: set_gaze
  ab_level: 0
  kind: primitive_affordance
  category: motor_attention
  implementation_status: implemented
  expected_effects:
    - camera/head orientation changes

- object_id: read_scene_summary
  ab_level: 0
  kind: ros_topic
  category: perception
  implementation_status: implemented
  transport:
    type: topic
    name: /scene/summary

- object_id: query_kb
  ab_level: 0
  kind: ros_service
  category: memory
  implementation_status: implemented
```

### 11.2 AB=1 fake skills

```yaml
- object_id: navigate_to
  ab_level: 1
  kind: fake_skill
  category: navigation
  implementation_status: fake
  owner_package: fake_skills
  aliases: [go_to, move_to, navigate]
  interface:
    inputs:
      target: string
      result_mode: string
    outputs:
      result_payload: SkillResultPayload
  validation:
    result_schema: SkillResultPayload
  safety_flags: [simulated_navigation]
```

```yaml
- object_id: wave_greet
  ab_level: 1
  kind: fake_skill
  category: social_motion
  implementation_status: fake
  owner_package: fake_skills
  aliases: [wave, greet_wave, wave_hello]
  decomposition:
    lower_level: 0
    expansion_type: sequence
    nodes:
      - {id: p1, ref: perform_motion, args: {motion: wave}}
```

### 11.3 AB=2 macro candidates

```yaml
- object_id: wave_at
  ab_level: 2
  kind: macro_skill
  category: social_hri
  implementation_status: proposal
  decomposition:
    lower_level: 1
    expansion_type: sequence
    nodes:
      - {id: s1, ref: look_at}
      - {id: s2, ref: wave_greet}
    edges:
      - [s1, s2]
  validation:
    executable_if_expanded_to_level: 1
  safety_flags: [attention, social_motion]
```

```yaml
- object_id: find_object_and_report
  ab_level: 2
  kind: macro_skill
  category: perception_dialogue
  implementation_status: proposal
  decomposition:
    lower_level: 1
    expansion_type: sequence
    nodes:
      - {id: s1, ref: find_object}
      - {id: s2, ref: report_result}
```

### 11.4 AB=3 strategy proposals

```yaml
- object_id: try_recover_clarify
  ab_level: 3
  kind: strategy
  category: recovery
  implementation_status: proposal
  decomposition:
    lower_level: 2
    expansion_type: conditional_graph
    nodes:
      - {id: m1, ref: navigate_with_recovery}
      - {id: m2, ref: find_object_and_report}
      - {id: m3, ref: ask_clarification}
    policy:
      - if: failure.recoverable == true
        then: choose_recovery_or_clarify
```

Keep AB=3 examples proposal-only until trace support exists.

---

## 12. Interaction with fake skills

The fake skills package should use the AB registry in two ways.

### 12.1 Executor mapping

Executable fake skills:

```text
navigate_to -> fake_skills.navigate_to
find_object -> fake_skills.find_object
wave_greet -> fake_skills.wave_greet
inspect_area -> fake_skills.inspect_area
```

### 12.2 Macro expansion

AB=2 proposals:

```text
wave_at -> look_at -> wave_greet
find_object_and_report -> find_object -> report_result
inspect_area_and_report -> inspect_area -> report_result
```

If no macro executor exists, the Workbench or planner should expand the macro to AB=1 components before sending to `nao_orchestrator`.

Fake skills should return metadata:

```json
{
  "ab_object_id": "navigate_to",
  "fake": true,
  "result_mode": "path_blocked",
  "trace_support": {
    "scenario_id": "path_blocked"
  }
}
```

---

## 13. Registry views

Different modules need different slices of the registry.

| View | Consumer | Contents |
|---|---|---|
| planner view | `planner_llm` | high-level callable or expandable objects with prompt guidance |
| chatbot view | `chatbot_llm` | aliases, user-language affordances, routing hints |
| executor view | `nao_orchestrator` | only executable AB=1 mappings and approved macros |
| workbench view | `neural_workbench` | full decomposition, status, safety, trace/entropy metadata |
| dashboard view | `stack_observer` | graph nodes/edges, status, runtime mapping, trace IDs |

Acceptance rule:

```text
The planner should not receive massive AB=4 recipes.
The orchestrator should not receive proposal-only objects as executable.
The Workbench should receive full lineage.
```

---

## 14. Workbench integration

The Workbench should use the AB registry during:

### Candidate generation

```python
registry.find_by_category("navigation", max_ab_level=2)
registry.find_executable_or_expandable("wave_at")
```

### Candidate verification

```text
object exists
AB level appropriate
decomposition valid
executable expansion exists
safety OK
required params available
```

### Energy scoring

Add penalties:

```text
unsupported_object_penalty
proposal_without_expansion_penalty
ab_mismatch_penalty
unsafe_expansion_penalty
low_trace_support_penalty
```

### Trace storage

```json
{
  "selected_ab_object": "wave_at",
  "expanded_to": ["look_at", "wave_greet"],
  "ab_level": 2,
  "result_payloads": [],
  "outcome": "success"
}
```

### Crystallization

Repeated traces:

```text
find_object -> report_result
```

can become:

```text
find_object_and_report
```

as an AB=2 candidate.

---

## 15. Dashboard requirements

The dashboard should display:

```text
selected object
AB level
kind
implementation status
owner package
transport
decomposition children
parents / higher-level users
safety flags
trace support
entropy profile
```

For runtime events:

```text
active skill -> AB object -> decomposition -> result payload -> trace update
```

This makes supervisor demos much clearer.

---

## 16. Implementation phases

### Phase A — Schema and loader

- Add `ABObjectSpec`.
- Add `ab_registry.yaml`.
- Load registry.
- Validate required fields.
- Export compact JSON.

Acceptance:

- current JSON state can be loaded or migrated;
- AB=0 and AB=1 objects load;
- tests cover missing fields and invalid references.

### Phase B — Graph validation

- Validate object references.
- Validate acyclic decomposition graph.
- Validate allowed lower-level composition.
- Add `expand()` utility.

Acceptance:

- `wave_at` expands to `look_at`, `wave_greet`;
- invalid references fail tests;
- AB=3/AB=4 objects are not flattened by default.

### Phase C — Registry views

Export planner, chatbot, executor, workbench, and dashboard views.

Acceptance:

- orchestrator receives executable mappings only;
- Workbench receives full symbolic lineage.

### Phase D — Fake skill integration

- Add fake skill AB entries.
- Connect executor mappings.
- Add macro proposals for `wave_at`, `find_object_and_report`, and `inspect_area_and_report`.

Acceptance:

- fake skills route through registry;
- macro proposals expand to executable AB=1 steps.

### Phase E — Workbench verifier integration

- Verifier checks AB registry.
- Energy scorer uses AB mismatch and implementation status.
- Trace memory stores AB object lineage.

Acceptance:

- invalid macro candidate rejected;
- proposal macro can execute only if expanded;
- trace stores selected object and expansion.

### Phase F — Dashboard integration

- Render AB registry graph.
- Show selected object's decomposition.
- Show runtime event to AB object mapping.

Acceptance:

- dashboard can show `wave_at -> look_at -> wave_greet`.

### Phase G — Entropy profile support

- Add optional entropy profile fields.
- Add symbolic entropy proxy fields to traces.
- Update AB object empirical stats after traces.

Acceptance:

- entropy fields optional;
- no dependency on a full probabilistic WME;
- trace-derived stats can later update registry metadata.

---

## 17. Tests

Add tests:

```text
test_ab_object_schema.py
test_ab_registry_loader.py
test_ab_registry_reference_validation.py
test_ab_registry_dag_integrity.py
test_ab_expand_depth_control.py
test_ab_registry_views.py
test_ab_macro_proposal_status.py
test_ab_fake_skill_mapping.py
test_ab_workbench_verifier.py
```

Specific cases:

| Test | Expected |
|---|---|
| `wave_at` proposal | not directly executable without macro executor |
| `wave_at` expansion | expands to `look_at`, `wave_greet` |
| invalid child ref | loader validation fails |
| AB=2 direct AB=0 flattening | not default; only if `target_level=0` |
| fake `navigate_to` | has executor mapping and result schema |
| `navigate_with_recovery` | proposal unless implemented |
| deprecated object | rejected by planner/workbench |
| entropy fields missing | accepted with default `unmeasured` status |
| dashboard view export | includes nodes and decomposition edges |

---

## 18. Acceptance checklist

- One canonical AB registry exists.
- All executable planner-visible skills have AB entries.
- All AB=2+ objects have explicit decomposition or proposal status.
- `skill_common` can still provide a simple executable skill view.
- Workbench can access full symbolic lineage.
- Orchestrator can access only executable mappings.
- Dashboard can render decomposition graph.
- Fake skills have registry entries and executor mappings.
- Macro proposals are not silently treated as live executable skills.
- Verifier can reject unsupported or unsafe expansions.
- Trace memory can attach outcomes to AB object IDs.

---

## 19. Warnings

Do not:

- flatten all AB=4/AB=3 objects to AB=0 by default;
- let embeddings replace symbolic decomposition;
- treat proposal macros as executable without expansion or approval;
- let fake skills bypass `nao_orchestrator`;
- let skill descriptions drift separately in planner and chatbot;
- let registry become prompt-only documentation;
- remove backward-compatible simple skill views needed by existing planner/orchestrator code.

---

## 20. First PR recommendation

```text
PR 1: Add ABObjectSpec + ab_registry loader
PR 2: Add graph validation + expansion utilities
PR 3: Add registry views for planner/chatbot/orchestrator/workbench/dashboard
PR 4: Add fake skill AB entries and macro proposal objects
PR 5: Wire Workbench verifier to AB registry
PR 6: Dashboard decomposition view
```

---

## 21. Summary sentence

> The AB Registry provides a symbolic compositional lineage for every abstraction object, allowing the Neural Workbench to validate, expand, compress, compare, visualize, and eventually promote behaviours across abstraction levels without flattening all high-level strategies into primitive AB=0 recipes.
