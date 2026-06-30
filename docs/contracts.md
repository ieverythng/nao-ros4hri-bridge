# Runtime Contracts

Last updated: 2026-06-30

This document is the richer reference for the JSON payloads that move task,
scene, and execution state between nodes. The root README contains compact
examples; this file is the contract-focused view.

## chatbot_llm JSON Output

**Owner:** `chatbot_llm`
**Purpose:** response generation, route decision, intent declaration, planner routing.
**Important:** chatbot does not own executable plan steps.

```json
{
  "verbal_ack": "Okay, I will do that.",
  "route": "execution",
  "confidence": 0.82,
  "user_intent": {
    "type": "head_look_left",
    "goal": "look left",
    "goal_text": "look left",
    "scene_targets": [],
    "request_kind": "new_goal",
    "interaction_mode": "speech"
  }
```

**Allowed route values**

```text
dialogue | knowledge_query | execution
```

**Policy**

```text
chatbot_llm declares the user-facing intent and routes execution turns.
planner_llm owns executable planning and supervision.
nao_orchestrator owns deterministic execution.
visibility-only scene checks default to knowledge_query unless explicit scan/action wording is present.
```

---

## Grounded Context

Owner:

- `chatbot_llm` builds the compact planner-facing projection.
- `planner_common` validates and normalizes the contract shape.
- `nao_scene_grounding` and `kb_skills` remain the owners of detector and
  KnowledgeCore facts.

Purpose:

- provide a bounded symbolic view for chatbot routing, planner prompts, and
  deterministic validation.
- preserve people, deliverable objects, support surfaces, rooms, and navigation
  targets as different roles.
- keep raw RDF and detector details available at their owning seams instead of
  exposing ontology helper classes as user-facing objects.

Current compact shape:

```json
{
  "schema_version": "grounded_context_v3",
  "captured_at_sec": 1782840000.0,
  "observer": "myself",
  "entities": [
    {
      "id": "codex_kitchen_cup",
      "label": "red cup",
      "kind": "object",
      "class": "Cup",
      "visible": true,
      "relations": [
        {"predicate": "dbp:name", "object": "TITAS"},
        {"predicate": "dbp:color", "object": "red"},
        {"predicate": "oro:isIn", "object": "codex_kitchen"}
      ]
    },
    {
      "id": "codex_recipient_person",
      "label": "ALEX",
      "kind": "person",
      "class": "Human",
      "visible": true,
      "relations": [
        {"predicate": "dbp:name", "object": "ALEX"},
        {"predicate": "oro:isIn", "object": "handoff_area"}
      ]
    }
  ],
  "locations": [
    {
      "id": "codex_kitchen",
      "label": "kitchen",
      "kind": "location_group",
      "role": "navigation_target",
      "member_count": 2,
      "object_count": 2,
      "person_count": 0,
      "contains": [
        {
          "id": "codex_kitchen_cup",
          "label": "red cup",
          "kind": "object",
          "class": "Cup",
          "relation": "oro:isIn"
        },
        {
          "id": "codex_kitchen_book",
          "label": "blue book",
          "kind": "object",
          "class": "Book",
          "relation": "oro:isIn"
        }
      ]
    }
  ],
  "counts": {"entities": 2, "people": 1, "objects": 1, "locations": 1}
}
```

Role policy:

- `entities` is the stable subject inventory. Each entity keeps its type,
  label, visibility flag, and bounded relations.
- `locations` is a derived compact view. It groups members by support or place
  relation, but it does not replace `entities`.
- Support surfaces such as tables, desks, counters, and shelves may form
  `support_group` entries. Rooms, kitchens, corridors, labs, and robot stations
  may form `navigation_target` or `location_group` entries.
- People remain `person` entities and recipients. A person is not treated as a
  location, even if a pose or room relation is available.
- User-facing object lists filter ontology and support/meta entries. Do not
  expose `owl:Thing`, `cyc:SpatialThing*`, `Location`, `Place`, support
  surfaces, rooms, or tables as deliverable objects unless the user explicitly
  asks about those categories.
- Relation aliases such as `isContainedIn`, `placeOf`, and `isAt` are normalized
  into the compact predicates used by the planner view.

Admission policy:

- Execution requests that name a human recipient or target must be checked
  against the current grounded context before planner handoff.
- If the request names a person that is not present in `entities`, chatbot
  routing must ask for clarification instead of handing an executable request to
  the planner.
- Stale or absent facts should produce a truthful clarification, help request,
  replan, or failure. They must not be hidden behind generic object names.

Implementation references:

- source projection and filtering: `src/planner_common/planner_common/contracts.py`
- chatbot digest projection: `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py`
- planner admission and fallback behavior:
  `src/planner_llm/planner_llm/planner_engine.py`
- runtime evidence plan:
  `docs/plans/CRITIC_RUNTIME_HARDENING_2026-06-30.md`

## Planner Request

Topic:

- `/nao_orchestrator/planner_request` (chatbot -> orchestrator gate ingress)
- `/planner/request` (orchestrator-admitted request -> planner ingress)
- type: `hri_actions_msgs/msg/Intent`
- publisher (ingress): `chatbot_llm`
- publisher (`/planner/request`): `nao_orchestrator`
- consumer: `planner_llm`

ROS envelope policy:

- `Intent.priority` and `Intent.confidence` are part of
  `hri_actions_msgs/msg/Intent`, so they will appear in both
  planner-ingress topics.
- `chatbot_llm` sets deterministic planner priority/confidence on ingress
  requests, and `nao_orchestrator` planner-gate forwards admitted requests to
  `/planner/request`.
- Planner semantics live in `Intent.data`; do not duplicate priority or
  confidence inside the JSON payload unless a planner policy genuinely needs it.

Preferred payload:

```json
{
  "request_id": "turn_123",
  "goal_id": "goal_turn_123",
  "parent_goal_id": "",
  "supersedes_goal_id": "",
  "request_kind": "new_goal",
  "goal_text": "navigate to the kitchen and report completion",
  "normalized_intents": ["navigate_to"],
  "scene_targets": ["kitchen"],
  "dialogue_context": [],
  "requested_plan": [],
  "grounded_context": {
    "schema_version": "grounded_context_v3",
    "entities": [
      {
        "id": "codex_kitchen",
        "label": "kitchen",
        "kind": "location",
        "class": "Room",
        "visible": true
      },
      {
        "id": "codex_recipient_person",
        "label": "ALEX",
        "kind": "person",
        "class": "Human",
        "visible": true,
        "relations": [{"predicate": "dbp:name", "object": "ALEX"}]
      }
    ],
    "locations": [
      {
        "id": "codex_kitchen",
        "label": "kitchen",
        "role": "navigation_target",
        "contains": [
          {
            "id": "codex_kitchen_cup",
            "label": "red cup",
            "kind": "object",
            "class": "Cup",
            "relation": "oro:isIn"
          }
        ]
      }
    ]
  },
  "planner_mode": "default",
  "interaction_mode": "speech",
  "dialogue_turn_id": "role:turn"
}
```

Field policy:

- `goal_text`: concise planner-facing objective parsed by
  `planner_common.PlannerRequest` and sent to `planner_llm`.
- `normalized_intents`: strict intent labels from `chatbot_llm`.
- `scene_targets`: target labels/entities extracted by `chatbot_llm`.
- `grounded_context`: bounded symbolic context.
- `requested_plan`: optional hint or fallback. It should not be required for the
  planner to abstract a simple task.
- `user_text`: legacy parser input accepted by `PlannerRequest` for backward
  compatibility, but omitted from normal `chatbot_llm` planner requests and
  from the `planner_llm` prompt payload.

## Planner Output

Topic:

- `/intents`
- type: `hri_actions_msgs/msg/Intent`
- publisher: `planner_llm`
- consumer: `nao_orchestrator`

Payload:

```json
{
  "grounded_context": {
    "knowledge_snapshot": {},
    "scene_summary": {},
    "state_t0": {}
  },
  "plan": {
    "goal_id": "goal_turn_123",
    "plan_id": "plan_123",
    "plan_version": 1,
    "status": "planning",
    "validation_status": "draft",
    "failure_reason": "",
    "user_facing_reason": "",
    "replan_hint": "",
    "retry_budget": 1,
    "scene_targets": ["kitchen"],
    "communication_policy": {
      "emit_acknowledge": false,
      "emit_progress": false,
      "emit_completion": true,
      "emit_failure": true
    },
    "steps": [
      {
        "id": "step_1",
        "type": "skill",
        "name": "perform_motion",
        "args": {"object": "head_look_left"},
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 0
      }
    ]
  }
}
```

Ownership note:

- `grounded_context` in planner output is currently a transitional echo for
  traceability.
- AB=1 skill execution must continue to resolve live world state from AB=0
  seams (`/scene/summary`, `/kb/query`, tracked-person topics/services), not
  from planner output payload copies.
- When the context-ref seam is promoted, planner output should carry only
  lightweight lineage such as:

```json
{
  "plan": {
    "context_ref": {
      "captured_at_sec": 1777040000.0,
      "observer": "myself",
      "backend": "emorobcare_cv"
    }
  }
}
```

Allowed step types:

- `noop`
- `say`
- `skill`
- `look_at`

Allowed failure policies:

- `fail`
- `continue`
- `replan`
- `clarify`
- `ask_user`
- `ignore`

Current limitation: unsupported planner steps can be filtered by the skill
registry. The next diagnostic should verify whether multi-step plans can degrade
into partial plans without being classified as invalid.

## Execution Feedback

Topic:

- `/planner/execution_feedback`
- type: `std_msgs/msg/String`
- publisher: `nao_orchestrator`
- consumer: `planner_llm`

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "intent": "raw_user_input",
  "source": "planner_llm",
  "event_type": "step_failed",
  "status": "failed",
  "reason": "target action server unavailable",
  "validation_status": "draft",
  "replan_hint": "",
  "retry_budget": 1,
  "blocking": true,
  "unmet_preconditions": [],
  "needs_user_input": false,
  "scene_targets": ["kitchen"],
  "validation_errors": [],
  "timestamp_sec": 1777040000.0,
  "result_summary": "I found one person (id: anonymous_person_daeba).",
  "result_payload": {
    "skill": "scan",
    "target": "people",
    "target_kind": "people",
    "target_found": true,
    "people": [
      {
        "id": "anonymous_person_daeba",
        "source": "hri_tracked_persons",
        "last_seen_age_sec": 0.4
      }
    ],
    "objects": [],
    "summary_text": "I found one person (id: anonymous_person_daeba).",
    "confidence_policy": "grounded_current_observation"
  },
  "plan_outcome_summary": {
    "completed_targets": ["anonymous_person_daeba"],
    "failed_targets": [],
    "pending_targets": [],
    "last_successful_step_id": "step_1",
    "terminal_step_id": "step_1",
    "terminal_reason": "completed",
    "all_required_steps_succeeded": true
  },
  "step": {
    "id": "step_1",
    "type": "skill",
    "name": "perform_motion",
    "retry_budget": 1,
    "on_failure": "replan",
    "requires": []
  }
}
```

`result_summary` remains the backward-compatible short text mirror.
`result_payload` carries the typed skill result (for scan/person evidence).
`plan_outcome_summary` is structured executor evidence, not user-facing prose.
It lets planner supervision and report-result wording distinguish completed,
failed, and pending targets without asking any node to infer that state from a
free-text summary.

Important event types:

- `plan_accepted`
- `step_started`
- `step_succeeded`
- `step_failed`
- `plan_invalid`
- `plan_completed`

## Planner Dialogue Act

Topic:

- `/planner/dialogue_act`
- type: `std_msgs/msg/String`
- publisher: `planner_llm`
- consumer: `dialogue_manager`
- dialogue policy: direct-mode planner acts; completion wording may be relayed
  through `chatbot_llm` when available

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "act": "ask_clarification",
  "priority": "normal",
  "await_user_response": true,
  "reason": "missing target object",
  "text_hint": "Which target should I use?",
  "slots_needed": ["target"],
  "context": {
    "scene_targets": ["kitchen"],
    "status": "waiting_user"
  }
}
```

Allowed dialogue acts:

- `acknowledge`
- `progress_update`
- `ask_clarification`
- `ask_for_help`
- `explain_failure`
- `notify_completion`
- `notify_cancellation`

## Knowledge Snapshot

Owner:

- `chatbot_llm`

Source:

- `/kb/query` through `kb_skills`

Purpose:

- prompt context for response and intent/planner routing stages.

`knowledge_snapshot` is the chatbot's textual/symbolic view of KnowledgeCore
facts. It is useful because it gives the LLM a compact fact set without exposing
raw KB transport details.

Default query group:

```text
myself sees ?entity && ?entity rdf:type ?type
```

`grounded_context.knowledge_snapshot` example:

```json
{
  "schema_version": "knowledge_snapshot_v2",
  "captured_at_sec": 1777040000.0,
  "references": [
    {"normalized_name": "cup", "id": "cup_1", "type": "Cup"},
    {"normalized_name": "person", "id": "person_1", "type": "Person"}
  ],
  "counts": {"entities": 2, "people": 1, "objects": 1}
}
```

This is not a `knowledge_core` native concept; it is the local prompt-facing
formatting layer in `chatbot_llm`.

## Scene Summary

Topic:

- `/scene/summary`
- type: `std_msgs/msg/String`
- publisher: `nao_scene_grounding`
- consumer: `chatbot_llm` and operator/debug tooling

Purpose:

- transient detector-grounded observation summary.
- carries detection metadata that is not yet represented as KB facts, including
  labels/classes, scores, image positions, observer/source, and recency.
- supports debugging and future `object_manager`/world-model work without
  making the planner subscribe directly to detector output.

Current policy:

- Keep `/scene/summary` while the KB only stores the selected symbolic facts
  needed for reasoning, such as `myself sees entity` and `entity rdf:type Type`.
- If the KB/object-manager schema later stores confidence, position,
  provenance, and recency directly, the scene summary can become mostly
  operator/debug output.

Example payload:

```json
{
  "schema_version": "scene_summary_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.0,
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.91,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 320.0,
      "center_y": 240.0,
      "last_seen_sec": 1777040000.0
    }
  ],
  "people": [
    {
      "id": "person_1",
      "label": "person",
      "type": "Person",
      "source": "emorobcare_cv",
      "score": 0.88,
      "center_x": 210.0,
      "center_y": 180.0,
      "last_seen_sec": 1777040000.0
    }
  ]
}
```

`/scene/summary` is not the same as `knowledge_snapshot`: it is a current object
summary, while `knowledge_snapshot` is prompt-ready text from KB queries.
Within `grounded_context.scene_summary`, people-like detections are promoted to
`people`, and `objects` remains object-only.

## State T0 Context

`grounded_context.state_t0` carries deterministic planner-facing context for
pre/postcondition reasoning without free-text world-model seams.

Planner guidance policy:

- treat every `state_t0.entities[*].id` as a valid `look_at.target_frame`
  candidate; do not require a dedicated `look_at_candidates` payload field.

Example:

```json
{
  "schema_version": "state_t0_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.0,
  "entity_counts": {"entities": 2, "people": 1, "objects": 1},
  "entities": [
    {
      "normalized_name": "cup",
      "id": "cup_1",
      "type": "Cup",
      "kind": "object",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.0
    },
    {
      "normalized_name": "person",
      "id": "person_1",
      "type": "Person",
      "kind": "person",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.0
    }
  ]
}
```
