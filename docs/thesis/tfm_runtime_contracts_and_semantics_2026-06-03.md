# TFM Runtime Contracts and Semantics Reference

Date: 2026-06-03
Audience: TFM writing, supervisor review, implementation handoff
Scope: Planner request/output contracts, grounding semantics, execution feedback, and dialogue acts.

---

## Purpose

This document is the thesis-facing contract reference for the planner stack. It records the JSON shapes that connect dialogue, grounding, planning, execution, feedback, and planner dialogue. The goal is to make the implementation auditable and to give the thesis a precise vocabulary for discussing runtime behavior.

## Contract Design Principles

- Payloads should be concise enough to inspect during a live run.
- Fields should be owned by the node that can keep them truthful.
- Planner requests can contain T0 evidence; planner outputs should remain plan-centric.
- Duplicate fields should be removed unless they represent different ownership scopes.
- Human detections must be represented as people, not as generic objects.
- Natural-language summaries may help the LLM, but they should be bounded and secondary to structured JSON.

## Planner Request Envelope

The planner request travels as `hri_actions_msgs/msg/Intent`. The ROS envelope carries transport metadata, while `Intent.data` carries the planner request JSON.

```json
{
  "intent": "planner_request",
  "source": "user_123",
  "modality": "speech",
  "confidence": 0.82,
  "priority": 128,
  "data": {
    "request_id": "turn_123",
    "goal_id": "goal_turn_123",
    "request_kind": "new_goal",
    "goal_text": "look at the person and report completion",
    "normalized_intents": ["look_at"],
    "scene_targets": ["person"],
    "dialogue_context": [],
    "requested_plan": [],
    "grounded_context": {
      "knowledge_snapshot": {},
      "scene_summary": {},
      "state_t0": {}
    },
    "planner_mode": "default",
    "interaction_mode": "speech",
    "dialogue_turn_id": "role:turn"
  }
}
```

| Field | Meaning | Owner |
|---|---|---|
| `request_id` | Unique turn/request identifier | `chatbot_llm` |
| `goal_id` | Logical goal continuity identifier | `chatbot_llm`, admitted by orchestrator |
| `request_kind` | Transition type: `new_goal`, `goal_update`, `clarification_answer`, `cancel_request` | `chatbot_llm` |
| `goal_text` | Planner-facing task objective | `chatbot_llm` |
| `normalized_intents` | Strict intent labels | `chatbot_llm` |
| `scene_targets` | Compact target labels/entities | `chatbot_llm` |
| `grounded_context` | Hybrid Minimal T0 evidence | `chatbot_llm` |

## Knowledge Snapshot

`knowledge_snapshot` is a prompt-facing compact view of symbolic KB facts. It is not a raw KnowledgeCore transport object.

```json
{
  "schema_version": "knowledge_snapshot_v2",
  "captured_at_sec": 1777040000.2,
  "references": [
    {"normalized_name": "cup", "id": "cup_1", "type": "Cup"},
    {"normalized_name": "person", "id": "person_1", "type": "Person"}
  ],
  "counts": {"entities": 2, "people": 1, "objects": 1}
}
```

The `references` array is deliberately small: it gives the planner names, stable identifiers, and types without forcing it to parse large text blocks. The `counts` field supports quick consistency checks and allows prompts to mention cardinality without repeating entity lists.

## Scene Summary

`scene_summary` carries transient detector-grounded evidence. It includes provenance, recency, image-space coordinates, and source confidence. People and objects are separated.

```json
{
  "schema_version": "scene_summary_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.2,
  "objects": [
    {
      "entity_id": "cup_1",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.92,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 321.0,
      "center_y": 238.0,
      "last_seen_sec": 1777040000.1
    }
  ],
  "people": [
    {
      "id": "person_1",
      "label": "person",
      "type": "Person",
      "source": "emorobcare_cv",
      "score": 0.81,
      "center_x": 186.0,
      "center_y": 202.0,
      "last_seen_sec": 1777040000.2
    }
  ]
}
```

This representation preserves the useful image-space information needed for future `look_at` arguments while avoiding a duplicate `look_at_candidates` field. The planner prompt can state that every `state_t0.entities[*].id` is a valid candidate for `look_at.target_frame` when the entity is visible.

## State T0

`state_t0` is the canonical planner-facing snapshot for precondition and postcondition reasoning.

```json
{
  "schema_version": "state_t0_v2",
  "observer": "myself",
  "backend": "emorobcare_cv",
  "captured_at_sec": 1777040000.2,
  "entity_counts": {"entities": 2, "people": 1, "objects": 1},
  "entities": [
    {
      "normalized_name": "cup",
      "id": "cup_1",
      "type": "Cup",
      "kind": "object",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.1
    },
    {
      "normalized_name": "person",
      "id": "person_1",
      "type": "Person",
      "kind": "person",
      "source": "emorobcare_cv",
      "last_seen_sec": 1777040000.2
    }
  ]
}
```

The distinction between `scene_summary` and `state_t0` is important. `scene_summary` keeps richer detector metadata. `state_t0` is a normalized reasoning set. The former is closer to perception; the latter is closer to symbolic planning.

## Planner Output

Planner output travels on `/intents` as `hri_actions_msgs/msg/Intent`. The executable part is `Intent.data.plan`.

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
    "retry_budget": 2,
    "scene_targets": ["person"],
    "communication_policy": {
      "emit_acknowledge": false,
      "emit_progress": false,
      "emit_completion": true,
      "emit_failure": true
    },
    "steps": [
      {
        "id": "step_1",
        "type": "look_at",
        "name": "look_at",
        "args": {"target_frame": "person_1"},
        "requires": [],
        "on_failure": "replan",
        "retry_budget": 1
      }
    ]
  }
}
```

The plan is intentionally nested under `plan` to avoid duplicated top-level planning metadata. Removed legacy fields include `goal_token`, planner `ack_mode`, planner `ack_text`, `world_model_snapshot`, and `world_model_text`.

## Execution Feedback

Execution feedback is the closed-loop signal from `nao_orchestrator` to `planner_llm`.

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "intent": "look_at",
  "source": "nao_orchestrator",
  "event_type": "step_failed",
  "status": "failed",
  "reason": "target frame unavailable",
  "validation_status": "draft",
  "replan_hint": "scan for people before retrying look_at",
  "retry_budget": 1,
  "blocking": true,
  "unmet_preconditions": ["target_frame_visible"],
  "needs_user_input": false,
  "scene_targets": ["person"],
  "validation_errors": [],
  "timestamp_sec": 1777040012.4,
  "result_summary": "The requested person target was not available.",
  "result_payload": {
    "target_found": false,
    "target_kind": "person"
  },
  "step": {
    "id": "step_1",
    "type": "look_at",
    "name": "look_at",
    "retry_budget": 1,
    "on_failure": "replan",
    "requires": []
  }
}
```

Feedback is not just logging. It is the planner supervisor's evidence stream. It determines whether a goal can continue, whether a step needs replanning, or whether the user should be asked for clarification.

## Planner Dialogue Act

Planner dialogue acts carry planner-owned conversational intent. They do not make the planner the speech owner.

```json
{
  "goal_id": "goal_turn_123",
  "plan_id": "plan_123",
  "plan_version": 1,
  "act": "ask_clarification",
  "priority": "normal",
  "await_user_response": true,
  "reason": "multiple candidate people visible",
  "text_hint": "Which person should I look at?",
  "slots_needed": ["target_person"],
  "context": {
    "scene_targets": ["person"],
    "status": "waiting_user"
  }
}
```

The thesis should describe this as a separation between dialogue intention and utterance realization. The planner can say that clarification is needed; the dialogue stack remains responsible for interaction lifecycle and speech delivery.

## Semantics of Goal and Plan Identity

`goal_id` represents continuity of the user's objective. `plan_id` and `plan_version` represent the lineage of a particular proposed plan. This enables replanning without pretending that a new plan is a new user goal.

| Identifier | Meaning | Example |
|---|---|---|
| `goal_id` | Stable objective identity | `goal_turn_123` |
| `plan_id` | Plan lineage identity | `plan_123` |
| `plan_version` | Revision number within lineage | `2` |
| `step.id` | Stable step join point | `step_look_person` |
