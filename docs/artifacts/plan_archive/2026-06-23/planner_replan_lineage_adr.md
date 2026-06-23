# ADR: Planner Replan Lineage And Hybrid T0 Contract

## Status

Accepted on 2026-05-31 for `feat/TFM-LLM_planner`.

## Context

Planner-facing payload seams had grown redundant across packages:

- duplicated plan metadata at multiple envelope levels
- stale `goal_token` ownership logic
- dead world-model snapshot/text fields
- mixed responsibilities for acknowledgement wording

At the same time, the executor needed a deterministic way to continue work when
the planner publishes a newer plan for the same goal.

## Decision

### 1. Canonical planner payload shape

- Planner metadata is canonical under nested `plan`.
- Top-level duplicated planning fields are removed.
- Removed from planner contracts and normalizers:
  - `goal_token`
  - `world_model_snapshot`
  - `world_model_text`
  - planner-level `ack_mode`
  - planner-level raw `ack_text`

### 2. Grounded context contract: Hybrid Minimal T0

- Keep `grounded_context` as:
  - `knowledge_snapshot`
  - `scene_summary`
  - `state_t0`
- `knowledge_snapshot` holds compact KB references (`normalized_name`, `id`, `type`).
- `state_t0` keeps deterministic planning facts (`observer`, `backend`, `captured_at_sec`,
  entity/source/last-seen fields) required for pre/postcondition reasoning.
- Free-text world-model seams are removed from planner ingress and plan payloads.

### 3. Orchestrator execution lineage

- Queue model: one active goal at a time.
- Identity model:
  - `goal_id` for goal continuity
  - `plan_id` + `plan_version` for plan lineage
- Replan join policy:
  - try mid-join by active `step_id` when the new plan is a newer version of
    the same `goal_id`
  - fallback to front-join when no step mapping exists
- Duplicate suppression checks exact active tuple:
  - `goal_id`, `plan_id`, `plan_version`

### 4. Transition contract (without token locks)

- `new_goal`: accepted when no active goal, or supersedes current goal.
- `goal_update`: must match active goal lineage.
- `clarification_answer`: must match active goal lineage.
- `cancel_request`: clears active goal lineage.

### 5. Dialogue ownership guardrail

- Planner emits dialogue acts and intent-level reasoning only.
- User-facing completion wording remains chatbot-owned via the existing
  planner-dialogue relay path.

## Implementation Notes

Primary touchpoints:

- `src/planner_common/planner_common/contracts.py`
- `src/nao_orchestrator/nao_orchestrator/planner_gate.py`
- `src/nao_orchestrator/nao_orchestrator/orchestrator.py`
- `src/chatbot_llm/chatbot_llm/planner_handoff.py`
- `src/chatbot_llm/chatbot_llm/planner_request_adapter.py`

## Deferred Work

- Full multi-goal scheduling is deferred.
- Full executor queue rewrite is deferred.
- Mid-join currently relies on stable `step_id` emission; preserving that
  stability remains mandatory for future planner changes.
