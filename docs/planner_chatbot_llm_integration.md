# planner_llm And chatbot_llm Integration Notes

Last updated: 2026-04-02

This note captures the minimal sibling-repo work still required in
`chatbot_llm` so the new planner path can be exercised end to end without
changing upstream `dialogue_manager` contracts.

## Current State In This Repo

This repo now provides:

- `planner_llm`
  - consumes `/planner/request`
  - consumes `/world_model/enriched_snapshot`
  - consumes `/world_model/enriched_text`
  - optionally replans from `/planner/execution_feedback`
  - publishes executable `hri_actions_msgs/msg/Intent` messages on `/intents`
- `nao_world_model_enricher`
  - consumes `/scene/summary`
  - consumes `/planner/execution_feedback`
  - optionally reads KB rows through `kb_skills`
  - publishes `/world_model/enriched_snapshot`
  - publishes `/world_model/enriched_text`

This means the missing end-to-end seam is no longer planner execution. It is
only the handoff from `chatbot_llm` into `/planner/request`.

## Minimal Required `chatbot_llm` Patch

Keep the current architecture intact:

- `dialogue_manager` remains unchanged
- `chatbot_llm` still owns dialogue, response generation, and intent extraction
- `nao_orchestrator` still owns deterministic execution

Add one planner mode flag in `chatbot_llm`:

- `planner_mode_enabled` or similar boolean runtime parameter

When planner mode is disabled:

- preserve the current direct publication path to `/intents`

When planner mode is enabled:

- publish a `hri_actions_msgs/msg/Intent` message on `/planner/request`
- set `Intent.intent = "planner_request"`
- keep `Intent.source` and `Intent.modality` aligned with the current turn
- serialize the planner request payload into `Intent.data`

## Required `/planner/request` Payload

The JSON payload should contain:

- `request_id`
- `user_text`
- `normalized_intents`
- `ack_text`
- `ack_mode`
- `scene_targets`
- `dialogue_context`
- `grounded_context`
- `planner_mode`

Suggested field ownership:

- `request_id`: generated in `chatbot_llm` per user turn
- `user_text`: raw user turn or cleaned dialogue-manager text
- `normalized_intents`: current intent extraction output before robot execution
- `ack_text`: short acknowledgement already suitable for robot speech when needed
- `ack_mode`: existing or new marker such as `auto`, `silent`, or `explicit`
- `scene_targets`: objects or people already extracted from the turn
- `dialogue_context`: bounded recent conversational turns or summaries
- `grounded_context`: current KB-grounded scene context already used by `chatbot_llm`
- `planner_mode`: optional planner policy label such as `default`, `replan`, or `constrained`

## Important Boundary Rules

Do not move these responsibilities into `planner_llm`:

- response-style dialogue generation
- `dialogue_manager` service/action contracts
- KB querying logic already implemented for grounding and response generation
- dialogue session lifecycle

Do not move these responsibilities back into `chatbot_llm`:

- executable plan construction
- plan retry budgeting
- orchestration feedback consumption
- planner-only world-model summarization

## Recommended Rollout Order

1. Add the `/planner/request` publisher path behind a feature flag in `chatbot_llm`.
2. Keep the old `/intents` publication path as the fallback.
3. Run local tests with `nao_chatbot_planner_local.launch.py` plus fixture publishers.
4. Switch one local interaction flow to planner mode.
5. Only after that consider making planner mode the default for selected profiles.
