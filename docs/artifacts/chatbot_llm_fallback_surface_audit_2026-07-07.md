# Chatbot LLM Fallback Surface Audit, 7 July 2026

## Scope

This audit records the runtime and source seams that can sway chatbot or planner
behavior without changing the canonical prompt packs. It is evidence for
`ISSUE_TRACKER_FULL_SUITE.html` and `ISSUE_TRACKER_FAKE_SUITE.html`.

## Live Runtime Snapshot

- Container: `nao_ros2`, image `iiia:nao`.
- Pipeline: `response_first`.
- Dialogue manager: `active [3]`.
- `chatbot_grounded_context_digest_enabled`: `true` in the current live stack.
- Artifact: `/tmp/nao_runtime_snapshot_fallback_audit_20260707.json`.
- Fallback markers in the last 30 minute snapshot: total `0`.
- Observability caveat: two `/interaction_trace_viewer` nodes are present with
  the same node name.

Because digest is enabled, this live run is not a JSON-only grounded-context
ablation. If a response says a person and object are colocated despite correct
JSON, rerun with `chatbot_grounded_context_digest_enabled:=false` before
assigning blame to the structured `grounded_context`.

## Source Surfaces That Influence LLM Output

| Surface | File | Role | Risk |
| --- | --- | --- | --- |
| Canonical chatbot policy | `src/chatbot_llm/config/chat_prompt_pack.yaml` | Response and intent policy. | Prompt edits require SkillOpt ledger. |
| Canonical planner policy | `src/planner_llm/config/planner_prompt_pack.yaml` | Planner JSON and recovery policy. | Prompt edits require SkillOpt ledger. |
| System-turn addenda | `src/chatbot_llm/chatbot_llm/system_turn.py` | Structural wording tasks for planner completion, planner dialogue, and execution reports. | Can overconstrain report wording if it drifts into policy. |
| Grounded context projection | `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py` and `src/planner_common/planner_common/contracts.py` | Converts KnowledgeCore rows to compact JSON and optional digest. | Digest compression can introduce ambiguous wording. |
| Route heuristics | `src/chatbot_llm/chatbot_llm/route_heuristics.py` | Dialogue, KB query, and execution route safety nudges. | High leverage. Keep as safety checks, not policy replacement. |
| Response fallbacks | `src/chatbot_llm/chatbot_llm/response_fallbacks.py` | Bounded fallback wording and execution-report postprocessing. | Must not fabricate success or hide LLM regressions. |
| Planner request adapter | `src/chatbot_llm/chatbot_llm/planner_request_adapter.py` | Builds planner goal ids, normalized intents, scene targets, and grounded context payload. | Contract seam, not LLM policy. Safe for lineage fixes. |
| Planner engine recovery | `src/planner_llm/planner_llm/planner_engine.py` | Provider call, validation retry, and narrow structural fallbacks. | Useful only after provider/retry failure. Avoid preemptive deterministic planning. |
| Planner gate | `src/nao_orchestrator/nao_orchestrator/planner_gate.py` and `orchestrator.py` | Goal admission, active-goal lifecycle, and feedback. | Runtime gate should not become dialogue policy. |

## Stash Consolidation Decision

The nested `src/chatbot_llm` stash contained two different classes of change.

- Accepted narrow contract fix: local dialogue turn ids such as `__default__`
  no longer become reusable planner goal ids. They now fall back to
  `planner_common.make_goal_id()`.
- Rejected for now: the large `turn_engine.py` missing-named-person route guard.
  It would add deterministic route policy around the LLM. Keep it out unless a
  later runtime review proves the existing grounded-context and planner
  clarification seams cannot handle the case.

## New Review Instrumentation

The runtime review scripts now report fallback pressure without altering runtime
behavior.

- `collect_runtime_snapshot.py` writes `derived.fallback_metrics` and
  `derived.fallback_total_count`.
- `run_active_questionnaire.py` writes
  `phase_observations.fallback_markers` per case.

Tracked markers include LLM response fallback, rules fallback, planner invalid
JSON, invalid executable plan, planner gate rejection, duplicate active goal,
route repair, and user-facing language-model-unreachable fallback speech.

## Current Interpretation

The scene digest remains a credible source of response synthesis errors when it
compresses correct JSON into ambiguous prose. It is not the only possible cause
of KB follow-up failures. If digest is disabled and failures remain, focus next
on route fallback, planner request normalization, and KB mutation syntax.

For the next scored pass, use a clean rebuild with:

```bash
chatbot_turn_pipeline_mode:=response_first
chatbot_grounded_context_digest_enabled:=false
```

Then run the fake-deep grouped delivery and KB mutation holdouts before changing
prompt text or adding new route guards.
