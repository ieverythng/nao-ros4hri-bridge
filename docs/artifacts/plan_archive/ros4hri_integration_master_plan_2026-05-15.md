# ROS4HRI Integration Master Plan (Consolidated)

**Date:** 2026-05-15  
**Branch context:** `feat/TFM-LLM_planner` + nested repos (`chatbot_llm`, `dialogue_manager`)  
**Scope:** Consolidated status of active ROS4HRI integration tracks, including fake-skill execution seams.

## 1. Source Plans Consolidated

This plan merges and supersedes the following implementation plans for active execution tracking:

- `codex_ros4hri_stack_hardening_handoff_2026-05-10.md` (+ HTML)
- `codex_skill_centric_ros4hri_handoff_2026-05-13_overlay.md` (+ HTML)
- `prompt_pack_hardening_plan_2026-05-09.html`

The fake skills handoff remains active and is now partially implemented:

- `fake_skills_codex_handoff.md` (+ HTML)

## 2. Explicit Out-of-Scope Items

- No GitNexus-generated docs refresh in this pass (left as-is intentionally).
- No migration of `wave_at` / `navigate_with_recovery` AB=2 macros into live executor dispatch yet (kept proposal-only).

## 3. Status Matrix (Done / In Progress / Pending)

## A. Core planner-executor-chatbot integration

- **Done**: Scan skill action-server ownership in runtime path, with orchestrator dispatch to `/skill/scan`.
- **Done**: Completion wording moved to chatbot-owned path using structured completion context from `dialogue_manager`.
- **Done**: Structured scan/evidence payload propagation in planner feedback/dialogue contexts.
- **Done**: Goal token/version arbitration foundation added across request/feedback/gate seams.
- **Done**: Stale plan-worker supersede handling added in orchestrator execution loop.

## B. Skill registry normalization

- **Done (policy decision updated)**: Local copied `src/skill_common` removed.
- **Done**: Planner/orchestrator/chatbot seams remain compatible with `skill_common` when provided by nested package source.
- **Done**: Runtime/unit validation run against nested `skill_common` in `src/Neural-Wokbench/src/skill_common`.
- **Done**: AB registry mappings updated to `fake_skills.*` adapters for fake executable skills.

## C. Fake skill execution substrate

- **Done**: New `src/fake_skills` ROS package with deterministic engine + scenario store.
- **Done**: Action servers created for `/skill/fake/execute`, `/skill/fake/navigate_to`, `/skill/fake/find_object`, `/skill/fake/wave_greet`, `/skill/fake/inspect_area`, and `/skill/fake/walk_to`.
- **Done**: Shared `SkillResultPayload` contract used via `skill_common` (with test fallback shim).
- **Done**: `nao_orchestrator` dispatch supports fake skill names discovered from `skill_common` adapter mappings.
- **Done**: Plan validation accepts fake skill names exported from `skill_common`.
- **Done**: Scenario-driven result modes include deterministic failure injection (`fail_once`, recoverable vs non-recoverable modes).
- **Pending**: Integrate optional fake-skill launch toggles into main `nao_chatbot` launch profiles.

## D. Dialogue/route correctness

- **Done**: Planner completion system payload no longer emits execution intents.
- **Pending**: Full memory/knowledge-vs-execution route hardening for ambiguous prompts (`person/people/room` execution-marker overreach).
- **Pending**: Final cleanup of dialogue-only intent leakage (`greet/help/identity/wellbeing`) on all edge paths.

## E. Ownership/architecture migration

- **In progress**: Planner gate exists with token/version checks and supersede semantics.
- **Pending**: Default ingress migration to `chatbot_llm -> /nao_orchestrator/planner_request -> /planner/request` across launch defaults and runtime validation.

## F. Upstream reconciliation

- **Done**: Upstream inventory artifact created with remotes/heads/conflict domains and merge order.
- **Pending**: Execute staged merges: `chatbot_msgs` contracts first, then `dialogue_manager`, then `chatbot_llm`.
- **Pending (explicit stashed-future bundle)**: Land upstream nested-repo sync changes tracked in `docs/artifacts/upstream_sync_inventory_2026-05-14.md` as a dedicated near-term integration sequence.

## 4. Remaining Implementation Backlog (Execution Order)

1. **P0** Fake skill live runtime validation:
- launch `fake_skills` with deterministic scenarios
- run one planner success and one planner replan path using fake skills
- confirm `/planner/execution_feedback.result_payload` propagation end-to-end

2. **P0** Route semantics hardening in `chatbot_llm/turn_engine.py`:
- knowledge/memory questions must route `knowledge_query` by default
- explicit new-scan imperatives must route `execution`

3. **P0** Runtime validation on live container/robot:
- lifecycle active checks
- action availability
- one success path and one failure/supersede path

4. **P1** Dialogue-only intent cleanup:
- ensure no execution-intent leakage for dialogue-only classes

5. **P1** Planner gate default ingress cutover in launch profiles:
- align launch defaults with orchestrator-gated ingress

6. **P2** Provider-neutral transport naming cleanup (`ollama_transport` compatibility shim strategy)

7. **P2** Upstream staged merge execution per inventory artifact

## 4.1 Stashed Future Changes (Must Land Soon)

These are intentionally queued but not yet applied in this branch snapshot:

1. `chatbot_msgs` source checkout/submodule decision and upstream remote wiring
2. `dialogue_manager` staged upstream reconciliation on `juan-feat-1`
3. `chatbot_llm` staged upstream reconciliation on `feat/juan_nao_chatbot`

Execution source of truth:

- `docs/artifacts/upstream_sync_inventory_2026-05-14.md`

## 5. Validation Gates (Mandatory Per Phase)

For each remaining backlog item:

1. Pre-edit audit:
- `python3 scripts/ros4hri_change_audit.py --mode working`

2. `deslop-refactor` gate:
- reduce control-flow duplication and dead branches in touched files only

3. `iiia-ros4hri-check` gate:
- ownership boundaries preserved
- lifecycle/contracts unchanged unless explicitly intended

4. Runtime gate:
- live container smoke checks (success + failure/supersede)

## 6. Docs Cleanup Actions Applied

- Consolidated active integration plan into this single markdown/html pair.
- Retained fake-skill handoff files as requested.
- Deprecated older plan artifacts removed from `docs/plans/`.

## 7. Next Session Start Checklist

1. Confirm nested `skill_common` availability from NeuralWorkbench branch `feat/TFM_planner_only` in the active workspace/container.
2. Re-run focused tests:
- `planner_common`, `planner_llm`, `nao_orchestrator`, `chatbot_llm`, `fake_skills`
3. Run fake-skill planner scenarios (`navigate_to:path_blocked`, `find_object:ambiguous`) and capture replanning behavior.
4. Execute live runtime gates on robot profile with planner mode enabled.
5. Continue P0 route hardening (`knowledge_query` vs `execution`).
