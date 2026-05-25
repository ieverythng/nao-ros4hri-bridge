# ROS4HRI Integration Master Plan (Consolidated, Active)

**Date:** 2026-05-21  
**Branch context:** `feat/TFM-LLM_planner` (+ `feat(R)/full_nao_dashboard` implementation seam)  
**Scope:** Single active execution plan for non-fake-skill integration, AB registry normalization, and observability/dashboard rollout.

## 1. Consolidation Policy (What This File Replaces)

This file is now the canonical integration plan and absorbs execution tracking from:

- `docs/artifacts/plan_archive/ros4hri_integration_master_plan_2026-05-15.md`
- `docs/artifacts/plan_archive/ab_registry_decomposition_codex_handoff.md`
- `docs/artifacts/plan_archive/dashboard_implementation_roadmap.md`
- `docs/artifacts/plan_archive/simple_dialogue_trace_viewer_spec.md`
- `docs/artifacts/plan_archive/full_nao_dashboard_spec.md`

These source plans are preserved in `docs/artifacts/plan_archive/` for provenance.

The fake skills stream remains separate by design:

- `fake_skills_codex_handoff.md` (+ HTML)

## 2. Repo-Doc Structure Contract

To avoid docs churn, active docs are constrained to:

- `docs/contracts.md`
- `docs/current_workflow.md`
- `docs/launch_profiles.md`
- `docs/planner_status.md`
- `docs/architecture/ab_registry_input.json`
- `docs/architecture/ros4hri_neural_workbench_interactive_architecture.html`
- `docs/plans/` (this master plan + fake skills handoff)

Everything else should be archived under `docs/artifacts/` unless it is actively used in runtime operations.

## 3. Current Execution Status

### A. Planner-Orchestrator Runtime Path

- **Done**: `scan` is action-server owned and dispatched by orchestrator.
- **Done**: `report_result` now executes as action-server-owned AB=1 skill (`/skill/report_result`) instead of a dialogue-act shortcut.
- **Done**: planner/orchestrator action routing validated for `/skill/scan`, `/skill/report_result`, `/skill/say`, `/skill/do_head_motion`.
- **Done**: goal token/version guardrails and supersede semantics are present in planner-orchestrator seam.
- **Pending**: final route-hardening for ambiguous utterances (`knowledge_query` vs `execution`) in edge dialogue cases.

### B. Registry Consistency and Canonicalization

- **Done**: canonical registry contract aligned for `report_result` across planner + skill_common + architecture docs.
- **Done**: automated consistency checker added (`scripts/check_skill_registry_consistency.py`) and wired to pre-commit.
- **Done**: planner skill alias normalization aligned with canonical skill_common views.
- **Pending**: full AB decomposition migration (AB=2+ symbolic lineage and decomposition metadata).

### C. Lifecycle and Launch Reliability

- **Done**: current live stack can expose expected action servers and dispatch path reliably.
- **In progress**: reduce lifecycle-race/operator confusion in mixed sim/robot toggles.
- **Pending**: document strict operator guidance for “live stack already running” workflows to avoid duplicate launch side effects.

### D. Upstream/Nested Repo Reconciliation

- **Done**: upstream inventory exists (`docs/artifacts/upstream_sync_inventory_2026-05-14.md`).
- **Pending**: staged merge application order:
  1. `chatbot_msgs`
  2. `dialogue_manager`
  3. `chatbot_llm`
- **Pending**: align nested `skill_common` integration with Neural-Wokbench branch strategy (`feat/TFM_planner_only` flow).

### E. Fake Skills + Dashboard Runtime Seams

- **Done**: deterministic `fake_skills` action-server package exists with scenario-driven success/failure control and event publication on `/fake_skills/events`.
- **In progress**: stack launch wiring for `fake_skills` and `nao_dashboard` optional bring-up in sim/demo profiles.
- **In progress**: `nao_dashboard` package scaffold implemented with HTTP API, shared trace normalization, ROS graph snapshots, action-health panel, and AB registry projection through `skill_common`.
- **Pending**: add Workbench candidate/verification overlays once NW candidate feeds are available in this branch line.

## 4. Active Track: AB Registry Full Integration

This track is now narrowed to practical implementation milestones.

### AB-F1: Canonical AB Object Schema Stabilization (P0)

- Keep `ab_registry.json` as canonical symbolic source for AB objects.
- Keep `skill_registry.yaml` as planner-facing projection generated/aligned from canonical AB entries.
- Keep architecture mirror (`docs/architecture/ab_registry_input.json`) synchronized with canonical file.
- Enforce consistency through pre-commit hook.

**Exit criteria**

- No drift among canonical AB file, planner registry projection, and docs architecture mirror.
- CI/local pre-commit fails on mismatch.

### AB-F2: Decomposition Fields for AB>=2 (P1)

Add decomposition-ready fields in canonical AB schema for higher-level objects:

- ordered sub-objects
- control policy
- failure transitions
- validation constraints
- trace-support hints

**Exit criteria**

- at least 2 concrete AB=2 objects modeled with decomposition metadata (proposal status acceptable).

### AB-F3: Verification Utilities (P1)

- add graph integrity checks (existence, cycle prevention, level constraints).
- add schema compatibility checks for decomposition nodes.

**Exit criteria**

- automated tests for DAG integrity + decomposition validation run green.

### AB-F4: P3 Prompt + Nested Registry Updates (P3)

- tighten planner/chatbot routing prompts and response schemas for non-execution turns.
- publish structured chatbot routing trace events for dialogue vs planner handoff visibility.
- enforce canonical registry projection sync across planner fallback config and docs mirrors via pre-commit checks.
- route planner dialogue-act user wording through `chatbot_llm` by default, with direct wording only as explicit compatibility mode.

**Exit criteria**

- no drift between canonical AB registry and projected planner/docs registry surfaces.
- trace viewer can show planner/execution plus chatbot routing outcomes from structured payloads.
- planner clarification/failure/completion user-facing wording remains chatbot-owned in live launch profiles.

## 5. Active Track: Observability Dashboard Rollout

This track merges prior simple-viewer and full-dashboard plans.

### OBS-0: Shared Event Model (P0)

- define one normalized event model reusable by trace viewer and dashboard.
- include:
  - timestamp
  - source/channel
  - event_type
  - ab object id + level
  - summary
  - full payload

### OBS-1: Simple Dialogue Trace Viewer (P0)

- **Status: Done**
- lightweight trace tool for supervisor demos.
- required flow visibility:
  - user input
  - chatbot route
  - planner request/plan
  - orchestrator dispatch
  - skill result
  - planner dialogue act
  - final speech

### OBS-2: Full Dashboard Skeleton (P1)

- **Status: In progress**
- web backend + UI skeleton with:
  - live timeline
  - ROS graph snapshot
  - action server health panel
- initial implementation landed under `src/nao_dashboard/`:
  - backend node with normalized timeline ingestion
  - `/api/state`, `/api/events`, `/api/ros_graph`, `/api/ab_registry`
  - compact web hub UI (`index.html`, `app.js`, `styles.css`)
  - stack launch knobs for `start_nao_dashboard` and `start_fake_skills`

### OBS-3: AB Registry + Workbench Panels (P2)

- AB object graph pane
- fake/real skill status view
- candidate/verification overlays (as NW integration lands)

### OBS-4: Reports/Replay/Export (P2)

- JSONL traces
- static HTML report generation
- optional replay bundle for postmortems

## 6. Backlog (Prioritized, Cross-Track)

1. **P0** Finalize chatbot routing policy edge-cases (`knowledge_query` default where appropriate).
2. **P0** Add operator-facing troubleshooting section in `launch_profiles.md` for live-stack/no-relaunch constraints.
3. **P0** Keep `interaction_trace_viewer` aligned with dashboard shared event model.
4. **P1** Begin AB decomposition schema expansion and tests.
5. **P1** Complete dashboard P1 hardening: tests + launch-level smoke checks + operator quickstart.
6. **P2** Stage upstream nested-repo merges per inventory artifact.

## 7. Mandatory Validation Gates (Per Change Slice)

1. **Pre-edit audit**  
   `python3 scripts/ros4hri_change_audit.py --mode working`

2. **Deslop gate**  
   behavior-preserving simplification in touched files only.

3. **IIIA ROS4HRI gate**  
   ownership/lifecycle/interface guardrail check for affected packages.

4. **Registry consistency gate**  
   `python3 scripts/check_skill_registry_consistency.py`

5. **Runtime gate (no relaunch unless requested)**  
   for live stack checks:
   - action availability
   - one success path
   - one failure/supersede path where applicable

## 8. Documentation Hygiene Rules (Going Forward)

- New tactical notes go to `docs/artifacts/` unless they are active operator docs.
- Plans in `docs/plans/` must be either:
  - this master plan, or
  - a clearly separate active stream (currently fake skills).
- All plan files must keep `.md` + `.html` pairs.
- When a sub-plan is implemented, fold its status into this master plan and archive the sub-plan.

## 9. Immediate Next Session Checklist

1. Re-run focused tests for touched planner/orchestrator/registry seams.
2. Verify live stack action endpoints remain healthy (`scan`, `report_result`, `say`, `head_motion`).
3. Run launch-level smoke for `start_fake_skills=true` and `start_nao_dashboard=true` in sim profile.
4. Start AB-F2 decomposition metadata pass with tests.
5. Prepare upstream merge staging branch sequence from inventory doc.
