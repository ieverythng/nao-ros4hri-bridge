# NAO ROS4HRI Masterplan (Consolidated, Active)

**Date:** 2026-06-23 (runtime recovery refresh)
**Branch context:** `feat/TFM-LLM_planner` (+ nested repos and Neural-Wokbench integration seam)
**Scope:** Single active execution plan for planner/chatbot/orchestrator seams,
grounded-context reliability, canonical registry alignment, fake-skill
operational hardening, LocateAnything migration, and validation reporting.

## 1. Consolidation Policy (What This File Replaces)

This file is now the canonical integration plan and absorbs execution tracking from:

- `docs/artifacts/plan_archive/ab_registry_decomposition_codex_handoff.md`
- `docs/artifacts/plan_archive/dashboard_implementation_roadmap.md`
- `docs/artifacts/plan_archive/simple_dialogue_trace_viewer_spec.md`
- `docs/artifacts/plan_archive/full_nao_dashboard_spec.md`

The older `ros4hri_integration_master_plan_2026-05-15` baseline has now been
fully merged into this file and removed from the archive to keep one canonical
masterplan surface.

The remaining source plans are preserved in `docs/artifacts/plan_archive/` for
provenance.

The fake-skills handoff stream has now been archived. Durable fake-skill
requirements live in this masterplan, the fake-skills scenario playbook, and the
runtime review tracker.

## 2. Repo-Doc Structure Contract

To avoid docs churn, active docs are constrained to:

- `docs/contracts.md`
- `docs/current_workflow.md`
- `docs/launch_profiles.md`
- `docs/planner_status.md`
- `docs/architecture/ab_registry_input.json`
- `docs/architecture/ros4hri_neural_workbench_interactive_architecture.html`
- `docs/architecture/demo_stack_seam_contract_2026-05-26.md` (+ `.html`)
- `docs/architecture/fake_skills_scenarios_playbook.md` (+ `.html`)
- `docs/plans/` (this master plan + active runtime/showing plans)

Everything else should be archived under `docs/artifacts/` unless it is actively used in runtime operations.

## 3. Current Execution Status

### A. Planner-Orchestrator Runtime Path

- **Done**: `scan` is action-server owned and dispatched by orchestrator.
- **Done**: `report_result` now executes as action-server-owned AB=1 skill (`/skill/report_result`) instead of a dialogue-act shortcut.
- **Done**: planner/orchestrator action routing validated for `/skill/scan`, `/skill/report_result`, `/skill/say`, `/skill/do_head_motion`.
- **Done (2026-06-12)**: real head motion is strict by default; convergence timeout is reported as execution failure unless an explicit debug override enables open-loop success.
- **Done (2026-06-12)**: fake `perform_motion` is available for validation runs and is selected only through explicit orchestrator launch/config mode, keeping fake outcomes scenario-controlled.
- **Done**: planner lineage now uses `goal_id` continuity plus `plan_id`/`plan_version`; token-based ownership seams were removed.
- **Done**: route-hardening now defaults visibility-only scene checks to `knowledge_query` unless explicit scan/action wording is requested.
- **Done**: planner dialogue acts run in direct mode by default, while completion wording stays chatbot-relay-owned when a chatbot client is available.
- **Done**: planner grounding contracts now use Hybrid Minimal T0 (`knowledge_snapshot`, `scene_summary`, `state_t0`) with world-model seams removed.
- **Done (2026-06-23)**: seam contract hardening conclusions are folded into the active source and this masterplan. The dated seam contract plan is archived under `docs/artifacts/plan_archive/2026-06-23/`.
- **Done (2026-06-23 source gate)**: compact grounded context now derives a `locations` view from `oro:isIn`, `oro:isAt`, `oro:isOn`, and `oro:contains`, while preserving the existing `entities` contract.
- **Done (2026-06-23 source gate)**: planner has a late, bounded fallback for grounded “bring every object from location X to recipient Y” requests when model output remains invalid after one repair attempt.
- **Done (2026-06-23 source gate)**: successful skill result payloads can apply structured `evidence.kb_effects` through the orchestrator’s existing `/kb/revise` boundary. This starts with fake manipulation skills and is reusable by real skills that emit the same payload shape.
- **Done**: structured `chatbot_turn_trace` visibility is available for dialogue vs planner-handoff attribution.
- **Done (2026-05-26)**: planner-mode routing now guards visibility-only scene questions toward `knowledge_query` unless the user explicitly requests a new scan/action.
- **In progress**: proactive wording + speech arbitration pass to avoid duplicate user-facing utterances when execution acknowledgements and planner dialogue completions occur in the same interaction.
- **In progress**: live rebuild proof for the new location-group and KB-effect seams. Source tests pass, but runtime score should not be raised until a fresh response-first run proves the updated container behavior.

### B. Registry Consistency and Canonicalization

- **Done**: canonical registry contract aligned for `report_result` across planner + skill_common + architecture docs.
- **Done**: automated consistency checker added (`scripts/check_skill_registry_consistency.py`) and wired to pre-commit.
- **Done**: planner skill alias normalization aligned with canonical skill_common views.
- **Pending**: full AB decomposition migration (AB=2+ symbolic lineage and decomposition metadata).

### C. Lifecycle and Launch Reliability

- **Done**: current live stack can expose expected action servers and dispatch path reliably.
- **Done (2026-06-12)**: sim/robot profile defaults no longer accept head-motion convergence timeout as success; fake validation can opt into `perform_motion_execution_mode=fake`.
- **In progress**: reduce lifecycle-race/operator confusion in mixed sim/robot toggles.
- **Done**: interaction trace viewer can be run as a separate operator window; sim default no longer auto-launches it.
- **Done**: compact trace channel/event filtering args are exposed through stack launch and can be toggled without code edits.
- **Done (2026-05-26)**: fixed fake-skills launch coercion seam where `fake_skill_mode_overrides_json` could be treated as dict and abort startup (`ParameterValue(..., value_type=str)`).
- **In progress (2026-05-26 live probe)**: planner request-admission behavior after entering `waiting_user` needs hardening; later `/planner/request` fixtures were trace-visible but not admitted by `planner_llm` in the same run.

### D. Grounding, LocateAnything, And Skill-Aware KB Effects

- **Done**: response-first runtime evidence shows simple dialogue, simple KB
  query, explicit KB mutation, fake-skill KB guards, and maximal
  kitchen-cup-to-person service-path success.
- **Done (source)**: compact grounded context now exposes location groups without
  replacing `entities`, so existing consumers remain compatible.
- **In progress**: location-aware execution validation for prompts such as
  “bring every object from the kitchen to ALEX”.
- **Done (harness source)**: runtime-review now supports named preloaded
  KnowledgeCore environment fixtures through `--preload-environment` and the
  dedicated `environment` case set. The first fixture pack includes
  `lab_table`, `kitchen_delivery`, and `gold_apple_handoff`, plus a visual SVG
  companion for the kitchen-delivery scene.
- **In progress**: skill-aware KB post-effects. Fake skills already report
  `evidence.kb_effects`; orchestrator now applies successful effects through
  KnowledgeCore. Real skills should adopt the same payload contract before
  direct KB mutation is enabled for them.
- **Pending**: LocateAnything migration into the grounding stack. Keep it as a
  perception/grounding provider, not as a planner or dialogue policy owner.
- **Pending**: location lifecycle semantics for movement/manipulation, including
  removal of stale support/location facts after pick, place, and bring.

**Exit criteria**

- A response-first runtime review proves grouped-location KB queries, location
  scoped delivery, and post-skill KB changes on the next turn.
- `robot-runtime-performance-review --case-set environment` proves the same
  scene can be preloaded deterministically before speech or service turns.
- Fake and real skill payloads use the same `kb_effects` shape for successful
  state changes.
- LocateAnything facts enter the same grounded-context projection as other
  perception or simulator facts.

### E. Upstream/Nested Repo Reconciliation

- **Done**: upstream inventory exists (`docs/artifacts/upstream_sync_inventory_2026-05-14.md`).
- **Pending**: staged merge application order:
  1. `chatbot_msgs`
  2. `dialogue_manager`
  3. `chatbot_llm`
- **Pending**: align nested `skill_common` integration with Neural-Wokbench branch strategy (`feat/TFM_planner_only` flow).

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

### AB-F4: Prompt Hardening + Nested Registry Updates (P0)

Prompt hardening is now a critical seam track (not backlog-only), because route
misclassification can trigger duplicate planner/chatbot utterances.

**Phase status (SkillOpt baseline: 2026-05-27)**

| Phase | Status | Notes |
| --- | --- | --- |
| F4-A Chatbot route hardening (`dialogue` vs `knowledge_query` vs `execution`) | **DONE** | Greeting/social turns now explicitly default to dialogue unless action is explicit; execution acknowledgements are constrained to intent-to-act wording. |
| F4-B Planner prompt hardening for social/greeting spillover | **DONE** | Planner prompt now forbids inferring `wave_greet` from greeting-only text and requires `decision=clarify` for social-only requests without explicit action. |
| F4-C Canonical prompt-pack loading | **DONE** | Chatbot and planner prompt text now comes from canonical YAML prompt packs; missing/invalid required prompt fields fail fast instead of falling back to hidden Python prompt prose. |
| F4-D Live seam validation in rebuilt container | **MISSING** | Must run full stack and verify first-turn greeting does not produce planner execution or duplicate speech. |
| F4-E Prompt mutation cadence and regression suite | **IN PROGRESS** | Continue bounded SkillOpt iterations using trace-backed train/holdout cases. |

**Prompt-hardening references used**

- `docs/planner_status.md` (known weak spot: occasional execution over-routing)
- `docs/architecture/demo_stack_seam_contract_2026-05-26.md` (ownership + duplicate-speech guardrails)
- `docs/launch_profiles.md` (planner/dialogue wording mode and planner ingress args)
- `src/chatbot_llm/test/test_turn_engine.py` (greeting/KB/execution route expectations)
- `src/planner_llm/test/test_planner_engine.py` (planner output-contract enforcement)
- `docs/artifacts/prompt_hardening_skillopt_2026-05-27.md` (iteration log + acceptance gate)

**Prompt-hardening seam targets**

- tighten planner/chatbot routing prompts and response schemas for non-execution turns.
- publish structured chatbot routing trace events for dialogue vs planner handoff visibility.
- enforce canonical registry projection sync across planner fallback config and docs mirrors via pre-commit checks.
- keep planner dialogue-act payload ownership in planner/orchestrator seam, with live launch defaults using direct dialogue_manager wording (no auto chatbot rewording).
- harden planner-mode KB visibility routing so non-action perception checks prefer `knowledge_query`.

**Exit criteria**

- no drift between canonical AB registry and projected planner/docs registry surfaces.
- trace viewer shows chatbot route, planner request, planner decision, planner/execution outcomes, and final user-facing speech with one authority per turn (from structured payloads).
- first-turn social greeting remains `dialogue` in planner mode (no planner request publish).
- planner clarification/failure/completion user-facing wording remains chatbot-owned in live launch profiles, with deterministic ownership and no planner-dialogue auto re-entry into chatbot.

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

### OBS-1: Simple Dialogue Trace Viewer (P0) — Done

- delivered: lightweight trace tool for supervisor demos.
- flow visibility includes:
  - user input
  - chatbot route
  - planner request/plan
  - orchestrator dispatch
  - skill result
  - planner dialogue act
  - final speech
- JSON-only payload rendering is supported in verbose mode, with compact/verbose runtime toggles and channel/event filtering.

### OBS-2: Full Dashboard Skeleton (P1)

- web backend + UI skeleton with:
  - live timeline
  - ROS graph snapshot
  - action server health panel

### OBS-3: AB Registry + Workbench Panels (P2)

- AB object graph pane
- fake/real skill status view
- candidate/verification overlays (as NW integration lands)

### OBS-4: Reports/Replay/Export (P2)

- JSONL traces
- static HTML report generation
- optional replay bundle for postmortems

## 6. Backlog (Prioritized, Cross-Track)

1. **P0** Complete speech-ownership arbitration so each turn has one user-facing utterance authority (no duplicate execution-ack + planner-dialogue speech).
2. **P0** Live-prove preloaded environment fixtures, grouped-location delivery, and post-skill KB effects in the rebuilt response-first stack.
3. **P0** Resolve planner request-admission/backpressure seam after `waiting_user` transitions; ensure subsequent `new_goal` requests are deterministically handled (accepted/superseded/rejected with explicit reason).
4. **P1** Continue AB decomposition schema expansion and tests (AB=2+ lineage coverage).
5. **P1** Begin LocateAnything migration through the grounding adapter layer.
6. **P1** Begin dashboard backend skeleton (`nao_dashboard`).
7. **P2** Stage upstream nested-repo merges per inventory artifact.

## Additional Runtime Evidence

- Live stack validation artifact:
  - `docs/artifacts/runtime_validation_report_2026-05-26_stack_live.md`

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
  - a clearly separate active runtime/showing stream.
- All plan files must keep `.md` + `.html` pairs.
- When a sub-plan is implemented, fold its status into this master plan and archive the sub-plan.

**2026-06-23 archive sweep**

Superseded fake-skill, grounding, replan-lineage, seam-hardening, runtime-friction,
and TFM fake-skill validation sub-plans were moved to
`docs/artifacts/plan_archive/2026-06-23/`. No files were deleted.

## 9. Immediate Next Session Checklist

1. Rebuild the response-first container and run the runtime-review main
   questionnaire plus the architecture sweep.
2. Run `run_active_questionnaire.py --case-set environment` and archive the
   JSON artifact with the runtime review notes.
3. Add location-group probes: “what is in the kitchen?”, “bring every object
   from the kitchen to ALEX”, and “bring every object from the table to ALEX”.
4. Verify post-skill KB effects: query before and after pick/place/bring and
   confirm old support facts are removed and new hold/location facts appear.
5. Verify live stack endpoints remain healthy (`scan`, `report_result`, `say`,
   strict `head_motion`, fake-skill endpoints including fake `perform_motion`).
6. Validate no duplicate user-facing speech in KB visibility and
   execution-failure flows.
7. Continue AB-F2 decomposition metadata pass with tests.
