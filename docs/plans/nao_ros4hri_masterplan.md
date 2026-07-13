# NAO ROS4HRI Masterplan (Consolidated, Active)

**Date:** 2026-07-07 (JSON-only deep fake/replan strict scoring refresh)
**Branch context:** `refactor/deslop_repo` with nested `chatbot_llm`
`feat/planner_llm_hooks` and Neural-Wokbench integration seams
**Scope:** Single active execution plan for planner/chatbot/orchestrator seams,
grounded-context reliability, canonical registry alignment, fake-skill
operational hardening, LocateAnything migration, and validation reporting.

The manuscript, final evidence synthesis, and submission closure track lives in
`docs/plans/tfm_completion_masterplan_2026-07-13.md` (+ `.html`). This
integration masterplan remains authoritative for runtime implementation status.

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
- `docs/architecture/ab_registry_input.json`
- `docs/architecture/ros4hri_neural_workbench_interactive_architecture.html`
- `docs/architecture/demo_stack_seam_contract_2026-05-26.md` (+ `.html`)
- `docs/architecture/fake_skills_scenarios_playbook.md` (+ `.html`)
- `docs/plans/` (this master plan + active runtime/showing plans)
- `docs/plans/CRITIC_RUNTIME_HARDENING_2026-06-30.md` (+ `.html`) for the
  current fake-deep/replan hardening pass
- `docs/plans/CHATBOT_LLM_REFACTOR_REPORT.md` (+ `.html`) for the separate
  chatbot modularity/refactor track. This is a sibling track, not a replacement
  for the CRITIC runtime validation plan.

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
- **Done (2026-06-30 source gate)**: compact grounded context uses the
  `grounded_context_v3` shape described in `docs/contracts.md`: `entities`
  remains the subject inventory, while `locations` is a derived grouping view
  with role separation for support groups, navigation/place targets, and
  recipients.
- **Done (2026-06-30 source gate)**: user-facing object lists now filter
  ontology/meta classes, rooms, places, support surfaces, tables, and people
  from deliverable object members unless the user explicitly asks about those
  categories.
- **Done (2026-06-30 source gate)**: chatbot admission now blocks execution
  handoff when a request names a human recipient or target that is absent from
  current grounded context. The expected behavior is clarification, not a
  planner request against the wrong person.
- **Done (2026-06-23 source gate)**: planner has a late, bounded fallback for grounded “bring every object from location X to recipient Y” requests when model output remains invalid after one repair attempt.
- **Done (2026-07-02 source gate)**: successful skill result payloads can apply structured `evidence.kb_effects` through the orchestrator’s existing `/kb/revise` boundary and verify post-conditions through `/kb/query` when available. Remove effects must disappear, add/update effects must resolve, and failed verification fails the skill step so the planner can replan or fail truthfully. This starts with fake manipulation skills and is reusable by real skills that emit the same payload shape.
- **Done (2026-06-30 source gate)**: execution feedback now includes
  `plan_outcome_summary` so report-result wording, failure review, and replan
  analysis can distinguish completed, failed, and pending targets without
  parsing free text.
- **Done (2026-07-01 source gate)**: chatbot planner handoff turn ids now
  include the dialogue id as well as role and request count. This prevents
  independent questionnaire dialogues from reusing `__default__:1` and
  producing the same planner `goal_id` lineage.
- **Done**: structured `chatbot_turn_trace` visibility is available for dialogue vs planner-handoff attribution.
- **Done (2026-05-26)**: planner-mode routing now guards visibility-only scene questions toward `knowledge_query` unless the user explicitly requests a new scan/action.
- **In progress**: proactive wording + speech arbitration pass to avoid duplicate user-facing utterances when execution acknowledgements and planner dialogue completions occur in the same interaction.
- **In progress**: live rebuild proof for the new location-group, KB-effect,
  and dialogue-scoped goal-lineage seams. Source tests pass, but runtime score
  should not be raised until a fresh response-first run proves the updated
  container behavior.
- **Done (2026-07-07 source gate)**: JSON-only deep fake scoring now treats
  clarification as failure when a preloaded fixture already provides the
  required source location and recipient. The runner also records fallback
  pressure per case and no longer leaks older ROS events into later cases when
  logs start with `[INFO]`.

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
  replacing `entities`, so existing consumers remain compatible. The front-facing
  contract is `grounded_context_v3` in `docs/contracts.md`.
- **Done (source)**: location membership now keeps deliverable objects separate
  from supports and places. This directly addresses fake-deep regressions where
  “spatial thing localized” or a table could leak into the user-facing object
  set.
- **Done (source)**: named-recipient admission checks are now covered in
  `chatbot_llm` tests. If the user asks for BLAKE while only ALEX is grounded,
  the stack must clarify before planner handoff.
- **In progress**: location-aware execution validation for prompts such as
  “bring every object from the kitchen to ALEX”.
- **Done (2026-07-07 source gate)**: location matching now prefers the most
  specific grounded label or alias over generic support terms. In the
  work-table failure, `work_table` now beats generic `table` matches when
  several table-like locations are present. Repeated named people such as
  multiple ALEX fixtures are disambiguated by relation scope first and by stable
  fixture namespace only as a tie-breaker.
- **Done (2026-07-07 source gate)**: compact location groups now keep movable
  user objects that also carry KnowledgeCore spatial materialization classes
  when their RDF type still identifies a deliverable object. This prevents
  books, phones, or cups from disappearing from `locations.contains` merely
  because the KB also includes `cyc:SpatialThing-Localized`.
- **Done (2026-07-07 source gate)**: grouped delivery and ordered walk fallback
  metadata now exports concrete member object ids in `scene_targets`, not the
  source table or support location. This fixes the trace-level regression where
  chatbot-authored `report_result` could truthfully follow the prompt but speak
  about completing a table because planner metadata named the table as the
  completed target.
- **Done (2026-07-09 source gate)**: `planner_common` now provides a shared
  `report_outcome` contract for `report_result` wording. The orchestrator
  injects reportable objects, recipients, anchors, excluded targets, events, and
  failures into the chatbot execution-report turn, while chatbot post-processing
  only rejects unsafe text such as treating a person, room, or table as a
  delivered object.
- **Done (harness source)**: runtime-review now supports named preloaded
  KnowledgeCore environment fixtures through `--preload-environment` and the
  dedicated `environment` case set. The first fixture pack includes
  `lab_table`, `kitchen_delivery`, and `gold_apple_handoff`, plus a visual SVG
  companion for the kitchen-delivery scene.
- **Done (2026-07-01 source gate)**: preloaded-environment SVGs have been
  normalized for the rqt human-radar loader with positive centimeter canvases
  and no text labels. This keeps the visual aid separate from scoreable RDF
  facts and avoids overlapping operator-facing text.
- **In progress**: skill-aware KB post-effects. Fake skills already report
  `evidence.kb_effects`; orchestrator now applies successful effects through
  KnowledgeCore and verifies their post-conditions when the query seam is
  available. The 2 July grouped-delivery probe showed that this verification
  must be paired with correct planner target selection: a model plan can target a
  location group instead of its contained objects. `planner_llm` now preempts
  grounded "bring every object from X to Y" requests with member expansion before
  model planning. Real skills should adopt the same payload contract before
  direct KB mutation is enabled for them.
- **Done (2026-07-09 source gate)**: stale spatial cleanup now lives in
  `nao_orchestrator.kb_effects` and covers `oro:isAt`, `oro:isOn`, `oro:isIn`,
  `oro:contains`, and `oro:placeOf` style aliases. Vague `kb_add` and
  `kb_revise` prose is rejected before KnowledgeCore dispatch so the dialogue
  stack can clarify the subject, predicate, and object.
- **Pending**: LocateAnything migration into the grounding stack. Keep it as a
  perception/grounding provider, not as a planner or dialogue policy owner.
- **Pending**: location lifecycle semantics for movement/manipulation, including
  removal of stale support/location facts after pick, place, and bring.

**Exit criteria**

- A response-first runtime review proves grouped-location KB queries, location
  scoped delivery, and post-skill KB changes on the next turn, including a
  post-action KB query proving delivered objects no longer retain stale source
  support or room relations.
- `robot-runtime-performance-review --case-set environment` proves the same
  scene can be preloaded deterministically before speech or service turns.
- `robot-runtime-performance-review --case-set fake_deep` now proves the
  all-success ladder behaviorally and the full `fail_once_navigation` ladder
  after a clean rebuild with `grounded_context_digest_enabled=false`.
  `fail_once_pick` and `delivery_blocked` remain the next recovery profiles.
- The fresh rebuilt container imports the patched `chatbot_llm` source rather
  than the old build copy before the fake-deep score is raised.
- The fresh rebuilt container imports the 7 July `planner_common` and
  `planner_llm` source gates. Proof now exists for
  `grounded_context_digest_enabled=false`, grouped work-table delivery member
  expansion without clarification, compact kitchen id matching, and
  missing-recipient clarification before planner handoff.
- Fake and real skill payloads use the same `kb_effects` shape for successful
  state changes, and executor-side post-condition checks prove those changes
  before later dialogue treats them as current KB truth.
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

- this masterplan and `docs/plans/ISSUE_TRACKER_FULL_SUITE.html`
  (known weak spot: occasional execution over-routing)
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

## 6. Research Track: Universal AB-Aware Agentic Harness

The [Universal Agentic Harness foundation](../agentic_harness/universal_agentic_harness_foundation.md)
defines a NAO-agnostic harness that compiles task-specific interaction modules
from an AB capability graph. The current NAO stack is its first reference
subsystem because `chatbot_llm` and `planner_llm` already contain substantial,
duplicated harness mechanisms: provider transport, prompt-pack loading,
structured-output repair, bounded context and skill projection, fallbacks, and
trace stages.

The extraction rule is deliberately narrow:

- keep dialogue policy and speaking in their current owners;
- keep planning, retry, replan, cancellation, and supervision in `planner_llm`;
- keep deterministic admission, dispatch, and execution evidence in
  `nao_orchestrator` and AB1 skills;
- consolidate only provider-neutral harness mechanisms behind compatibility
  adapters;
- allow an initial implementation in this repository for parity testing, while
  prohibiting ROS/NAO imports and semantics in the core;
- target a pure-Python `ab_harness` package beside Neural Workbench
  `skill_common`, with the canonical registry referenced rather than copied.

### HARNESS-0: Contract and Evidence Baseline (P0) - Parent Proof Done

- Freeze `HarnessSpec`, `TaskSpec`, `InteractionModuleSpec`, `ModelProfile`, and
  `TraceEvent`.
- Retain current chatbot/planner tests as behavioral oracles.
- Capture same-model and same-task baselines before moving shared mechanisms.
- Follow the H0 release in the
  [adaptive Neural Workbench extension](../agentic_harness/neural_workbench_adaptive_ab_harness.md):
  represent chatbot/planner as role-bounded AB3 model-agent objects inside the
  NAO AB4 system, compile only their admitted AB views, gate output reachability,
  and preserve a complete trace.
- Parent-only `src/ab_harness` now proves registry projection, role/output
  reachability, AB0 inspection-only enforcement, execution-claim rejection, and
  JSONL trace reconstruction without modifying nested LLM packages or runtime
  wiring. Live cooperative node integration remains HARNESS-1.

### HARNESS-1: Cooperative Node Migration (P1)

- Introduce thin adapters without changing node ownership or prompt policy.
- Migrate one mechanism at a time: capability probe, structured output,
  prompt-pack mechanics, task projection, then trace events.
- Compare standalone and harness-backed paths for route safety, KB behavior,
  planner admission, report wording, retry/replan, and duplicate speech.
- Reject or roll back any slice that fails behavioral parity.

### HARNESS-2: Portability and Uplift Proof (P2)

- Run generic-all-tools versus AB-projected interaction ablations.
- Prove the same contracts with one non-NAO synthetic adapter before declaring
  the kernel universal.
- Promote the package boundary only when relocation changes imports and package
  metadata, not behavior or schemas.

### HARNESS-3: Adaptive Workbench and Interaction Skills (P2-P4)

- Add candidate/recovery graph search only after the H0 compatibility path is
  stable.
- Maintain task-relative `InteractionSkill` objects with permissions, effects,
  proof obligations, capability profiles, supporting traces, and
  counterexamples.
- Let agents propose interaction repairs, but require independent validators,
  tests, environment evidence, or human review before acceptance.
- Introduce trace priors, entropy proxies, and crystallization in separate
  ablations; never allow frequency-only runtime promotion.
- Require one non-NAO AB4 adapter before calling the kernel universal.

No runtime harness migration is complete at this checkpoint. The foundation is
the accepted research baseline; implementation remains gated by schema review,
parity tests, and the ROS4HRI ownership invariants above.

## 7. Backlog (Prioritized, Cross-Track)

1. **P0** Complete speech-ownership arbitration so each turn has one user-facing utterance authority (no duplicate execution-ack + planner-dialogue speech).
2. **P0** Live-prove preloaded environment fixtures, grouped-location delivery,
   post-skill KB effects, and recipient-missing clarification in the rebuilt
   response-first stack.
3. **P0** Resolve planner request-admission/backpressure seam after terminal
   dialogue acts and `waiting_user` transitions; ensure subsequent `new_goal`
   requests are deterministically handled with accepted, superseded, or rejected
   status plus explicit reason.
4. **P1** Continue AB decomposition schema expansion and tests (AB=2+ lineage coverage).
5. **P1** Begin LocateAnything migration through the grounding adapter layer.
6. **P1** Begin dashboard backend skeleton (`nao_dashboard`).
7. **P2** Stage upstream nested-repo merges per inventory artifact.

## Additional Runtime Evidence

- Live stack validation artifact:
  - `docs/artifacts/runtime_validation_report_2026-05-26_stack_live.md`

## 8. Mandatory Validation Gates (Per Change Slice)

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

## 9. Documentation Hygiene Rules (Going Forward)

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

## 10. Immediate Next Session Checklist

1. Rebuild the response-first container and run the runtime-review main
   questionnaire plus the architecture sweep.
2. Run `run_active_questionnaire.py --case-set environment` and archive the
   JSON artifact with the runtime review notes.
3. Add location-group probes: “what is in the kitchen?”, “bring every object
   from the kitchen to ALEX”, and “bring every object from the table to ALEX”.
4. Verify post-skill KB effects: query before and after pick/place/bring and
   confirm old support facts are removed and new hold/location facts appear.
5. Run `run_active_questionnaire.py --case-set fake_deep` with
   `all_success`, `fail_once_navigation`, `delivery_blocked`, and
   `recipient_missing` policies. Compare against
   `docs/plans/CRITIC_RUNTIME_HARDENING_2026-06-30.md`.
6. Verify live stack endpoints remain healthy (`scan`, `report_result`, `say`,
   strict `head_motion`, fake-skill endpoints including fake `perform_motion`).
7. Validate no duplicate user-facing speech in KB visibility and
   execution-failure flows.
8. Continue AB-F2 decomposition metadata pass with tests.
