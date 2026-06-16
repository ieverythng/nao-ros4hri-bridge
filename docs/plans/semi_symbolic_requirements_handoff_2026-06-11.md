# Semi-Symbolic Requirements Implementation Handoff

**Date:** 2026-06-11  
**Branch:** `refactor/deslop_repo`  
**Status:** Consolidated with the latest `origin/refactor/deslop_repo` changes;
implementation is staged but uncommitted.
**Scope:** SV-driven KB mutation, node-owned refresh, spatial grounding, human-inclusive perception, validation resources, spoken execution guidance, and prompt guards.

---

## Executive Summary

This pass implemented the highest-impact portions of the 04/06/26 and 10/06/26
notes across the root repository and the nested `chatbot_llm` and
`Neural-Wokbench` repositories.

The central result is a clearer semi-symbolic path:

```text
chatbot_llm explicit request
  -> planner request with compact grounded_context
  -> planner_llm registry-grounded plan
  -> nao_orchestrator validation and deterministic dispatch
  -> kb_skills / AB=1 skill / nao_scene_grounding
  -> execution feedback and grounded dialogue
```

Direct KB mutations are now exposed as guarded planner skills. Scene reporting
separates humans from objects. Optional frame-qualified spatial evidence can flow
from a simulator or 3D source through `nao_scene_grounding`, KnowledgeCore, and
compact LLM grounding. The fake-skill validation probe now includes near/far
spatial cases and generates JSON, CSV, and HTML metrics artifacts.

Static and unit validation is green. Live ROS/container validation remains
pending because Colima could not start: its disk was reported as already in use.

---

## 14/06/26 Integration Consolidation

The branch was fast-forwarded through the latest incoming commits and the local
semi-symbolic work was reconciled file by file. The combined implementation
preserves both sides of the new seams:

- chatbot-owned execution-result wording through `report_result`;
- deterministic KB mutation dispatch through `kb_skills`;
- fake `perform_motion` coverage for validation;
- human/object separation and metric spatial evidence;
- YAML-owned planner prompt wording without a duplicate Python prompt default.

The deslop pass also centralized optional numeric evidence normalization in
`planner_common`, reducing repeated coercion in orchestrator scan normalization.
This increases locality without moving execution or grounding ownership.

Current integration gates:

| Gate | Result |
| --- | --- |
| Planner contracts, prompt pack, registry tests | 32 passed |
| Fake-skill, KB mutation, metrics tests | 21 passed |
| Nested `chatbot_llm` intent, grounded-context, prompt tests | 11 passed |
| Registry consistency | passed |
| ROS4HRI change audit | passed |
| Python compilation and diff checks | passed |
| ROS-dependent orchestrator/grounding tests | pending ROS environment (`chatbot_msgs`, `rclpy`) |
| Live KnowledgeCore, spatial overlay, speech/result validation | pending main-PC ROS/container run |

The root and nested repositories remain uncommitted. `.codex/config.toml` and
`src/Neural-Wokbench/docs/.DS_Store` remain intentionally excluded from staged
integration changes.

---

## 15/06/26 Runtime/Fake-Skill Consolidation

The branch was fast-forwarded through commit `b33c100` and reconciled against
the staged semi-symbolic implementation. Incoming runtime seams were treated as
authoritative where they overlapped:

- `look_at` now has a real fake-skill adapter, scenario, orchestrator execution
  mode, launch-profile wiring, and questionnaire coverage;
- execution-result wording uses the callback-safe chatbot interaction seam and
  preserves bounded motion-result evidence;
- the active questionnaire supports full ROS4HRI speech injection and
  chatbot-service injection, including a controlled KnowledgeCore probe;
- the expanded planner prompt pack remains canonical.

The consolidation retained the additive local seams:

- explicit KB mutation dispatch through `kb_skills`;
- near/far spatial fake scenarios and metric-evidence guards;
- human/object separation in general visibility reporting;
- shared numeric evidence normalization and validation metrics tooling.

Available static gates pass. Live runtime review is pending because Docker/Colima
is not running on this machine. The referenced `ISSUES BEFORE 17-06-26` HTML
could not be found in the current branch, its four incoming commits, or the
nearby fetched remote branches; it still needs to be supplied or pulled from its
actual source before it can inform the final review.

---

## Status Against the Original Notes

The sections below preserve the original requests as grouped checklist items.
Where one request produced several implementation tasks, those tasks remain under
the same request so the group can be removed from the working list only when its
stated remaining work is complete.

| Original request group | Can it come off the list? | Remaining condition |
| --- | --- | --- |
| Allow direct KB predicate changes through chat: add, revise, and remove facts; keep ordinary questions non-mutating; preserve deterministic node ownership. | **Keep open for live proof.** | Complete a live KnowledgeCore mutation/query run. |
| Formalize which node refreshes each part of the KB and world evidence, including the distinction between T0 planning evidence and execution-time proof. | **Implementation can come off.** | Supervisor review may create separate follow-up work for additional real AB=1 skills. |
| Expose trustworthy TF/location data for simulated objects so spatial and proximity questions can be answered without inferring metric distance from image-plane coordinates. | **Keep open.** | Connect a trustworthy pose producer, choose the canonical frame, and run live proximity evaluation. |
| Make “What can you see?” report humans as well as objects, without treating people as objects. | **Keep open for live proof.** | Perform one live mixed-scene spoken check. |
| Create the resources needed to write and validate the TFM: validation tooling, scenario plan, result artifacts, useful fake scenarios, and documented architecture/contracts/ownership. | **Keep open for the formal run.** | Run the updated ROS probe, retain the new artifacts, and incorporate results into the thesis. |
| Improve spoken acknowledgement, progress, and completion so useful execution evidence is communicated without premature success or duplicate speech. | **Keep open for wording evaluation.** | Evaluate live traces. |
| Add explicit prompt guards across LLM-facing seams to preserve ownership, require evidence, keep people/objects distinct, and prevent fabricated actions or success. | **Implementation can come off.** | Continue trace monitoring as an ongoing validation practice. |

### 04/06/26: Direct KB Predicate Changes Through Chat

**Status: Implemented and unit-verified; live KnowledgeCore validation pending.**

Completed:

- Added planner-visible `kb_add`, `kb_revise`, and `kb_remove` AB=1 skills.
- Added aliases including `kb_write`, `kb_update`, and `kb_delete`.
- Added explicit chatbot routing for phrases such as “remember that”, “revise the
  fact”, and “forget that”.
- Ordinary questions are guarded against accidental mutation.
- `nao_orchestrator` validates that mutation steps contain concrete statements.
- Orchestrator delegates mutations through `KnowledgeCoreMutationClient` and the
  canonical `/kb/revise` service rather than bypassing `kb_skills`.
- Added mutation diagnostics and client lifecycle cleanup.
- Synchronized canonical AB registry views into planner and architecture outputs.

Primary files:

- `src/nao_orchestrator/nao_orchestrator/orchestrator.py`
- `src/nao_orchestrator/nao_orchestrator/intent_rules.py`
- `src/kb_skills/kb_skills/mutation_client.py`
- `src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json`
- `src/chatbot_llm/chatbot_llm/intent_rules.py`

Still required:

- Run live requests against KnowledgeCore:
  - “Remember that cup_1 is on table_1.”
  - “Revise the fact so cup_1 is on table_2.”
  - “Forget that cup_1 is on table_2.”
  - “Is cup_1 on table_1?” must produce no mutation.

### 04/06/26: Formalize Per-Node KB Refresh Ownership

**Status: Formalized and partially implemented.**

Completed:

- Added `docs/architecture/kb_refresh_ownership_contract.md`.
- Preserved ownership boundaries:
  - `nao_scene_grounding` owns detector intake, transient object facts, spatial
    overlay merging, and `/scene/summary`.
  - AB=1 skills own live execution evidence.
  - `kb_skills` owns KnowledgeCore transport.
  - LLM nodes do not autonomously refresh or directly write KnowledgeCore.
  - `nao_orchestrator` remains deterministic dispatch/feedback owner.
- Explicitly documented T0 as planning evidence rather than execution-time proof.

Still required:

- Review the ownership contract with the supervisor and decide whether any
  additional real AB=1 skills should refresh their own KB effects.

### 04/06/26: Integrate TF/Location Data for Sim Objects

**Status: Architecture-ready and statically verified; real simulator source pending.**

Completed:

- Added optional `spatial_overlay_topic` to `nao_scene_grounding`.
- Overlay contract accepts object entries keyed by grounded `entity_id`:

```json
{
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "frame_id": "base_link",
      "position": {"x": 1.0, "y": 0.2, "z": 0.7}
    }
  ]
}
```

- `nao_scene_grounding` merges the pose, derives `distance_m`, refreshes KB
  spatial predicates, and publishes enriched `/scene/summary`.
- Compact `grounded_context.entities[]` preserves only frame-qualified metric
  positions. Image-plane `center_x`/`center_y` remain excluded by default.
- Added launch argument `scene_grounding_spatial_overlay_topic`.
- Added fake near/far spatial scenarios.

Important limitation:

- The existing simulator currently publishes the robot/camera TF chain but no
  object frames or trustworthy 3D object poses. A producer still needs to publish
  the spatial overlay.

### 04/06/26: Proximity Questions

**Status: Contract and guard implemented; live data source and evaluation pending.**

Completed:

- Chatbot and planner prompts only permit metric/proximity answers when
  `frame_id` plus `position` or `distance_m` exists.
- Explicitly forbid distance inference from image-plane centers.
- Added `object_near_robot` and `object_far_from_robot` fake scenarios.
- Added this as an explicit supervisor decision in the ownership and validation
  documents.

Point for next meeting:

- Select the canonical frame and source for simulator and real object poses.
- Decide whether `base_link`, `map`, or another world frame is authoritative for
  user-facing proximity comparisons.

### 10/06/26: “What Can You See?” Must Include Humans

**Status: Implemented and unit-verified.**

Completed:

- General scene scan summaries now compose people and object evidence.
- People remain under `people`; they are not treated as objects.
- Chatbot and planner prompt packs explicitly require both visible people and
  visible objects for general visibility questions.
- Added mixed human/object scene-report tests.

### 10/06/26: TFM Writing Resources

**Status: Confirmed and expanded.**

Existing resources confirmed:

- `docs/thesis/tfm_architecture_and_implementation_reference_2026-06-03.*`
- `docs/thesis/tfm_runtime_contracts_and_semantics_2026-06-03.*`
- `docs/thesis/tfm_validation_protocol_2026-06-03.*`

Added:

- `docs/thesis/tfm_validation_execution_plan_2026-06-11.md`
- This handoff pair.
- A sample metrics dashboard generated from the existing 2026-06-03 trace.

### 10/06/26: Validation Script, Scenarios, Dashboard, Representation

**Status: Implemented statically; new formal live run pending.**

Completed:

- Added `scripts/summarize_validation_traces.py`.
- Outputs:
  - JSON for machine-readable results;
  - CSV for thesis tables/statistics;
  - HTML dashboard for rapid comparison;
  - JSONL remains the authoritative raw trace.
- Updated `scripts/run_live_fake_skill_scenario_probe.py` to invoke the metrics
  exporter automatically.
- Corrected the probe to remove legacy contract fields:
  - `goal_token`;
  - planner `ack_text` / `ack_mode`;
  - `world_model_snapshot` / `world_model_text`;
  - transport-only `interaction_mode`.
- Expanded container synchronization/build scope so the probe tests current
  planner-common, grounding, KB, chatbot, planner, orchestrator, and fake-skill
  changes.
- Added near/far spatial fake scenarios.
- Added a concrete experiment plan mapping scenarios to metrics and gates.

Sample artifacts:

- `docs/artifacts/fake_skill_validation_metrics_20260603_sample.json`
- `docs/artifacts/fake_skill_validation_metrics_20260603_sample.csv`
- `docs/artifacts/fake_skill_validation_metrics_20260603_sample.html`

Still required:

- Run the updated probe on the main PC/container and retain the new generated
  trace/report/dashboard artifacts.
- Decide whether the scenario matrix needs another AB=1 fake skill after results
  reveal a specific coverage gap. No speculative extra skill was added.

### 10/06/26: Better Spoken Acknowledgement and Progress Guidance

**Status: Prompt guidance implemented; live wording evaluation pending.**

Completed:

- Initial acknowledgement must describe the accepted goal without claiming
  success.
- Progress/completion wording must use returned execution evidence.
- Planner may set `communication_policy.emit_progress=true` when a successful
  step unlocks a later step.
- Guidance permits grounded target identifiers in progress, for example:
  “I found person_1. I am navigating to them.”
- Planner still cannot insert arbitrary `say` steps into executable plans.

Still required:

- Evaluate wording quality and duplicate-speech behavior in live traces.

### 10/06/26: Explicit Prompt Guards

**Status: Implemented and unit-verified.**

Added guards covering:

- role and ownership;
- compact grounded context;
- people/object separation;
- perception evidence;
- skill registry constraints;
- intent and planner boundaries;
- KB mutation authorization;
- metric spatial evidence;
- acknowledgement/progress/completion semantics;
- no fabricated execution or perception claims.

---

## Validation Evidence

Latest passing static/unit gates:

| Scope | Result |
| --- | --- |
| `planner_common` contracts | 25 passed |
| `planner_llm` | 52 passed, 1 skipped |
| `nao_orchestrator` | 57 passed |
| targeted `chatbot_llm` seams | 58 passed |
| KB, fake skills, metrics | 26 passed, 2 skipped |
| canonical skill registry consistency | passed |
| ROS4HRI change audit | passed |
| Python compilation | passed |
| `git diff --check` across root/nested repos | passed |

Unavailable gates:

- Full `pre-commit`: dependency is not installed in the current environment.
- Live ROS launch/node validation: `rclpy` and launch dependencies are unavailable
  in the active host environment.
- Docker/Colima validation: Colima failed to start because its disk was reported
  as already in use by the `colima` instance.

---

## Repository and Branch Locations

| Repository | Branch | Relevant state |
| --- | --- | --- |
| root `nao-ros4hri-bridge` | `refactor/deslop_repo` | all root implementation/docs/tests are uncommitted |
| nested `src/chatbot_llm` | `refactor/upstream_packages` | chatbot routing, prompt, spatial handoff, and tests are uncommitted |
| nested `src/Neural-Wokbench` | `feat/base-implementation` | canonical AB registry and generated view changes are uncommitted |

Do not accidentally include:

- root `.codex/config.toml`: unrelated pre-existing user change;
- nested `src/Neural-Wokbench/docs/.DS_Store`: unrelated filesystem change.

---

## Recommended Next Actions

1. Recover/start Colima or use the main-PC ROS container.
2. Run:

```bash
python3 scripts/run_live_fake_skill_scenario_probe.py
```

3. Validate explicit KB add/revise/remove and an ordinary KB question.
4. Publish a test spatial overlay and verify:
   - enriched `/scene/summary`;
   - KB spatial predicates;
   - compact grounded context;
   - correct proximity answer.
5. Ask “what can you see?” with both a person and an object visible.
6. Inspect acknowledgement, progress, completion, and duplicate-speech behavior.
7. Run full precommit/colcon tests in the ROS environment.
8. Review the canonical spatial frame/source decision with the supervisor.
9. After validation, stage/commit each repository separately using the desired
   commit convention.

---

## Pause State

No files were staged, committed, or pushed during this pass. The implementation
is paused with static gates green and live validation explicitly outstanding.
