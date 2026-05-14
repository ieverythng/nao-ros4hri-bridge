# Skill-Centric ROS4HRI Implementation Overlay (2026-05-13)

This overlay is additive to:

- `/home/juanbeck/Downloads/codex_skill_centric_ros4hri_handoff_2026-05-12.md`
- `/home/juanbeck/Downloads/codex_skill_centric_ros4hri_handoff_2026-05-12 (1).html`

It does not supersede those documents. It introduces interleaved implementation guardrails and periodic live-container validation loops.

## Section Anchors (source-of-truth references)

- Architecture ownership and skill abstraction: **§3**, **§4.2**
- Shared skill registry and contract: **§5 Phase 1**, **§5 Phase 3**
- Scan extraction: **§5 Phase 4**, **§7 PR 4**
- Validation harness: **§5 Phase 6**, **§8**
- Warnings on bypassing orchestrator and low-level decomposition: **§10**

## Interleaved Execution Gates

For each implementation phase, apply gates in this order before moving to the next phase:

1. `deslop-refactor` gate
- run deslop pass on touched files only
- collapse duplicated control flow, keep behavior
- keep abstractions minimal and purposeful

2. `iiia-ros4hri-check` gate
- run `python3 scripts/ros4hri_change_audit.py --mode working`
- verify ownership: planner selects skills, skill server owns internals, orchestrator dispatches
- validate interface choices (action for composite skills)

3. Live-container runtime gate
- rebuild changed packages in live container
- verify lifecycle state and action availability
- run one phase-specific smoke command and capture result in this plan log

## Periodic Live-Container Check Cadence

Run every **20 minutes** while implementing:

- `ros2 node list | grep -E 'nao_orchestrator|scan_skill_server|planner_llm|chatbot_llm|dialogue_manager'`
- `ros2 action info /skill/scan`
- `ros2 lifecycle get /scan_skill_server`
- one scan goal smoke:
  - `ros2 action send_goal /skill/scan nao_skills/action/ScanScene '{target: people, target_kind: people, max_sweeps: 1, evidence_policy: grounded_current_observation, result_mode: success}' --feedback`

If any command fails, stop phase progression and resolve first.

## Phase Status (with interleaved seams)

### Phase A — Direct scan action-server switch

Scope:
- remove orchestrator internal scan choreography/evidence logic
- use `/skill/scan` action as the only scan execution path
- keep scan internals in `scan_skill_server`

Seam status:
- `deslop-refactor`: applied on orchestrator scan path cleanup
- `iiia-ros4hri-check`: package scope validated (`nao_skills`, `nao_orchestrator`, `nao_chatbot`)
- live-container check: passed after callback fix (see validation log)

### Phase B — Shared skill normalization priority

Immediate next scope:
- introduce `skill_common` package with shared registry and result contracts
- switch planner/chatbot/orchestrator skill-name normalization to shared source
- preserve current runtime behavior while replacing duplicate registry seams

Required gates:
- deslop + iiia checks before merge
- live-container smoke with planner->orchestrator->scan flow

### Phase C — Validation harness expansion

Scope:
- controlled action failure injection and replan validation
- scenario tracking for people/objects ambiguity and not-found flows

Required gates:
- periodic checks retained
- update overlay log after each scenario batch

## Validation Log (live container)

Date: **2026-05-13**
Container: `nao_ros2` (`022e10b190a1`)

1. Build check:
- `colcon build --packages-select nao_skills nao_orchestrator nao_chatbot --symlink-install`
- result: success

2. Scan failure root cause found:
- error: `scan_skill_server.action_server: Error raised in execute callback: no running event loop`
- effect: action aborted after first/partial sweep with empty result

3. Fix applied:
- removed `asyncio` dependency in scan execute callback
- switched scan sweep wait to synchronous `time.sleep`

4. Post-fix scan smoke:
- `/skill/scan` goal now reaches `SUCCEEDED`
- feedback includes sequential sweep progress (`0.366`, `0.633`, `0.899`)
- result payload contains grounded people evidence and summary

## Commit Discipline Notes

- keep Phase A/B/C as separate commits where feasible
- each commit message must mention:
  - whether deslop gate passed
  - whether iiia gate passed
  - whether live-container periodic check passed
