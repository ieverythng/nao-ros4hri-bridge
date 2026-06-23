# Plans and Execution Tracks

This folder is intentionally minimal.

## Active plans

- `nao_ros4hri_masterplan.md` (+ `.html`)  
  Canonical integration tracker (planner/orchestrator, AB registry integration, observability/dashboard stream, upstream reconciliation).
- `fake_skills_codex_handoff.md` (+ `.html`)  
  Separate active stream for fake-skills runtime policy/scenario operations and Workbench validation support.
- `planner_grounding_moe_contract_2026-06-01.md` (+ `.html`)  
  Grounding ownership contract and MoE-by-AB planner seam policy.
- `planner_replan_lineage_adr.md` (+ `.html`)  
  Accepted ADR for canonical `plan` lineage fields, Hybrid Minimal T0, and replan join policy.
- `sv_e2e_planner_flow_walkthrough_2026-06-01.md` (+ `.html`)  
  Supervisor walkthrough deck for end-to-end user -> planner -> orchestrator -> dialogue flow.

## Archival rule

When a plan becomes fully implemented or is superseded, move it to
`docs/artifacts/plan_archive/` and fold the status into the master plan.

## Authoring rule

For any active plan intended for review:

1. Keep Markdown as source of truth.
2. Keep an HTML companion in the same folder.
3. Update both in the same commit.
