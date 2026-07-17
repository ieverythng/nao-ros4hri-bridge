# Documentation Index

This top-level docs surface is intentionally small and operational.

## Active top-level docs

- `contracts.md` — runtime payload contracts.
- `current_workflow.md` — canonical ownership and runtime flow.
- `launch_profiles.md` — launch matrix, arguments, operator runbooks.
- `plans/nao_ros4hri_masterplan.md` — planner execution status, known gaps,
  and next checks.

## Architecture

- `architecture/ab_registry_input.json` — current AB registry architecture snapshot.
- `architecture/demo_stack_seam_contract_2026-05-26.md` (+ HTML) — demo runtime seam ownership, message flow, and operator checks.
- `architecture/fake_skills_scenarios_playbook.md` (+ HTML) — scenario runbook for fake-skills simulation and validation.
- `architecture/ros4hri_neural_workbench_interactive_architecture.html` — interactive architecture view for supervisor review.
- `architecture/skill_registry_contract.md` — skill registry contract and AB routing baseline.

## Plans

- `plans/nao_ros4hri_masterplan.md` (+ HTML) — canonical integration tracker.
- `plans/ISSUE_TRACKER_FULL_SUITE.html` — detailed runtime-readiness tracker.
- `plans/ISSUE_TRACKER_FAKE_SUITE.html` — deep fake-skill, preloaded
  environment, and replan validation tracker.
- `plans/FINAL_RUNTIME_REVIEW.html` — supervisor-facing runtime review.
- `plans/IRR_Implementation.html` — intent-route-response ablation tracker.
- `plans/LLM_MODEL_ABLATIONS.md` (+ HTML) — model-backend ablation ledger for
  Watson/Qwen, lab Qwen3-VL, mixed-role runs, and launch-wiring lessons.
- `plans/AGENT_HARNESS_RESEARCH_2026-07-11.md` (+ HTML) — research plan for a
  Watson/Hermes-style applied harness around ROS4HRI and future agent systems.
- `plans/runtime_seam_deslop_2026-07-14.md` (+ HTML) — active, evidence-gated
  implementation plan for fallback visibility, chatbot/planner authority,
  report-result ownership, strict KB effects, and controlled deslop.
- `plans/CRITIC_RUNTIME_HARDENING_2026-06-30.md` (+ HTML) — current
  source-hardening and live-proof handoff for the CRITIC pass.
- `plans/deep_fake_replan_stabilization_2026-07-01.md` (+ HTML) — current
  deep fake/replan stabilization handoff for preloaded fixtures, SVG maps, and
  dialogue-scoped planner lineage.
- `plans/CHATBOT_LLM_REFACTOR_REPORT.md` (+ HTML) — separate chatbot
  modularity/refactor plan. Treat it as a sibling plan until its changes are
  validated and folded into the masterplan.
- `plans/locate-anything-3b-impl-report.html` and
  `plans/locateanything-nao-integration (1).md` — LocateAnything migration references.
- `plans/semi_symbolic_requirements_handoff_2026-06-11.md` (+ HTML) — semi-symbolic KB requirements.
- `plans/wsl2-native-setup-plan.md` (+ HTML) — native WSL2 setup plan.

## Universal Agentic Harness

- `agentic_harness/README.md` — project index and current extraction decision.
- `agentic_harness/universal_agentic_harness_foundation.md` (+ HTML) —
  AB-aware harness architecture, implementation survey, current-stack
  extraction map, hypothesis registry, serving strategy, evals, and phased
  implementation plan.
- `agentic_harness/neural_workbench_adaptive_ab_harness.md` (+ HTML) —
  frame-relative AB control, adaptive pulse graphs, task interaction skills,
  trace-derived capability profiles, and staged Neural Workbench
  crystallization.

## Reference/Generated Material

- `evaluation/` — hash-addressed thesis runtime evidence, normalized per-case
  metrics, and provenance manifests. Historical and diagnostic bundles remain
  separate from the pending final frozen run.
- `artifacts/` — archived handoffs, historical plans, and research notes.
- `artifacts/runtime_seam_deslop_review_2026-07-14.md` — qualified runtime
  review for the final deslop batch, accepted main seams, image provenance, and
  the endpoint-blocked deep-fake gate.
- `artifacts/runtime_review_2026-07-10_sv_seams.md` — latest evidence-backed
  runtime review for KB mutation, grouped delivery, fake-deep recovery, and
  report-result seams.
- `artifacts/runtime_review_2026-07-11_qwen_personal_stack.md` — Qwen personal
  stack model-ablation runtime review, including full main and deep fake-suite
  comparison.
- `artifacts/runtime_multiperson_grounding_audit_2026-07-16.md` — v21
  authoritative tracker filter audit for detector churn, stale people in
  KnowledgeCore, and repeated current-scene queries.
- `knowledge/` — GitNexus knowledge layer docs and generated wiki snapshots.

## Retention rule

If content is tied to a specific date, sweep, or temporary handoff, place it in
`docs/artifacts/` and keep durable conclusions in active docs above.
