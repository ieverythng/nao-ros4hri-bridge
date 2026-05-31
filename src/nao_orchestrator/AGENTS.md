# nao_orchestrator Agent Notes

- Keep this package as deterministic execution owner.
- Execution feedback payloads must remain schema-stable for planner and trace tooling.
- Reuse registered skill aliases/mappings from canonical registry views; avoid ad-hoc name maps.
- Long-running robot actions should stay action-based with explicit success/failure payloads.
- Preserve ROS interface ownership boundaries; avoid bypassing existing skill/action seams.
- Keep planner gate/admission logic goal-lineage based (`goal_id`, `plan_id`, `plan_version`), not token-based.
- Preserve replan join seam behavior: mid-join by stable `step_id` when mappable, otherwise front-join fallback.
- Keep orchestrator downstream-only: no planner wording policy branches and no chatbot-style fallback text generation.
