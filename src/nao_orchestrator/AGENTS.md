# nao_orchestrator Agent Notes

- Keep this package as deterministic execution owner.
- Execution feedback payloads must remain schema-stable for planner and trace tooling.
- Reuse registered skill aliases/mappings from canonical registry views; avoid ad-hoc name maps.
- Long-running robot actions should stay action-based with explicit success/failure payloads.
- Preserve ROS interface ownership boundaries; avoid bypassing existing skill/action seams.
