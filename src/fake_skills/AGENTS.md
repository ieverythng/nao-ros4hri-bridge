# fake_skills Agent Notes

- Fake skills must simulate AB=1 behavior while exposing clear AB=0 decomposition intent in registry metadata.
- Keep deterministic, scenario-driven outcomes with explicit result/failure payloads.
- Preserve semantic separation: `walk_to` (local locomotion segment) vs `navigate_to` (destination navigation).
- Keep `find_object` semantics tied to scene-evidence freshness policy.
- Any fake skill contract change must be mirrored in canonical AB registry and planner-facing projections.
