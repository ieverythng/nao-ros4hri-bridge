# Skill Registry Contract (Canonical + Projections)

## Canonical Source
- Canonical registry: `src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json`
- This file defines AB=0 interfaces/primitives, AB=1 runtime skills, and higher-level AB proposals.

## Projection Views
- Planner/common projection: `src/Neural-Wokbench/src/skill_common/skill_common/defaults/skill_registry.yaml`
- Planner fallback projection: `src/planner_llm/config/skill_registry.json`
- Docs mirrors:
  - `docs/architecture/ab_registry_input.json`
  - `src/Neural-Wokbench/docs/neural_workbench/data/ab_registry_input.json`

## Decomposition Rule
- Use `metadata.decomposition.max_depth = -1` for staged full decomposition.
- For AB=1 runtime skills, `decomposes_to` should point to AB=0 leaves (actions/interfaces/primitives).
- AB=2 proposal objects remain non-runtime-callable unless explicitly promoted.

## Runtime Callability
- `metadata.runtime_callable` gates what can appear in planner runtime projections.
- AB=0 and proposal objects must keep `runtime_callable=false`.

## Enforcement
- Sync utility: `python3 scripts/sync_skill_registry_views.py --write`
- Consistency checks:
  - `python3 scripts/sync_skill_registry_views.py --check`
  - `python3 scripts/check_skill_registry_consistency.py`
- Hooks wired in `.pre-commit-config.yaml`.
