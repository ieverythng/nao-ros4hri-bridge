# Deslop Refactor Tracking — `nao-ros4hri-bridge`

**Branch:** `refactor/deslop_repo`
**Date:** 2026-06-03
**Scope:** Last ~5 commits across `planner_common`, `planner_llm`, and `nao_orchestrator`

---

## Summary

This pass targeted code slop introduced during recent development: duplicated helpers, oversized modules, and cross-package dependency violations. All changes are behavior-preserving — no contracts, interfaces, or runtime semantics were altered.

**Net effect:** ~425 lines removed, 3 new public helpers added to `planner_common`, 1 new module created.

---

## Change 1: Extract grounded context projection helpers

**Principle:** *Separation of Concerns* — large contract file contained 30+ private helpers for a single sub-domain.

### What changed

| File | Before | After |
|------|--------|-------|
| `planner_common/contracts.py` | 1276 lines | ~883 lines |
| `planner_common/grounded_context_projection.py` | (new) | 526 lines |

### Details

- Extracted 30+ private helpers (`_normalize_grounded_entity`, `_normalize_relations`, etc.) from `contracts.py` into a dedicated module `grounded_context_projection.py`.
- The two public entry points (`project_llm_grounded_context`, `grounded_context_to_context_ref`) now live as thin lazy re-imports in `contracts.py` for backward compatibility.
- `normalize_grounded_context` compact-path branch lazily imports from the new module.

### Risk

Low — all helper signatures and call sites are unchanged. Tests pass (128 passed, 1 skipped).

---

## Change 2: Deduplicate `coerce_optional_float`

**Principle:** *DRY* — same helper defined in three packages.

### What changed

| File | Change |
|------|--------|
| `planner_common/contracts.py` | Added `coerce_optional_float` as public helper |
| `nao_orchestrator/intent_rules.py` | Removed local definition, imports from `planner_common.contracts` |
| `nao_orchestrator/scan_skill_server.py` | Removed local definition, imports from `planner_common.contracts` |

### Details

- The helper was defined identically in `intent_rules.py` and `scan_skill_server.py`. Consolidated into `planner_common.contracts` as the single source of truth.
- Exported through `planner_common/__init__.py`.

### Risk

Low — function signatures are identical across all three locations. No behavioral change.

---

## Change 3: Move plan validation helpers to `planner_common`

**Principle:** *Separation of Concerns* + *Repo Boundaries* — planner_llm must not depend on nao_orchestrator.

### What changed

| File | Change |
|------|--------|
| `planner_common/contracts.py` | Added 3 public helpers (renamed from private) |
| `planner_llm/planner_engine.py` | Removed local definitions, imports from `planner_common.contracts` |

### Moved helpers

| Old name (in planner_engine) | New name (in contracts) |
|-------------------------------|------------------------|
| `_scan_report_summary_error` | `scan_report_summary_error` |
| `_missing_requested_report_error` | `missing_requested_report_error` |
| `_request_requests_report` | `request_requests_report` |

### Details

- These helpers validate plan contracts and belong in `planner_common`, not in the planner runtime node.
- Placed in `planner_common` (not `nao_orchestrator`) because `planner_llm` must not depend on robot-specific packages per repo boundary guardrails.
- Underscore prefix removed since these are now public contract helpers.

### Risk

Low — call sites updated to import from new location. All tests pass.

---

## Change 4: Fix `__init__.py` exports

**Principle:** *Least Surprise* — public API surface must reflect available symbols.

### What changed

| File | Change |
|------|--------|
| `planner_common/__init__.py` | Added 4 new exports to both imports and `__all__` |

### New exports

- `coerce_optional_float`
- `missing_requested_report_error`
- `request_requests_report`
- `scan_report_summary_error`

---

## Intentionally Skipped

| Item | Reason |
|------|--------|
| `_first_non_empty_value` deduplication | Different signatures across `orchestrator.py` and `chatbot_llm/knowledge_snapshot.py`; different ownership domains |
| `__all__` sort order in `__init__.py` | Pre-existing issue, outside scope of this pass |
| Nested repo changes (`chatbot_llm`, `dialogue_manager`) | Guardrails require seam-focused changes only for fork-tracked packages |

---

## Validation

- **py_compile:** All 6 touched files compile cleanly
- **Tests:** 128 passed, 1 skipped (pre-existing `rclpy` import error in `test_kb_spatial_statements.py`)
- **ROS4HRI audit:** Changes confined to first-party packages (`planner_common`, `planner_llm`, `nao_orchestrator`); no boundary violations
- **Dependency direction:** `planner_llm` and `nao_orchestrator` import from `planner_common` only — no reverse dependencies introduced

---

## Commit Sequence

1. `refactor(planner_common): extract grounded context projection helpers` — new module + contracts.py surgery
2. `refactor(planner_common): deduplicate coerce_optional_float` — consolidate across packages
3. `refactor(planner_llm): move plan validation helpers to planner_common` — fix cross-package dependency
4. `docs: add deslop refactor tracking document` — this HTML file
