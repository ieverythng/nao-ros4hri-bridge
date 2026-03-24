"""Phase-2 stubs for KnowledgeCore write/update operations."""

from __future__ import annotations


# ---------------------------------------------------------------------------
# Reserved write-side boundary
# ---------------------------------------------------------------------------

class KnowledgeCoreMutationClient:
    """Reserved seam for future KnowledgeCore writes and revisions.

    Phase 1 intentionally keeps the local stack read-only for grounded dialogue.
    These methods exist so planner- or tool-driven KB mutations can later live
    behind a dedicated package boundary rather than being embedded in LLM code.
    """

    _PHASE_2_TEMPLATE = (
        "KnowledgeCore {operation} is not wired yet. Phase 2 should bind the "
        "upstream {endpoint} endpoint behind kb_skills."
    )

    def add_fact(self, *args, **kwargs):
        self._raise_not_ready("add_fact", "write")

    def revise_fact(self, *args, **kwargs):
        self._raise_not_ready("revise_fact", "revise")

    def remove_fact(self, *args, **kwargs):
        self._raise_not_ready("remove_fact", "delete/remove")

    def _raise_not_ready(self, operation: str, endpoint: str) -> None:
        raise NotImplementedError(
            self._PHASE_2_TEMPLATE.format(
                operation=operation,
                endpoint=endpoint,
            )
        )
