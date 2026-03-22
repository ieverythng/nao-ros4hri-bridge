"""Phase-2 stubs for KnowledgeCore write/update operations."""

from __future__ import annotations


class KnowledgeCoreMutationClient:
    """Reserved seam for future KnowledgeCore writes and revisions.

    Phase 1 intentionally keeps the local stack read-only for grounded dialogue.
    These methods exist so planner- or tool-driven KB mutations can later live
    behind a dedicated package boundary rather than being embedded in LLM code.
    """

    def add_fact(self, *args, **kwargs):
        raise NotImplementedError(
            "KnowledgeCore add_fact is not wired yet. Phase 2 should bind the "
            "upstream write endpoint behind kb_skills."
        )

    def revise_fact(self, *args, **kwargs):
        raise NotImplementedError(
            "KnowledgeCore revise_fact is not wired yet. Phase 2 should bind the "
            "upstream revise endpoint behind kb_skills."
        )

    def remove_fact(self, *args, **kwargs):
        raise NotImplementedError(
            "KnowledgeCore remove_fact is not wired yet. Phase 2 should bind the "
            "upstream delete/remove endpoint behind kb_skills."
        )
