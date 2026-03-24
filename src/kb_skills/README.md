# kb_skills

`kb_skills` is the dedicated local boundary for KnowledgeCore interactions.

Phase 1 scope:

- formalize the read-only `/kb/query` capability behind a reusable client
- surface KB query capability through package-level skill metadata
- keep `chatbot_llm` responsible for deciding when to query and how to inject
  the result into prompts

Current role in the grounded stack:

1. `nao_scene_grounding` writes transient detector-derived facts into
   `knowledge_core`
2. `chatbot_llm` reads those facts through `/kb/query`
3. `kb_skills` provides the reusable read-side query client and canonical KB
   intent labels used by the local stack

Today this package stays read-only on purpose. Detector writes and other KB
mutations are still owned outside `kb_skills`, even though the package already
has placeholder mutation helpers for later planner-facing work.

Planned Phase 2 scope:

- add explicit wrappers for KnowledgeCore write/update operations such as
  revise, add, and remove
- provide one policy-aware seam for planner-driven or tool-driven KB mutations
- keep low-level KnowledgeCore transport out of both `chatbot_llm` and
  `nao_orchestrator`
