# kb_skills

`kb_skills` is the dedicated local boundary for KnowledgeCore interactions.

Phase 1 scope:

- formalize the read-only `/kb/query` capability behind a reusable client
- surface KB query capability through package-level skill metadata
- keep `chatbot_llm` responsible for deciding when to query and how to inject
  the result into prompts

Planned Phase 2 scope:

- add explicit wrappers for KnowledgeCore write/update operations such as
  revise, add, and remove
- provide one policy-aware seam for planner-driven or tool-driven KB mutations
- keep low-level KnowledgeCore transport out of both `chatbot_llm` and
  `nao_orchestrator`
