# kb_skills

`kb_skills` is the dedicated local boundary for KnowledgeCore interactions.

Current scope:

- formalize the read-only `/kb/query` capability behind a reusable client
- formalize the write-side `/kb/revise` capability behind reusable add/update/remove helpers
- surface both read and write KB capabilities through package-level skill metadata
- keep `chatbot_llm` responsible for deciding when to query and how to inject
  the result into prompts

Current role in the grounded stack:

1. `nao_scene_grounding` writes transient detector-derived facts into
   `knowledge_core` through the shared mutation boundary
2. `chatbot_llm` reads those facts through `/kb/query`
3. `kb_skills` provides the reusable read-side query client and canonical KB
   intent labels used by the local stack
4. future planner-facing work can use the same package for `add`, `update`, and
   `remove` mutations instead of embedding KnowledgeCore transport logic in the
   planner or executor

## Query Surface

- `KnowledgeCoreQueryClient.query_rows(...)`
- package skill metadata: `kb_query`

## Mutation Surface

`KnowledgeCoreMutationClient` now exposes three explicit planner-facing helpers:

- `add_facts(...)`
- `revise_facts(...)`
- `remove_facts(...)`

Single-statement convenience wrappers also exist:

- `add_fact(...)`
- `revise_fact(...)`
- `remove_fact(...)`

All of them route through the same `/kb/revise` transport so mutation policy,
timeouts, tracing, and error handling stay in one place.

## Mutation Policy

Recommended ownership split:

- `nao_scene_grounding` remains the semantic owner of detector-derived transient
  facts such as `myself sees detected_pear_320_240`
- planner- or tool-driven KB updates should go through `kb_skills` rather than
  talking to `knowledge_core` directly
- `nao_orchestrator` should not become a raw KnowledgeCore client; it should
  stay downstream-only and consume structured planner decisions

Recommended method usage:

- `add`: persistent or additive world assertions
- `update`: refresh or revise transient grounded state
- `remove`: retract invalidated or obsolete facts

Transient grounded writes should provide a lifespan so they expire naturally.
Longer-lived planner memory should omit the lifespan unless the fact is meant to
be temporary.
