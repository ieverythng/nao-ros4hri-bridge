# Context Projection Research

## Question

Would an MCP-like context server provide a safer single representation of the
robot scene than the current KnowledgeCore to `grounded_context` path?

## Findings

The current stack already has the relevant ownership split. `kb_skills` owns
KnowledgeCore transport, `chatbot_llm` creates the bounded
`grounded_context_v3` projection, and `planner_common` normalizes that
projection for planner ingress. The active response-first launch disables the
optional natural-language scene digest, so the chatbot receives the structured
grounded JSON as its live scene representation.

The live regression was caused by a different source of duplication. A broad
scene query received a fresh five-object grounded context, while the retained
dialogue window still contained an earlier lab answer naming `ATLAS`, `MIDAS`,
`TITAS`, and `VEGA`. The model combined those historical claims with the new
snapshot and said that all objects were on the table. The current snapshot
contained only one table relation for `apple_jbdym`; it did not support that
statement.

The structural correction is a scene boundary for non-mutating current-scene
queries. The live grounded snapshot remains the only scene context supplied to
that turn, and the returned history is reset to the current query and answer.
Reflective scene-change questions, such as “is that the same person as before?”,
retain dialogue history because their meaning depends on temporal comparison.

## MCP comparison

The official MCP architecture describes a host that manages isolated client
sessions and servers that expose specialized resources, tools, and prompts.
Resources are application-controlled context, while tools are model-controlled
actions. This separation is useful vocabulary for future interfaces, but
adopting the protocol would add another session and transport layer without
removing the need to define freshness and turn-scoped evidence.

Sources:

- [MCP architecture](https://modelcontextprotocol.io/specification/2025-06-18/architecture)
- [MCP server primitives](https://modelcontextprotocol.io/specification/2025-06-18/server/index)
- [MCP tools](https://modelcontextprotocol.io/specification/2025-06-18/server/tools)

Anthropic's context-engineering guidance also treats message history, external
data, tools, and MCP as parts of the context state that can introduce context
pollution. It recommends explicit management of that state rather than assuming
that a larger context is safer.

Source:

- [Effective context engineering for AI agents](https://www.anthropic.com/engineering/effective-context-engineering-for-ai-agents)

## Decision

Do not add an MCP-like world-model server in this pass. Keep one canonical live
scene representation, `grounded_context_v3`, and enforce turn-scoped history
for current-scene queries. Reopen the architecture question only if the
bounded-history holdout still produces unsupported scene claims after this
boundary is live-proven.

## Validation gate

- Current-scene inventory and attribute queries must not send stale scene
  messages to response or intent LLM calls.
- Reflective scene-change queries must retain enough history to compare turns.
- The returned history after a current-scene query must not preserve the old
  scene window for the next action.
- The optional digest remains an ablation flag, not a second world-model owner.
