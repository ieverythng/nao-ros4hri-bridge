# Knowledge Base — kb_skills

# kb_skills — KnowledgeCore Client Helpers

## Purpose

`kb_skills` provides the canonical ROS2 client interface for interacting with KnowledgeCore. It encapsulates all transport logic, error handling, and timeout management behind reusable Python clients, keeping downstream modules (planners, scene grounding, chatbot) decoupled from raw ROS service details.

The module serves two roles:
1. **Runtime client library** — `KnowledgeCoreQueryClient` and `KnowledgeCoreMutationClient` for code that needs to read or write KB state
2. **Skill metadata registry** — Declares `kb_query`, `kb_revise`, `kb_add`, and `kb_remove` skills for planner discovery

## Architecture

```mermaid
flowchart TB
    subgraph kb_skills
        QC[KnowledgeCoreQueryClient]
        MC[KnowledgeCoreMutationClient]
        IL[Intent Labels]
    end

    subgraph KnowledgeCore
        KBQ[/kb/query]
        KBR[/kb/revise]
    end

    subgraph Consumers
        SG[nao_scene_grounding]
        CB[chatbot_llm]
        PL[planner_llm]
    end

    SG -->|write transient facts| MC
    MC -->|add/update/remove| KBR
    CB -->|read scene state| QC
    PL -->|discover skills| IL
    QC -->|query patterns| KBQ
```

## Public API

### KnowledgeCoreQueryClient

Read-only client wrapping `/kb/query`.

```python
from kb_skills import KnowledgeCoreQueryClient

client = KnowledgeCoreQueryClient(
    node=ros_node,
    callback_group=mutually_exclusive_group,
    service_name="/kb/query",
    timeout_sec=0.5,
)

rows = client.query_rows(
    patterns=["?obj rdf:type VisibleObject", "?obj at_location ?loc"],
    query_vars=["obj", "loc"],
    models=["default"],
    turn_id="turn_42",
    trace=my_trace_callback,  # optional
)
# Returns list[dict] parsed from JSON response
```

**Key methods:**

| Method | Purpose |
|--------|---------|
| `query_rows()` | Execute a query and return parsed result rows |
| `parse_response_rows()` | Static helper to parse JSON payloads |
| `dedupe_rows()` | Static helper to remove duplicate rows while preserving order |

The client handles service unavailability gracefully — it logs a warning once and returns an empty list rather than raising exceptions.

### KnowledgeCoreMutationClient

Write client wrapping `/kb/revise`.

```python
from kb_skills import KnowledgeCoreMutationClient, MutationResult

client = KnowledgeCoreMutationClient(
    node=ros_node,
    callback_group=mutually_exclusive_group,
    service_name="/kb/revise",
    timeout_sec=0.5,
)

# Single-statement convenience methods
result: MutationResult = client.add_fact(
    "book1 rdf:type Book",
    models=["default"],
    lifespan_sec=60.0,  # optional expiration
)

# Batch operations
result = client.revise_facts(
    ["obj1 at_location table", "obj2 at_location shelf"],
    models=["default"],
    wait_for_result=True,
)

# Remove statements
result = client.remove_fact("obsolete_fact rdf:type OldInfo")
```

**MutationResult fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation succeeded |
| `operation` | `str` | `"add"`, `"update"`, or `"remove"` |
| `dispatched` | `bool` | Whether the request was sent to the service |
| `error_msg` | `str` | Error message on failure |
| `statement_count` | `int` | Number of statements processed |

**Operation methods:**

| Method | Operation | Use Case |
|--------|-----------|----------|
| `add_fact()` / `add_facts()` | `add` | Persistent or additive world assertions |
| `revise_fact()` / `revise_facts()` | `update` | Refresh transient grounded state |
| `remove_fact()` / `remove_facts()` | `remove` | Retract invalidated facts |
| `mutate()` | configurable | Low-level interface with full control |

### Intent Labels

Canonical constants for KB-facing dialogue intents:

```python
from kb_skills import (
    KB_QUERY_INTENTS,
    KB_QUERY_VISIBLE_PEOPLE,
    KB_QUERY_VISIBLE_OBJECTS,
    KB_QUERY_SCENE_CHANGE,
)

# KB_QUERY_INTENTS is a tuple of all three labels
assert KB_QUERY_INTENTS == (
    KB_QUERY_VISIBLE_PEOPLE,    # "kb_query_visible_people"
    KB_QUERY_VISIBLE_OBJECTS,   # "kb_query_visible_objects"
    KB_QUERY_SCENE_CHANGE,      # "kb_query_scene_change"
)
```

## Skill Metadata

The package exports four skills via `package.xml` for planner discovery:

| Skill ID | Service | Purpose |
|----------|---------|---------|
| `kb_query` | `/kb/query` | Read symbolic scene state |
| `kb_revise` | `/kb/revise` | Update/revise symbolic state |
| `kb_add` | `/kb/revise` | Add persistent facts |
| `kb_remove` | `/kb/revise` | Retract obsolete facts |

All mutation skills share the `/kb/revise` endpoint but differ in their semantic intent for planner decision-making.

## Integration Patterns

### Scene Grounding Writer

`nao_scene_grounding` owns transient detector-derived facts and writes them through the mutation client:

```python
# In scene_grounding_node.py
result = self._mutation_client.revise_facts(
    statements=grounded_statements,
    models=["default"],
    lifespan_sec=30.0,  # transient facts expire
    wait_for_result=False,  # fire-and-forget for performance
)
```

### Chatbot Query Reader

`chatbot_llm` reads scene state through the query client:

```python
rows = self._query_client.query_rows(
    patterns=["?entity rdf:type VisibleObject"],
    query_vars=["entity"],
    models=["default"],
)
entities = [row.get("entity") for row in rows]
```

### Planner Mutation

Future planner-facing code should use `kb_skills` rather than calling KnowledgeCore directly:

```python
# Preferred: go through kb_skills
result = mutation_client.add_fact(
    "task_completed rdf:type Fact",
    models=["planner_memory"],
)

# Avoid: direct KnowledgeCore transport
# (keeps planner decoupled from ROS service details)
```

## Error Handling

Both clients follow a "fail gracefully" philosophy:

1. **Service unavailable** — Log warning once, return empty/failure result without raising
2. **Timeout** — Cancel the future, log warning, return failure result
3. **Service error** — Return `MutationResult` with `success=False` and populated `error_msg`
4. **Missing dependency** — If `kb_msgs` is unavailable, clients log a warning and operate as no-ops

This allows callers to handle errors inline without try/except boilerplate:

```python
result = client.add_fact(statement)
if not result.success:
    logger.warn(f"KB mutation failed: {result.error_msg}")
    # Continue execution rather than crashing
```

## Statement Coercion

The mutation client accepts flexible input types:

```python
# String input
client.add_fact("book1 rdf:type Book")

# List input
client.add_facts(["book1 rdf:type Book", "book1 authored_by AuthorX"])

# Empty strings are filtered automatically
client.add_facts(["valid statement", "", "another valid"])
# Only non-empty statements are sent
```

## Tracing

Both clients support an optional `trace` callback for debugging:

```python
def my_trace(turn_id: str, stage: str, message: str, level: str):
    print(f"[{turn_id}] {stage}: {message}")

client.query_rows(
    patterns=[...],
    query_vars=[...],
    models=[...],
    turn_id="conversation_123",
    trace=my_trace,
)
```

Trace stages for mutations: `KB_ADD`, `KB_REMOVE`, `KB_REVISE` (or `KB_MUTATE` for custom operations).

## Dependencies

- `rclpy` — ROS2 Python client library
- `kb_msgs` — Service definitions for `Query` and `Revise` (optional at import time, required at runtime)
