# kb_skills

`kb_skills` is the local KnowledgeCore boundary. It provides reusable clients
and skill metadata for KB reads and writes so planner/chatbot/executor code does
not embed raw KnowledgeCore transport details.

## Owns

- `KnowledgeCoreQueryClient`
- `KnowledgeCoreMutationClient`
- canonical KB query intent labels
- package skill metadata for KB query/revise capability

It does not decide what the robot should say or execute.

## Public ROS Services Used

| Service | Type | Purpose |
| --- | --- | --- |
| `/kb/query` | `kb_msgs/srv/Query` | Read symbolic facts |
| `/kb/revise` | `kb_msgs/srv/Revise` | Add/update/remove symbolic facts |

## Query Surface

- `query_rows(...)`

Used by `chatbot_llm` to build `knowledge_snapshot` prompt context.

## Mutation Surface

- `add_facts(...)`
- `revise_facts(...)`
- `remove_facts(...)`
- `add_fact(...)`
- `revise_fact(...)`
- `remove_fact(...)`

Used by `nao_scene_grounding` for transient detector-derived object facts.

## Contract Role

Recommended split:

- `nao_scene_grounding`: owns detector-derived facts.
- `chatbot_llm`: owns prompt formatting and when to query.
- `planner_llm`: may use this boundary later for planner-visible KB operations.
- `nao_orchestrator`: should not become a raw KB client.

## Tests

```bash
PYTHONPATH=src/kb_skills python3 -m pytest -q src/kb_skills/test
```
