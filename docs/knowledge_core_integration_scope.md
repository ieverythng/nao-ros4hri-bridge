# KnowledgeCore Integration Scope

This document records exactly what was added or changed for the local
KnowledgeCore and `interaction_sim` integration so the branch can be reviewed
against upstream behavior.

## Upstream Packages Added To `src/`

The following packages were added as source overlays and are intended to remain
upstream code:

- `src/kb_msgs`
- `src/knowledge_core`
- `src/interaction_sim`

Current local state:

- no source changes inside `src/kb_msgs`
- no source changes inside `src/knowledge_core`
- no source changes inside `src/interaction_sim`

These packages were added only so the workspace can build and run the upstream
knowledge/simulator stack locally.

## Why `kb_msgs` Was Added

`kb_msgs` is not a local invention. It is part of the official ROS interface
used by `knowledge_core`.

In the upstream `knowledge_core` repo:

- `package.xml` declares `kb_msgs` as an `exec_depend`
- `knowledge_core_ros.py` imports services such as `Query`, `Revise`, `Manage`,
  `About`, `Lookup`, `Sparql`, and `Event` from `kb_msgs`
- the upstream README states that `kb_msgs` is required if the ROS interface is
  used

`reasonable` is separate and optional. It is only the OWL RL reasoner backend
used by `knowledge_core` when reasoning is enabled.

## Local Changes Outside Upstream KB Packages

The local integration changes are in the existing fork overlays and launch
surface, not in `knowledge_core` or `interaction_sim`.

### `chatbot_llm`

Custom local change:

- add a read-only `/kb/query` client
- fetch a KB snapshot once per response turn
- append that snapshot to the LLM prompts for response generation and intent
  extraction

This is local glue so the LLM can consume the upstream symbolic state. It does
not modify `knowledge_core` semantics or API.

### `dialogue_manager`

Custom local change:

- set `default_chat_configuration` to a JSON block that enables the local
  `chatbot_llm` KB snapshot feature for the default chat

This change exists so upstream `interaction_sim` can continue launching
`dialogue_manager.launch.py` with `enable_default_chat=True` and still exercise
the KB-grounded chatbot path.

### `nao_chatbot`

Custom local changes:

- add launch support for starting `knowledge_core`
- pass the same default KB role configuration in the migrated NAO launch path
- document the simulator-based test path

## What `knowledge_snapshot` Is

`knowledge_snapshot` is a local configuration block interpreted only by the
forked `chatbot_llm` package in this workspace. It is not part of upstream
`knowledge_core` and it is not a new KB service.

Its purpose is to describe how `chatbot_llm` should query `/kb/query` before a
turn:

- `enabled`
- `patterns`
- `vars`
- `models`
- `max_results`
- `max_chars`

It is effectively a prompt-grounding adapter between the upstream KB and the
LLM.

## Current Behavioral Contract

What stays upstream:

- `kb_msgs` service/message definitions
- `knowledge_core` storage, reasoning, events, and ROS APIs
- `interaction_sim` simulator loop and UI

What is custom in this workspace:

- the LLM-side read-only KB consumer path in `chatbot_llm`
- the launch/default-role wiring that turns that consumer path on for default
  chat sessions

## If Maximum Upstream Alignment Is Preferred

The most custom part today is `knowledge_snapshot`.

If you want a narrower local diff, the next simplification would be:

- remove role-level `knowledge_snapshot` JSON from `dialogue_manager`
- enable KB querying directly through `chatbot_llm` node parameters instead

That would keep the same behavior while reducing one custom configuration seam.
