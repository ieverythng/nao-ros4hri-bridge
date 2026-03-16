# KnowledgeCore Integration Scope

This document records exactly what needs to be present in the workspace for the
ROS4HRI migration to use the upstream symbolic knowledge stack, and which parts
of the current integration are upstream versus local glue.

## Source Overlays To Download

Codex on a fresh laptop should ensure these source overlays exist under `src/`:

- `src/chatbot_llm`
- `src/dialogue_manager`

- `src/kb_msgs`
- `src/knowledge_core`
- `src/interaction_sim`

Canonical clone sources for the forked migration packages:

- `https://github.com/ieverythng/nao_chatbot_llm.git`
  branch: `feat/juan_nao_chatbot`
- `https://github.com/ieverythng/dialogue_manager.git`
  branch: `juan-feat-1`

Canonical clone sources for the upstream knowledge/simulator packages:

- `https://github.com/pal-robotics/kb_msgs.git`
- `https://gitlab.iiia.csic.es/socialminds/neurosymbolic-ai/knowledge_core.git`
- `https://gitlab.iiia.csic.es/socialminds/ros4hri/interaction_sim.git`

The tracked helper for this workspace is:

- `./scripts/bootstrap_socialminds_sources.sh`

These packages should stay upstream and unmodified unless a specific bug forces
an explicit local patch.

The forked overlays are part of the migration itself:

- `chatbot_llm` is the NAO-side backend where the read-only KB-to-LLM adapter
  lives
- `dialogue_manager` is the canonical ROS4HRI dialogue runtime used both by the
  migrated NAO stack and by `interaction_sim`

## Why `kb_msgs` Is Required

`kb_msgs` is part of the official ROS interface for `knowledge_core`.

In the upstream `knowledge_core` repo:

- `package.xml` declares `kb_msgs` as an `exec_depend`
- `knowledge_core_ros.py` imports ROS services and messages from `kb_msgs`
- the upstream README says `kb_msgs` is required when using the ROS interface

`reasonable` is separate and optional. It provides OWL RL reasoning support,
but it is not the ROS transport layer.

## Upstream Behavior

What stays upstream in the current test path:

- `kb_msgs` defines the ROS message/service types
- `knowledge_core` stores symbolic facts, performs optional reasoning, and
  exposes `/kb/query`, `/kb/revise`, `/kb/events`, and related APIs
- `interaction_sim` launches the upstream simulator loop, including
  `knowledge_core`, `chatbot_llm`, `dialogue_manager`, and the UI tools

No source changes are currently required inside:

- `src/kb_msgs`
- `src/knowledge_core`
- `src/interaction_sim`

The migrated stack also assumes these fork overlays are present:

- `src/chatbot_llm`
- `src/dialogue_manager`

## Local Changes In The Migration Workspace

The current local integration changes are limited to the existing forked
packages and NAO launch surface.

### `chatbot_llm`

Local change:

- add a read-only `/kb/query` client
- fetch KB state once per response turn
- convert the structured `/kb/query` result into prompt text for the LLM

This is the only vital local seam in the current test path. It exists because
our `chatbot_llm` backend is the NAO-side LLM implementation and upstream
`knowledge_core` does not natively know how to feed an LLM prompt.

### `dialogue_manager`

Current test-path change:

- no custom default role configuration is required

For the current test path, `dialogue_manager` stays on its upstream-style
default chat flow and does not inject a custom KB role block.

### `nao_chatbot`

Local changes:

- add launch support for starting `knowledge_core` in the migrated NAO stack
- document the upstream simulator-based smoke-test path

## What The Previous `knowledge_snapshot` Seam Was

`knowledge_snapshot` is a local `chatbot_llm` concept, not an upstream
`knowledge_core` concept.

It was a role-level JSON block used to tell `chatbot_llm` how to query
`/kb/query` before a turn:

- `enabled`
- `patterns`
- `vars`
- `models`
- `max_results`
- `max_chars`

That mechanism still exists in the local `chatbot_llm` code, but it is not the
default integration path anymore.

## Current Default Test Path

The current default test path is deliberately narrower:

- `interaction_sim` launches the upstream simulator stack
- `chatbot_llm` has KB querying enabled through node parameters
- `dialogue_manager` uses its normal default chat behavior
- `knowledge_core` remains the only symbolic store

This keeps the test closer to upstream behavior while preserving the minimum
LLM-side glue needed for the NAO chatbot.

## Expected Smoke Test

After the overlays are present and the workspace is built, the intended local
test is:

1. launch `interaction_sim`
2. add or move objects through `rqt_human_radar` or `/kb/revise`
3. verify the symbolic state through `/kb/query`
4. ask the chatbot about those objects through `rqt_chat`

If that works, the migration is complete enough for the first integration test.
