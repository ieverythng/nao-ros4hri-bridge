---
name: iiia-ros4hri-check
description: Apply ROS 2, ROS4HRI, SocialMinds, and IIIA-specific architectural guardrails to code changes in this repo or closely related repos. Use when implementing, reviewing, refactoring, or validating ROS packages, launch files, actions, services, topics, skills, lifecycle nodes, or planner/orchestrator/chatbot integrations, especially when nested repos or upstream-sensitive packages are involved.
---

# IIIA ROS4HRI Check

## Overview

Run a guardrail pass for ROS, ROS4HRI, SocialMinds, and repo-specific
architecture before or after meaningful code changes. Use it to reduce repeated
manual reminders about ownership boundaries, lifecycle usage, interface choice,
package structure, and upstream-sensitive edits.

## Quick Start

1. From the **repository root** (`nao-ros4hri-bridge`), run
   `python3 scripts/ros4hri_change_audit.py` to identify touched packages and the
   guardrails that apply. The canonical script lives in the repo; the copy under
   `.codex/skills/iiia-ros4hri-check/scripts/ros4hri_change_audit.py` delegates to
   it so the skill stays in sync when this repo is checked out as a subfolder, with
   a standalone fallback when the canonical script is unavailable in a branch.
2. Read `references/repo-boundaries.md` to understand which packages are safe to
   evolve freely and which require seam-focused changes only.
3. Read `references/ros4hri-guardrails.md` for the actual checklist.
4. Apply the narrowest useful fixes.
5. Validate with targeted tests, `py_compile`, launch checks, or containerized
   ROS checks as appropriate.
6. When adjusting prompt/skill wording or routing rules, prefer a bounded
   SkillOpt-style pass (`baseline -> mutate -> holdout gate -> accept/reject log`).

## Workflow

### 1. Classify the scope before editing

- Identify the touched ROS packages, nested repos, and launch surfaces.
- Treat `src/chatbot_llm` and `src/dialogue_manager` as nested repos with extra
  care.
- Treat ROS4HRI and SocialMinds-aligned packages as contract-sensitive even when
  they live inside this monorepo.
- Prefer seam-focused changes over broad rewrites when a package is fork-tracked
  or upstream-aligned.

### 2. Preserve architectural ownership

- Keep `dialogue_manager` as the dialogue and speaking owner.
- Keep `chatbot_llm` as the user-facing LLM and grounded dialogue owner.
- Keep `planner_llm` as the task supervisor and planner.
- Keep `nao_orchestrator` as the deterministic executor and planner-dialogue relay
  seam owner.
- Keep `kb_skills` as the KnowledgeCore boundary.
- Keep `nao_scene_grounding` as the detector-to-KB grounding bridge.
- Do not bypass exposed ROS interfaces with direct robot-specific shortcuts when
  the ROS driver or skill seam already exists.
- In this stack, keep planner dialogue flow as
  `planner_llm -> /planner/dialogue_act -> nao_orchestrator relay -> dialogue_manager`;
  do not wire `dialogue_manager` directly to planner dialogue in live seam
  profiles.
- Preserve orchestrator as pure executor/relay: planner dialogue must not become
  direct chatbot wording by default in seam-test profiles unless the active
  contract explicitly says so.

### 3. Check ROS interface choices

- Use topics for streamed state, observations, or status.
- Use services for short request/response operations.
- Use actions for long-running or feedback-bearing commands.
- Reuse existing message, service, and action types before inventing new ones.
- Favor ROS4HRI-compatible public contracts and keep NAO-specific logic behind
  adapter or execution packages.

### 4. Check lifecycle and package patterns

- When a local robot-facing package already follows a lifecycle or `rpk`
  template, preserve that pattern.
- Do not convert upstream or fork-tracked packages to lifecycle nodes casually.
- Keep package structure consistent with the repo: `package.xml`, `setup.py`,
  config, launch, and README surfaces should remain coherent.
- Update launch wiring and package metadata together when new runtime seams are
  added.

### 5. Validate like a ROS package, not just a Python module

- Run the narrowest relevant tests first.
- Add or update focused tests when the change affects contracts or routing.
- Use `python3 -m py_compile` for touched Python entrypoints.
- Use launch argument inspection or containerized ROS checks when runtime wiring
  changes.
- Call out anything that could only be fully validated inside the real ROS or
  robot environment.

## Stop Conditions

- Do not broaden changes into vendored or unrelated packages just because they
  are nearby.
- Do not move responsibilities across nodes unless the architecture explicitly
  calls for it.
- Do not create new ROS interfaces when existing ones are sufficient.
- Do not deslop or normalize upstream code unless it is in scope and the change
  is justified.

## Resources

- `references/repo-boundaries.md` (under this skill directory)
  - package ownership, nested repo notes, and repo-specific sensitivity map
- `references/ros4hri-guardrails.md` (under this skill directory)
  - ROS, ROS4HRI, SocialMinds, and IIIA-specific checklist with external links
- **`scripts/ros4hri_change_audit.py` at the repo root** (not only under `.codex/`)
  - summarize changed packages and print the most relevant guardrail cues
- `scripts/link_into_codex_home.sh`
  - install this skill into `${CODEX_HOME:-$HOME/.codex}/skills` with either
    `copy` (branch-independent local snapshot) or `symlink` (always follow this repo checkout)
