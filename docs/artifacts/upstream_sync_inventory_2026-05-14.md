# Upstream Sync Inventory (Phase D) - 2026-05-14

This inventory is the staged prep artifact for upstream reconciliation work.
It records the exact remotes, known heads, and conflict domains before any
merge/rebase action.

## 1) dialogue_manager

- Local path: `src/dialogue_manager`
- Active local branch: `juan-feat-1`
- Remotes:
  - `origin`: <https://github.com/ieverythng/dialogue_manager.git>
  - `upstream`: <https://github.com/ros4hri/dialogue_manager.git>
  - `gitlab-upstream`: `git@gitlab.iiia.csic.es:socialminds/ros4hri/dialogue_manager.git`
- Observed upstream heads:
  - `upstream/main`: `862a5207527db8936549ef2ad328000180df2495`
  - `upstream/refactoring`: `de09d9cbfe5c9076723be1e839901d1b671cd37e`

Conflict domains expected:

- planner completion wording ownership (`manager_node.py`, `chatbot_client.py`)
- `/planner/dialogue_act` handling policy
- defaults (`config/00-defaults.yml`)
- unit tests around completion pass and chatbot request envelopes

## 2) chatbot_llm

- Local path: `src/chatbot_llm`
- Active local branch: `feat/juan_nao_chatbot`
- Remotes:
  - `origin`: <https://github.com/ieverythng/nao_chatbot_llm.git>
  - `upstream`: <https://gitlab.iiia.csic.es/socialminds/ros4hri/chatbot_llm.git>
- Observed heads:
  - `upstream/main`: `0bb5060ea7ff94ef02293f70243951cf409d2ec0`
  - `origin/main`: `688d8db3c5ece9ce558a11becaa57cac5425e421`
  - `origin/feat/juan_nao_chatbot`: `e754d37ed53b4e64e53fcd424797d4385be976ff`

Conflict domains expected:

- planner handoff (`planner_handoff.py`, `planner_request_adapter.py`)
- system-turn completion rendering (`turn_engine.py`, `node_impl.py`)
- skill catalog loading source of truth (`skill_catalog.py`)
- parser/intent tests with planner payload normalization changes

## 3) chatbot_msgs

- Local path in this workspace: **not currently vendored as a nested repo**
- Current state: provided as dependency package only (no local git remote set in `src/`)

Required follow-up before merge stage:

- add explicit source checkout (or submodule/subtree decision) for `chatbot_msgs`
- register both GitHub and GitLab remotes for parity with SocialMinds upstream
- pin baseline commit before pulling API changes used by dialogue/chatbot stacks

## 4) Staged merge order (mandatory)

1. `chatbot_msgs` (contracts first)
2. `dialogue_manager` (dialogue ownership and completion routing)
3. `chatbot_llm` (completion rendering + planner handoff alignment)

Why this order:

- message and service contracts must settle before node behavior merges
- dialogue-side ownership must be stable before chatbot-side planner/system paths are rebased

## 5) Safety rails before merge/rebase

- run `python3 scripts/ros4hri_change_audit.py --mode working` before and after each stage
- run focused unit tests for touched package before moving to next package
- run one sim-profile runtime smoke after each stage:
  - planner request ingress
  - one execution success
  - one failure/cancel path
