# Codex Handoff — ROS4HRI Stack Hardening Pass

**Date:** 2026-05-10  
**Owner context:** Álvaro / PIPELINE / RESEARCH-GLOBAL orchestration style  
**Primary runtime target:** NAO ROS4HRI planner stack  
**Audience:** Codex / Warp / GitHub-agent implementation workers

## 0. Branches and repository scope

Use these exact branches unless the operator explicitly changes scope:

| Repo | Branch | Role |
|---|---|---|
| `ieverythng/nao-ros4hri-bridge` | `feat/TFM-LLM_planner` | Monorepo / main planner, orchestrator, contracts, launch, knowledge tooling |
| `ieverythng/nao_chatbot_llm` | `feat/planner_llm_hooks` | Chatbot backend, prompt pack, routing, knowledge snapshot, planner handoff |
| `ieverythng/dialogue_manager` | `juan-feat-1` | Upstream-aligned dialogue manager with planner dialogue-act bridge |

`RESEARCH-GLOBAL` was provided as an orchestration reference repo, but the current GitHub connector did not list or fetch it. Keep this task plan compatible with that future orchestration by using issue/task cards with: **repo, branch, objective, context, files, tests, acceptance gates, risks**.

## 1. Current evidence-backed diagnosis

The backend and vLLM routing path are now stable enough that the major remaining failures are not primarily transport bugs.

The current failure class is:

> The robot can execute deterministic actions, but the semantic evidence produced by composite skills is not being carried cleanly into the final user-facing answer.

The existing docs already define the layered runtime:

```text
dialogue_manager -> chatbot_llm -> planner_llm -> nao_orchestrator -> skills
```

The current workflow assigns clear ownership: dialogue manager speaks, chatbot routes/declares, planner supervises, planner_common normalizes contracts, and orchestrator executes deterministically.

The latest architecture note also explicitly states that completion wording currently goes through a `dialogue_manager -> chatbot_llm` completion pass, while the target architecture eventually moves planner ingress through `nao_orchestrator` as deterministic gate. Treat those as two separate migrations: **fix wording/evidence now, planner-gate ownership later**.

## 2. Non-negotiable architectural principles

1. **Dialogue manager owns speech realization.** Do not make `planner_llm` or `nao_orchestrator` speak directly except through existing dialogue-act/TTS seams.
2. **Chatbot declares route and intent, not executable plans.** `chatbot_llm` may produce `verbal_ack`, `route`, `confidence`, and `user_intent` metadata. It must not own final skill steps.
3. **Planner owns plan structure and supervision.** `planner_llm` plans over abstract skills, not robot-specific APIs.
4. **Orchestrator owns deterministic execution.** `nao_orchestrator` validates and executes plan steps. It should return typed execution feedback.
5. **Skills must return evidence, not just prose.** Composite skills like `scan` need structured result payloads with a short summary mirror for old paths.
6. **Prompts are configuration; contracts are code.** Prompt-pack migration is good, but validation must stay in Python tests/contracts.

## 3. Indexed repo findings

### 3.1 `nao-ros4hri-bridge`

The feature branch is large and mature: planner_common, planner_llm, planner gate, GitNexus knowledge tooling, docs/contracts, and multiple handoff artifacts are present.

Important current state:

- `docs/current_workflow.md` defines layer ownership and runtime topics.
- `docs/planner_architecture_current.md` states that `scan` is first-party, planner-visible, and completion wording currently goes through chatbot.
- `docs/contracts.md` documents `/planner/request`, `/intents`, `/planner/execution_feedback`, `/planner/dialogue_act`, `knowledge_snapshot`, and `/scene/summary`.
- `AGENTS.md` introduces the GitNexus layer and instructs agents to query/context/source, then confirm in code.

Gaps:

- Execution feedback currently centers on `result_summary` string. That is too weak for scan/person evidence.
- `planner_llm` still embeds the main system prompt in `planner_engine.py`.
- `skill_registry.json` should make `scan` explicit and MCP-like, even if runtime can derive it.
- Planner gate exists but is not the first PR target.

### 3.2 `nao_chatbot_llm`

This branch has a full rewrite around `backend_config.py`, `prompt_pack.py`, `prompt_builders.py`, `turn_engine.py`, `planner_handoff.py`, `planner_request_adapter.py`, `knowledge_snapshot.py`, and `ollama_transport.py`.

Good:

- Prompt pack exists.
- Planner-mode single-stage response routing exists.
- Knowledge snapshots and recent scene memory exist.
- Tests already cover planner-mode routing, prompt-pack loading, knowledge snapshots, and planner handoff.

Gaps:

- `ollama_transport.py` now supports OpenAI-compatible `/v1/chat/completions`, so the name is misleading.
- `_EXECUTION_HINT_MARKERS` in `turn_engine.py` is too broad. Terms like `person`, `people`, and `room` can accidentally convert knowledge/memory questions into execution.
- Completion wording is currently handled as a normal chatbot turn when sent by dialogue_manager. That can re-trigger routing.

### 3.3 `dialogue_manager`

This branch intentionally keeps upstream alignment and adds only limited local changes.

Good:

- Planner dialogue-act subscription exists.
- `notify_completion` can request chatbot wording through the default dialogue.
- TTS ownership remains in dialogue_manager.

Gaps:

- `notify_completion -> send_default_system_input()` causes the completion wording request to enter the normal chatbot turn path.
- That path can infer `route=execution`, publish planner requests, or contaminate history.
- `explain_failure` is currently suppressed from TTS; sanitize instead of suppressing.

## 4. Implementation phases

## Phase 0 — Agent/branch/bootstrap discipline

**Goal:** Make every implementation run reproducible.

**Repos/files**

- `nao-ros4hri-bridge/AGENTS.md`
- `nao-ros4hri-bridge/docs/knowledge/WORKFLOWS.md`
- `.codex/skills/iiia-ros4hri-check/*`
- `.codex/skills/deslop-refactor/*`

**Actions**

1. Start from the exact branches listed in section 0.
2. In `nao-ros4hri-bridge`, run knowledge tooling before deep edits:

```bash
tools/knowledge/status.sh || true
scripts/bootstrap_socialminds_sources.sh || true
tools/knowledge/index_repo.sh || true
```

3. Use GitNexus when available:
   - exploration: `query -> context -> source`
   - debugging: `query symptom -> context suspect -> source`
   - refactor: `context + impact -> source edit -> tests -> index refresh`
4. Always spot-check source code even if GitNexus returns confident answers.

**Acceptance**

- Branch names logged in PR description.
- Baseline tests run or explicitly marked unavailable.
- Any GitNexus finding is source-confirmed before code changes.

## Phase 1 — Stop planner completion wording from re-entering normal routing

**Priority:** P0  
**Reason:** Highest immediate impact on scan/report robustness.

### Option A — Fast safe path

**Files**

- `dialogue_manager/dialogue_manager/manager_node.py`

**Change**

For `PlannerDialogueAct.act == "notify_completion"`:

1. Prefer `dialogue_act.text_hint` or `dialogue_act.context.result_summary`.
2. Speak it directly through `_tts_client.speak()`.
3. Do **not** call `_ask_chatbot_for_planner_reply()` unless `use_llm_completion_wording` parameter is true.

Add parameter:

```yaml
planner_completion_wording_mode: "direct"  # direct | chatbot
```

Default: `direct`.

**Expected behavior**

- Scan completion speaks the factual result directly.
- No new chatbot route decision is created for planner completion.
- No extra `/planner/request` is published because of a completion prompt.

### Option B — Proper wording-only service

**Files**

- `chatbot_llm/node_impl.py`
- `chatbot_llm/turn_engine.py`
- `chatbot_llm/prompt_builders.py`
- `dialogue_manager/chatbot_client.py`
- `dialogue_manager/manager_node.py`

**Change**

Add a wording-only path that accepts a planner dialogue act and returns only:

```json
{"verbal_ack":"I found one person, but I cannot confirm it is the same person as before."}
```

Rules:

- no route inference
- no planner handoff
- no intent publication
- no execution fallback
- no normal user-turn history pollution unless explicitly enabled

**Acceptance tests**

- `notify_completion` with `text_hint` speaks directly.
- `notify_completion` with `result_summary` speaks directly.
- `notify_completion` does not publish planner request.
- `ask_clarification` still speaks and awaits response.
- `explain_failure` emits sanitized failure text instead of being fully suppressed.

## Phase 2 — Add structured scan evidence

**Priority:** P0  
**Reason:** Fixes human/object confusion and makes composite skills thesis-worthy.

**Files**

- `nao-ros4hri-bridge/src/planner_common/planner_common/contracts.py`
- `nao-ros4hri-bridge/docs/contracts.md`
- `nao-ros4hri-bridge/src/nao_orchestrator/nao_orchestrator/intent_rules.py`
- `nao-ros4hri-bridge/src/nao_orchestrator/nao_orchestrator/orchestrator.py`
- `nao-ros4hri-bridge/src/planner_llm/planner_llm/supervisor.py`
- `dialogue_manager/dialogue_manager/manager_node.py`

**Change**

Add `result_payload` to execution feedback while keeping `result_summary` for backward compatibility.

Example payload:

```json
{
  "skill": "scan",
  "target": "people",
  "target_kind": "people",
  "target_found": true,
  "people": [
    {
      "id": "anonymous_person_daeba",
      "source": "hri_tracked_persons",
      "last_seen_age_sec": 0.4
    }
  ],
  "objects": [],
  "summary_text": "I found one person (id: anonymous_person_daeba).",
  "confidence_policy": "grounded_current_observation"
}
```

**Scan priority rules**

For `target_kind in {"person","people","human","humans"}`:

1. Use fresh `/humans/persons/tracked` IDs first.
2. Use KB/world-model human entities second.
3. Use `/scene/summary` person-like detections third.
4. Only mention object detections if no person was confirmed, and clearly say they are not the requested target.

**Compatibility**

- Continue filling `result_summary = result_payload.summary_text`.
- `PlannerSupervisor.latest_result_summary` remains valid.
- Add `latest_result_payload` alongside it.

**Acceptance tests**

- Person scan with fresh person IDs returns `target_found=true`.
- Person scan without people but with objects returns `target_found=false` and does not claim object success.
- Scene scan can summarize objects.
- Result payload is preserved into planner dialogue-act context.
- Dialogue manager can speak `summary_text` directly.

## Phase 3 — Fix chatbot route semantics for scan/memory/knowledge questions

**Priority:** P0/P1  
**Reason:** Prevents “last scan” and “same person” questions from triggering new execution.

**Files**

- `nao_chatbot_llm/chatbot_llm/turn_engine.py`
- `nao_chatbot_llm/chatbot_llm/prompt_builders.py`
- `nao_chatbot_llm/chatbot_llm/prompt_pack.py`
- `nao_chatbot_llm/config/chat_prompt_pack.yaml`
- `nao_chatbot_llm/test/test_turn_engine.py`

**Change**

Replace broad execution markers with a two-class policy.

Execution request examples:

- “scan the room”
- “look around”
- “look around again”
- “check for people now”
- “move your head left”
- “stand up”

Knowledge/memory question examples:

- “what did you see?”
- “did you see anyone?”
- “did you see people in the last scan?”
- “what was the person id?”
- “is it the same person as before?”
- “what can you currently see?”

The second class should route to `knowledge_query` unless the user explicitly asks to scan/check/look again.

**Implementation sketch**

Add helper:

```python
def _looks_like_knowledge_or_memory_question(user_text: str) -> bool:
    ...
```

Then in `_infer_route()`:

1. if explicit route from model is valid, keep it only if not contradicted by hard safety rules;
2. if knowledge/memory question, return `knowledge_query`;
3. if physical imperative/action request, return `execution`;
4. else dialogue.

**Acceptance tests**

- `did you see people in the last scan?` -> `knowledge_query`
- `what was the person id?` -> `knowledge_query`
- `is that the same person as before?` -> `knowledge_query`
- `scan the room for a person` -> `execution`
- `look around and tell me what you see` -> `execution`
- LLM empty response for scan request still hands off to planner.
- LLM empty response for memory question does not hand off to planner.

## Phase 4 — Make scan an explicit MCP-like skill affordance

**Priority:** P1  
**Reason:** Improves planner reliability without adding hardcoded special cases.

**Files**

- `nao-ros4hri-bridge/src/planner_llm/config/skill_registry.json`
- `nao-ros4hri-bridge/src/planner_llm/planner_llm/skill_registry.py`
- `nao-ros4hri-bridge/src/planner_llm/test/test_skill_registry.py`
- `nao-ros4hri-bridge/docs/contracts.md`

**Change**

Add explicit `scan` entry to `skill_registry.json`, even if derived runtime loading already exposes it.

Recommended fields:

- aliases: `look_around`, `inspect_scene`, `check_visible_entities`
- params: `target`, `target_kind`, `max_sweeps`, `evidence_policy`
- expected_effects: bounded head scan + refreshed evidence
- observable_success: `result_payload.target_found`, `result_payload.people`, `result_payload.objects`
- planner_guidance:
  - use scan for fresh perception
  - do not use scan for last-scan questions unless user asks to scan again
  - prioritize people evidence for people-target scans

**Acceptance**

- `prompt_manifest()` includes scan with aliases and guidance.
- Planner fixtures choose scan for fresh scan commands.
- Planner fixtures avoid scan for memory-only questions when chatbot routes them as knowledge.

## Phase 5 — Planner prompt-pack symmetry

**Priority:** P1  
**Reason:** Aligns with uploaded hardening plan; makes prompt iteration controlled and testable.

**Files**

- `nao-ros4hri-bridge/src/planner_llm/planner_llm/prompt_pack.py`
- `nao-ros4hri-bridge/src/planner_llm/config/planner_prompt_pack.yaml`
- `nao-ros4hri-bridge/src/planner_llm/planner_llm/planner_engine.py`
- `nao-ros4hri-bridge/src/planner_llm/planner_llm/planner_node.py`
- `nao-ros4hri-bridge/src/planner_llm/test/test_planner_prompt_pack.py`

**Change**

Move planner policy prose and validation-retry copy out of Python constants into YAML.

Keep in Python:

- allowed step validation
- skill filtering
- mixed say/executable rejection
- JSON extraction
- retry logic
- fallback/requested-plan logic

Add:

- `prompt_pack_version`
- prompt pack path logging
- fallback behavior on invalid YAML
- tests for partial override merge

**Acceptance**

- Existing planner tests pass.
- Bad YAML falls back safely.
- Startup logs prompt pack path + version.
- No weakening of planner contract validation.

## Phase 6 — Clean dialogue-only intent leakage and hardcoded robot utterances

**Priority:** P2  
**Reason:** Architectural hygiene after urgent scan/completion stability.

**Files**

- `nao_chatbot_llm/chatbot_llm/prompt_builders.py`
- `nao_chatbot_llm/chatbot_llm/intent_rules.py`
- `nao-ros4hri-bridge/src/nao_orchestrator/nao_orchestrator/intent_rules.py`
- `nao-ros4hri-bridge/src/planner_llm/planner_llm/planner_engine.py`
- `dialogue_manager/dialogue_manager/manager_node.py`

**Change**

- Stop producing executable `SAY` intents for `greet`, `identity`, `wellbeing`, `help`.
- Keep orchestrator ignore shims temporarily.
- Move hardcoded wording to prompt packs or dialogue response layer.
- Remove planner `greet -> say` rule after dialogue-only route is proven stable.

**Acceptance**

- Greeting stays entirely in chatbot/dialogue path.
- Orchestrator receives no execution intent for greeting/help in normal dialogue.
- Direct `/intents` compatibility tests still pass.

## Phase 7 — Rename transport or introduce provider-neutral wrapper

**Priority:** P2  
**Reason:** Maintainability, not primary behavior.

**Files**

- `nao_chatbot_llm/chatbot_llm/ollama_transport.py`
- `nao_chatbot_llm/chatbot_llm/node_impl.py`
- `nao_chatbot_llm/test/test_ollama_transport.py`

**Option A**

Keep file for compatibility but add:

```python
class ChatCompletionTransport(OllamaTransport):
    ...
```

and import/use the neutral name.

**Option B**

Rename file to `chat_completion_transport.py` and leave `ollama_transport.py` as compatibility shim.

**Acceptance**

- Existing tests pass.
- OpenAI-compatible `/v1/chat/completions` behavior remains covered.
- Log messages no longer say “Ollama” for vLLM/OpenAI-compatible endpoints.

## Phase 8 — Planner gate ownership migration

**Priority:** P3  
**Reason:** Correct target architecture, but not before P0/P1 stability.

**Files**

- `nao-ros4hri-bridge/src/nao_orchestrator/nao_orchestrator/planner_gate.py`
- `nao-ros4hri-bridge/src/nao_orchestrator/nao_orchestrator/orchestrator.py`
- `nao_chatbot_llm/chatbot_llm/backend_config.py`
- `nao_chatbot_llm/chatbot_llm/planner_handoff.py`
- launch files in `nao_chatbot`

**Change**

Migrate planner ingress from:

```text
chatbot_llm -> /planner/request
```

to:

```text
chatbot_llm -> /nao_orchestrator/planner_request -> /planner/request
```

Only after completion wording and scan evidence are stable.

**Acceptance**

- Duplicate active goals rejected.
- Supersede/cancel/clarification pass through correctly.
- `planner_gate` clears on completion/failure/cancellation.
- Launch defaults documented.

## 5. Suggested GitHub issue/task cards

### Issue 1 — P0: Stop planner completion from re-entering normal chatbot routing

**Repo:** `dialogue_manager`  
**Branch:** `juan-feat-1`  
**Files:** `manager_node.py`, `chatbot_client.py`, tests  
**Acceptance:** completion act speaks factual text; no new planner request is created.

### Issue 2 — P0: Add structured scan result payload to execution feedback

**Repo:** `nao-ros4hri-bridge`  
**Branch:** `feat/TFM-LLM_planner`  
**Files:** `planner_common/contracts.py`, `nao_orchestrator/orchestrator.py`, `intent_rules.py`, `planner_llm/supervisor.py`, docs/tests  
**Acceptance:** person-target scan returns typed people evidence and `summary_text`.

### Issue 3 — P0/P1: Separate execution requests from knowledge/memory questions

**Repo:** `nao_chatbot_llm`  
**Branch:** `feat/planner_llm_hooks`  
**Files:** `turn_engine.py`, `prompt_builders.py`, `prompt_pack.py`, `chat_prompt_pack.yaml`, tests  
**Acceptance:** “last scan/person id/same person” routes to knowledge/memory, not execution.

### Issue 4 — P1: Make scan explicit and MCP-like in skill registry

**Repo:** `nao-ros4hri-bridge`  
**Branch:** `feat/TFM-LLM_planner`  
**Files:** `skill_registry.json`, `skill_registry.py`, tests/docs  
**Acceptance:** prompt manifest contains enriched scan affordance and aliases.

### Issue 5 — P1: Add planner prompt pack

**Repo:** `nao-ros4hri-bridge`  
**Branch:** `feat/TFM-LLM_planner`  
**Files:** `planner_llm/prompt_pack.py`, `planner_prompt_pack.yaml`, `planner_engine.py`, `planner_node.py`, tests  
**Acceptance:** prompt version logged; invalid YAML fallback; current planner tests pass.

### Issue 6 — P2: Clean dialogue-only intent leakage

**Repos:** `nao_chatbot_llm`, `nao-ros4hri-bridge`  
**Branches:** listed above  
**Acceptance:** greetings/help stay in dialogue layer; no execution intent emitted.

### Issue 7 — P2: Rename or wrap Ollama transport

**Repo:** `nao_chatbot_llm`  
**Branch:** `feat/planner_llm_hooks`  
**Acceptance:** provider-neutral naming; OpenAI-compatible behavior unchanged.

### Issue 8 — P3: Move planner ingress behind orchestrator gate

**Repos:** `nao-ros4hri-bridge`, `nao_chatbot_llm`  
**Acceptance:** `/nao_orchestrator/planner_request` is default ingress; gate manages active goal state.

## 6. Baseline tests to run before and after each PR

### `nao_chatbot_llm`

```bash
cd src/chatbot_llm
PYTHONPATH="$PWD:../planner_common:../kb_skills" \
python3 -m pytest -q \
  test/test_turn_engine.py \
  test/test_planner_request_adapter.py \
  test/test_knowledge_snapshot.py \
  test/test_prompt_pack.py \
  test/test_ollama_transport.py
```

### `nao-ros4hri-bridge`

```bash
PYTHONPATH=src/planner_common:src/planner_llm:src/nao_orchestrator:src/kb_skills \
python3 -m pytest -q \
  src/planner_common/test/test_contracts.py \
  src/planner_llm/test/test_planner_engine.py \
  src/planner_llm/test/test_supervisor.py \
  src/planner_llm/test/test_skill_registry.py \
  src/nao_orchestrator/test/test_nao_orchestrator_intent_rules.py \
  src/nao_orchestrator/test/test_planner_gate.py
```

### `dialogue_manager`

```bash
python3 -m pytest -q \
  test/test_chatbot_client.py \
  test/test_manager_node.py
```

## 7. Scenario acceptance suite

| Scenario | Expected route | Expected execution | Expected speech |
|---|---:|---:|---|
| “scan the room for a person” | execution | scan | person-focused result or honest no-person result |
| “look around and tell me what you see” | execution | scan | grounded current scene summary |
| “did you see people in the last scan?” | knowledge_query | no new scan | answer from last scan evidence |
| “what was the person id?” | knowledge_query | no new scan | ID if known, otherwise honest unknown |
| “is it the same person as before?” | knowledge_query | no new scan unless asked | do not claim sameness without evidence |
| “stand up and then look left” | execution | two-step plan | completion once, no duplicate ack |
| “help” | dialogue | none | dialogue response only |
| planner step failure | none/new clarify | no hidden failure | sanitized failure text or clarification |

## 8. PR discipline

For each PR:

1. Update tests first or with code.
2. Keep scope to one phase unless explicitly directed.
3. Include before/after scenario notes.
4. Include exact branches and files changed.
5. Run the relevant subset of tests.
6. Refresh GitNexus index after significant code changes if working locally:

```bash
tools/knowledge/post_commit_refresh.sh || true
```

## 9. First PR recommendation

Start with **Issue 1 / Phase 1 Option A**.

Reason: it is the smallest change with the highest chance of removing the current “planner completion becomes a new execution route” failure.

Then implement **Issue 2 structured scan evidence**. Once scan evidence is typed, prompt hardening becomes much more effective because the model is no longer guessing from loose prose.
