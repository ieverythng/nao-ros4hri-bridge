---
name: robot-runtime-performance-review
description: Use when reviewing or debugging the live NAO ROS4HRI container, robot stack logs, grounded_context/KnowledgeCore behavior, interaction_sim perception, planner/chatbot routing, duplicate speech, fake-skill validation runs, or overall runtime performance against the TFM validation expectations. Produces evidence-backed findings, a score, and concrete next probes/fixes.
---

# Robot Runtime Performance Review

## Objective

Audit a live or recently run NAO ROS4HRI stack as a reviewer, not a guesser.
Use container logs, ROS graph state, topic samples, parameters, prompt/contract
code, and validation traces to decide whether the robot behaved as expected.

Primary P0 question:

> When an object or relation is added in interaction_sim/KnowledgeCore, does the
> grounded context expose it to chatbot/planner clearly enough for dialogue and
> execution turns to use it?

## Review Discipline

Borrow the SQL review skill's posture:

- Read raw evidence, not summaries only.
- Finish one seam before jumping to another.
- Separate fact absence from model failure.
- Suggest fixes with exact file/param/topic references.
- Do not pad findings. If the evidence is inconclusive, say what probe is missing.

## Fast Start

From the repo root, run:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/collect_runtime_snapshot.py \
  --container nao_ros2 \
  --since 30m \
  --out /tmp/nao_runtime_snapshot.json
```

Then inspect the JSON and the live source files relevant to any flagged seam.
Add `--sample-topics` only when you need one-shot ROS topic payloads; sparse
topics can slow the loop. Add `--include-heavy-topics` only when raw detector
messages are the target of the diagnosis.

## Evidence Sources

Use the minimum set that answers the question:

- Docker status and recent logs from `nao_ros2`.
- ROS nodes, topics, selected params, and one-shot topic samples.
- `/scene/summary`, `/detected_objects`, `/humans/persons/tracked`,
  `/humans/faces/tracked`, `/kb/*`, `/chatbot_llm/turn_trace`, and planner
  dialogue/feedback topics when available.
- Source files for the suspect seam:
  - `src/nao_scene_grounding/nao_scene_grounding/scene_grounding_node.py`
  - `src/chatbot_llm/chatbot_llm/node_impl.py`
  - `src/chatbot_llm/chatbot_llm/planner_handoff.py`
  - `src/chatbot_llm/chatbot_llm/prompt_builders.py`
  - `src/planner_common/planner_common/contracts.py`
  - `src/nao_orchestrator/nao_orchestrator/orchestrator.py`
  - `src/nao_chatbot/nao_chatbot/stack_launch.py`
- TFM fake-skill validation plan:
  `docs/plans/tfm_fake_skill_validation_suite_codex_plan.md`.

## Review Lenses

Run the lenses in this order. Skip lenses that do not apply.

### 1. Grounding And KB Freshness

Check:

- `nao_scene_grounding` params:
  - `knowledge_lifespan_sec`
  - `knowledge_refresh_interval_sec`
  - `local_stale_after_sec`
  - `fallback_match_distance_px`
  - `fallback_match_max_age_sec`
  - `min_detection_score`
- Whether `/scene/summary` contains the expected object.
- Whether KnowledgeCore logs show the expected predicate update.
- Whether KnowledgeCore removes the fact before the next user turn.
- Whether the entity id churns across frames for the same visible object.
- Whether relation predicates such as `dbp:name`, `dbp:color`, `oro:isOn`,
  `oro:isAt`, `oro:contains`, or `foaf:knows` reach `grounded_context`.

Finding rule:

- If the fact never enters KB, blame perception/KB mutation.
- If the fact enters KB but expires/renames before the turn, blame grounding
  freshness/identity stability.
- If the fact is in `grounded_context` but the answer ignores it, blame prompt
  or model behavior.

### 2. HRI Person Stability

Check:

- `/humans/persons/tracked`
- `/humans/faces/tracked`
- face detector warnings about skipped frames
- KnowledgeCore person visibility updates/deletes
- whether person ids churn between adjacent turns

Do not treat HRI person-manager churn as chatbot hallucination until the tracked
person/faces topics and KB facts prove the person was stable.

### 3. Chatbot Route And Prompt Contract

Check:

- Response route: `dialogue`, `knowledge_query`, or `execution`.
- Whether dialogue-only turns were sent to planner.
- Whether execution-looking turns preserved complete goal text.
- Whether `grounded_context` is the only world-state input.
- Whether prompt builders keep the canonical YAML identity and append only
  structural stage/task instructions.

Flag duplicated or contradictory route policy if Python templates and YAML prompt
pack both carry policy in different words.

### 4. Planner/Orchestrator/Speech Exactly-Once

Check:

- `chatbot_llm -> nao_orchestrator -> planner_llm` planner ingress.
- `planner_llm:/planner/dialogue_act -> nao_orchestrator relay -> dialogue_manager`.
- `report_result` and `notify_completion` interactions.
- one semantic event should produce one spoken utterance.

Classify duplicate speech by source: chatbot ack, planner dialogue act,
`report_result`, completion notification, or replayed planner act.

### 5. Fake-Skill And TFM Validation Readiness

Compare live behavior with the TFM validation plan:

- route correctness
- expected plan steps
- fake-skill dispatch path
- success/failure/ambiguity handling
- replanning or safe failure
- final response wording
- trace completeness

Use fake-skill validation when perception noise is not the target of the test.
Use full user-turn validation when chatbot routing or grounding is the target.

## Severity Bands

Report findings under these bands:

- 🔴 **Critical** — speech duplication, raw planner/model leakage, execution
  falsely reported as success, or facts present in grounded context but
  consistently ignored in user-visible behavior.
- 🟠 **Grounding/KB instability** — facts expire too quickly, entity ids churn,
  scene summary missing expected objects, relation predicates not projected.
- 🟡 **Route/prompt ambiguity** — dialogue vs execution confused, duplicated
  prompt policy, incomplete goal_text or intent_sequence.
- 🔵 **Runtime/performance pressure** — skipped frames, slow materialisation,
  LLM/preflight/connectivity warnings, overloaded detector streams.
- 🟣 **Observability gaps** — missing traces, no topic sample, unclear source of
  the fact, insufficient logs to separate absence from model ignoring.

## Output Format

Use this structure:

```text
Runtime Review — <container or run id>

Score: X/10

🔴 Critical
1. <finding with evidence> → <fix/probe>

🟠 Grounding/KB instability
1. <finding with timestamps/topics/params> → <fix/probe>

Checks passed
- <short evidence-backed positives>

Next probes
- <one command or action per probe>

Summary: N findings — 🔴 a · 🟠 b · 🟡 c · 🔵 d · 🟣 e
```

If no material findings exist, reply with `PASS` plus the score and evidence
sample that justified it.

## Automation Loop

For iterative live sessions:

1. Collect runtime snapshot.
2. Score against the review lenses.
3. Patch only the responsible seam.
4. Run focused tests/build.
5. Sync/rebuild container if needed.
6. Restart stack when params or launch wiring changed.
7. Re-run the same snapshot and compare score.

Do not claim a launch/default fix is live until the running node has restarted
and `ros2 param get` confirms the new value.
