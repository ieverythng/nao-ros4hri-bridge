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

For an active E2E questionnaire pass, run:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set smoke \
  --out /tmp/nao_active_questionnaire.json
```

The questionnaire must include an interaction_sim/KnowledgeCore-style object
insertion probe when the KB seam is under review. The preferred probe is:

1. Add a stable object id and predicates through `/kb/revise` when that service
   is available, for example `codex_probe_cup rdf:type Cup`,
   `codex_probe_cup dbp:name TITAS`, and `codex_probe_cup dbp:color gold`.
2. Verify the same facts through `/kb/query`.
3. Ask the chatbot through ROS4HRI speech ingress what the object's name/color is.
4. Check the `GROUNDED_CONTEXT` trace for the object id and predicates before
   blaming the model. If `/kb/revise` or `/kb/query` is unavailable, record that
   explicitly as an observability or launch seam finding.

Live probing requirement:

- If the container is running, actively probe it. Do not rely on historical logs
  alone for a full review.
- Confirm node visibility with `docker exec <container> ... ros2 node list`.
- Confirm critical live parameters with `ros2 param dump` or `ros2 param get`,
  especially chatbot token budget, KB query budget, scene-grounding freshness,
  orchestrator execution mode, and fake-skill mode.
- Sample current topics when a questionnaire item depends on present state:
  `/scene/summary`, `/chatbot_llm/turn_trace`, `/planner/request`,
  `/planner/execution_feedback`, `/nao_orchestrator/planner_dialogue_act`,
  `/debug/nao_say/speech`, and `/speech`.
- When a user reports a live failure, align log timestamps with direct topic and
  parameter probes before assigning blame to planner, chatbot, KB, or execution.

Active questionnaire requirement:

- For a full runtime pass, inject at least one turn from each applicable
  questionnaire category through the ROS4HRI speech ingress instead of only
  reading historical logs.
- Prefer the rqt_chat/dialogue_manager input seam used by the active launch:
  `/nao_chatbot/humans/voices/anonymous_speaker/speech`
  (`hri_msgs/msg/LiveSpeech`) plus
  `/nao_chatbot/humans/voices/tracked` (`hri_msgs/msg/IdsList`). Verify
  `ros2 topic info -v` shows a `dialogue_manager` subscription before injecting
  turns. Use another topic only when the active launch explicitly exposes it.
- Use direct `/planner/request` publication only for planner-isolated probes.
  Mark those probes as planner-only, because they bypass chatbot routing.
- After each injected turn, collect `/chatbot_llm/turn_trace`,
  `/planner/request`, `/planner/execution_feedback`,
  `/nao_orchestrator/planner_dialogue_act`, `/debug/nao_say/speech`, and
  recent container logs before scoring the case.
- If a code or prompt fix was made before review, rebuild and restart the stack
  first, then verify live params and one trace from the restarted nodes before
  claiming the fix is live.

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

Interaction_sim object rule:

- Treat simulator-authored objects and relations as the primary stability probe.
  They should not depend on detector confidence or HRI person-manager stability.
- For interaction_sim objects, first verify the explicit simulator object id and
  predicates in KnowledgeCore logs or `/kb/query`, then verify the same facts in
  `grounded_context`, then judge chatbot/planner behavior.
- If detector entities such as `detected_blueberry_*` crowd the snapshot, report
  the KB query budget and whether stable simulator ids such as `cup_xslil`,
  `apple_*`, or `phone_*` are still present in the rendered context.
- Do not classify a simulator object-add miss as perception noise unless the
  object never enters KnowledgeCore or `/scene/summary`.

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

### 6. E2E Operational Questionnaire

When the user asks for a full runtime pass, score these scenarios explicitly.
Use the live stack when the target is routing, grounding, or speech. Use fake
skills when the target is deterministic execution, replanning, or failure policy.

1. Simple dialogue turn:
   - Prompts such as "Hey!", "How are you?", and "What is your favourite color?"
   - Expected: route stays dialogue, planner handoff is false, exactly one speech
     event is emitted, and no skill feedback appears.
2. KB query dialogue turn:
   - Prompts such as "What can you see?", then add interaction_sim objects, then
     ask "What can you see now?" or "What is the id/name/color of ...?"
   - Expected: fresh simulator object facts appear in `grounded_context` on the
     next user turn, stable ids/relations are preserved, and the answer uses
     those facts without requiring a second confirmation turn.
   - Required probe: insert or ask the operator to insert one stable simulator/KB
     object with at least `rdf:type`, `dbp:name`, and `dbp:color`; verify
     KnowledgeCore mutation/query evidence separately from chatbot wording.
3. Simple skill execution:
   - Prompts such as "Move your head to the right" or "Wave at me."
   - Expected: chatbot emits one acknowledgement, orchestrator dispatches one
     skill, planner feedback is truthful, and completion/report speech is not
     duplicated.
4. Composite skill execution:
   - Prompts such as "Move your head in all directions" or "Navigate to the
     phone in the scene and tell me what else you see."
   - Include ordered multi-object navigation when at least three grounded
     objects are available: "Now walk to every object, let me know when you are
     there and then walk to the next!"
   - Expected: planner produces multiple ordered steps, orchestrator preserves
     step evidence, `report_result` receives a filled or derivable summary, and
     final speech reflects the whole chain rather than only the last step.
5. Simple fake-skill scenario execution:
   - Run one-skill cases under all-success, all-failure, fail-once, alternating,
     and random fake modes.
   - Expected: fake payloads include `metadata.fake=true`, result mode, and
     stable summary text; failures cause clarification, replanning, or safe stop
     according to the plan policy.
6. Composite fake-skill scenario execution:
   - Run multi-step cases under the same fake modes.
   - Expected: successful steps are not repeated unnecessarily, failed steps are
     reported with structured feedback, user clarification is routed through
     chatbot wording, and exactly one semantic speech event is emitted per stage.

Questionnaire scoring:

- Mark each scenario as pass, degraded, fail, or not run.
- A fail in simple dialogue, duplicate speech, raw planner leakage, or false
  execution success caps the total runtime score at 6/10.
- A fail in interaction_sim object-add grounding caps the score at 7/10 unless
  logs prove the fact never reached KnowledgeCore.
- A fail only in detector/HRI variability should not cap the interaction_sim
  grounding score; report it under runtime/performance pressure instead.

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

E2E questionnaire
- Simple dialogue: <pass/degraded/fail/not run>
- KB query dialogue: <pass/degraded/fail/not run>
- Simple skill execution: <pass/degraded/fail/not run>
- Composite skill execution: <pass/degraded/fail/not run>
- Simple fake-skill scenarios: <pass/degraded/fail/not run>
- Composite fake-skill scenarios: <pass/degraded/fail/not run>

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
   Use a bounded SkillOpt-style iteration before accepting prompt-pack or review
   workflow wording changes.
4. Run focused tests/build.
5. Sync/rebuild container if needed.
6. Restart stack when params or launch wiring changed.
7. Re-run the same snapshot and compare score.

Do not claim a launch/default fix is live until the running node has restarted
and `ros2 param get` confirms the new value.
