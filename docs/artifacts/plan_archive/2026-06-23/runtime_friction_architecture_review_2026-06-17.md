# Runtime Friction Architecture Review

Generated: 17 June 2026, CEST.

Scope: NAO ROS4HRI runtime friction after the latest 8.x validation runs, with a targeted comparison against local Aily repositories for prompt/context/session/evaluation patterns. This is a source-and-architecture report, not a fresh live container score. Live runtime proof still needs the robot-runtime questionnaire after any source changes are rebuilt.

## Executive Summary

The stack is close, but the remaining friction is concentrated in three seams:

| Seam | Symptom | Likely owner | Main fix direction |
|---|---|---|---|
| KB subject recall and relation ingestion | Stable KB facts can exist, but chatbot may answer that it has no information, or may miss relations such as name/color/isOn when similar detector objects crowd the context. | `chatbot_llm`, `kb_skills`, `planner_common`, `nao_scene_grounding` | Build a canonical subject-focused grounded context packet and add a direct subject lookup path for knowledge-query turns. |
| Exactly-once speech | Double utterances returned after being absent for a long time. The likely causes are response-route mismatch, chatbot acknowledgement plus later planner/report speech, or missing orchestrator-side duplicate suppression. | `chatbot_llm`, `nao_orchestrator`, `dialogue_manager` | Treat speech as a semantic event with an id, classify source, and suppress duplicates across chatbot ack, planner dialogue, completion, and report_result. |
| Prompt/session drift under long context | Long runs and detector churn can pollute history and grounded context; prompt edits risk fixing one scenario while regressing holdouts. | `chatbot_llm`, prompt packs, runtime-review scripts | Promote Aily-style scenario suites and SkillOpt train/holdout gates before accepting prompt mutations. |

The strongest architectural move is to stop asking the LLM to infer from an amorphous mix of natural language snapshot, scene summary, history, and planner context. Instead, every LLM-facing turn should receive one canonical `grounded_context` object, plus an optional short natural-language view derived from that same object. Chatbot and planner should consume the same shape, with role-specific projections only after the canonical packet is built.

## Evidence Read

| Source | What it says | Why it matters |
|---|---|---|
| `docs/plans/ISSUES_BEFORE_17-06-26.html` | Latest tracker records an 8.6 success-path baseline, remaining caps around natural-chat KB mutation proof, failure/fail-once recovery, non-mutating KB subject recall, cold-start `report_result`, and spatial proof. | The current task should not be framed as a full rewrite. It is a final-seam hardening pass. |
| `src/chatbot_llm/chatbot_llm/turn_engine.py` | Planner mode uses a response stage that can return `verbal_ack`, `route`, and `user_intent`, then route inference can override dialogue to execution if the acknowledgement implies execution. | This is a high-risk seam for duplicate speech and route mismatch because the same LLM response both speaks and routes. |
| `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py` | KB query rows are formatted into deterministic natural-language summaries, scene memory, and compact grounded-context text. | It is helpful, but it is still text-first for chatbot response. Relation fidelity needs a structured subject packet. |
| `src/chatbot_llm/chatbot_llm/planner_handoff.py` | Planner handoff builds `knowledge_snapshot`, `scene_summary`, and `state_t0`, then projects to compact `grounded_context`. People are separated from objects. | This is the right direction, but chatbot response should consume the same canonical packet, not a separately formatted text-only snapshot. |
| `src/chatbot_llm/chatbot_llm/planner_request_adapter.py` | Planner requests include bounded dialogue history, normalized intents, scene targets, and normalized grounded context. Assistant history is sanitized to extract acks from JSON. | Good seam hygiene. The missing piece is making subject selection and relation requirements explicit before planner handoff. |
| `/Users/juanbendek/repos/aily-context/README.md` | Aily semantic-layer changes are validated with targeted scenario suites, Langfuse traces, expected tables/tools, and replayable test selectors. | NAO needs the same discipline: every prompt/grounding fix should land with a replayable case and a trace artifact. |
| `/Users/juanbendek/repos/aily-mcp/aily-mcp/.claude/skills/langfuse-autoimprove/SKILL.md` | Aily trace improvement loop diagnoses production traces, edits only justified descriptions, validates integration cases, and writes a report. | This is directly portable as a robot trace improvement loop over `/chatbot_llm/turn_trace`, `/planner/request`, `/planner/execution_feedback`, and speech topics. |
| `/Users/juanbendek/repos/aily-mcp/aily-mcp/packages/internal/memory/README.md` | Memory storage separates agent-selected content from app-owned identity/provenance, and supports programmer-pinned scopes. | NAO should avoid letting the LLM own KB/provenance scope. Runtime nodes should pin source, freshness, identity, and object scope. |
| `/Users/juanbendek/repos/aily-super-agent/tests/unit/benchmarks/test_evaluation_evidence.py` | Evaluation evidence flattens ordered tool calls, errors, skills used, steps, and fallback sources. | NAO should produce a similar ordered evidence bundle per turn: speech input, chatbot route, KB facts used, planner steps, skill results, and speech events. |

## Root Cause Model

### 1. KB facts are present but not always reachable by the LLM

The tracker already shows a non-mutating KB question that safely avoided mutation but answered as if no facts existed. That points to a retrieval/projection issue rather than a pure model issue. The likely pattern is:

| Stage | Current risk |
|---|---|
| `/kb/query` | Query patterns may be broad enough for visibility but not subject-specific enough for "what do you remember about X?" |
| Knowledge snapshot text | Relations may be summarized as prose, causing subject/object/predicate fidelity loss. |
| `grounded_context` projection | Compact entities are present, but relation facts may not be weighted or grouped by mentioned subject. |
| Chatbot response prompt | It may answer from the visible scene summary instead of running a direct subject recall. |

Recommendation: add a subject-focused KB projection for knowledge-query turns. If the user mentions an entity id, normalized label, name, color, or known alias, the chatbot should request or build a `subject_context` block keyed by canonical entity id:

```json
{
  "schema_version": "grounded_context_v3",
  "query_focus": {
    "kind": "subject_recall",
    "raw_mention": "probe cup",
    "canonical_id": "codex_probe_cup",
    "match_confidence": 0.91,
    "match_source": "kb_alias_and_scene"
  },
  "entities": [
    {
      "id": "codex_probe_cup",
      "kind": "object",
      "label": "probe cup",
      "class": "Cup",
      "visible": true,
      "freshness": {"last_seen_sec": 1781672100.1, "source": "interaction_sim"}
    }
  ],
  "relations": [
    {"subject": "codex_probe_cup", "predicate": "rdf:type", "object": "Cup"},
    {"subject": "codex_probe_cup", "predicate": "dbp:name", "object": "TITAS"},
    {"subject": "codex_probe_cup", "predicate": "dbp:color", "object": "gold"},
    {"subject": "codex_probe_cup", "predicate": "oro:isOn", "object": "codex_table"}
  ],
  "excluded_detector_noise": [
    {"id": "detected_cup_17", "reason": "lower-confidence duplicate of stable KB subject"}
  ]
}
```

The natural-language block can still exist, but it should be generated from this object, never separately authored.

### 2. Duplicate utterance is probably a semantic-event problem, not only a prompt problem

The current chatbot turn engine can produce a spoken `verbal_ack` and a route in the same response. If route is later repaired to execution, the spoken ack may already sound like a task commitment. Then planner dialogue, `report_result`, or completion notification may speak again. This is powerful when correct, but brittle under mismatch.

Exactly-once speech should be enforced structurally:

| Semantic event | Allowed speaker | Suppression key |
|---|---|---|
| execution accepted acknowledgement | `dialogue_manager` from chatbot ack, once per `turn_id` or `goal_id` | `ack:{turn_id}:{goal_id}` |
| planner clarification/help | `dialogue_manager` via orchestrator relay | `planner_dialogue:{goal_id}:{act_id}` |
| progress report | chatbot-authored wording from execution feedback | `progress:{goal_id}:{step_id}` |
| terminal report_result | chatbot-authored report from structured results | `completion:{goal_id}:{plan_id}:{plan_version}` |
| fallback failure | planner/orchestrator failure act, chatbot wording if available | `failure:{goal_id}:{failure_code}` |

Recommendation: add a small speech-event ledger in `nao_orchestrator` or the dialogue relay seam, not inside the LLM. The LLM can propose text; the deterministic seam decides whether that semantic event has already been spoken.

### 3. Detector churn can poison object identity and KB retrieval budget

Detector-derived objects should not outrank stable interaction-sim or explicit KB objects. The tracker already treats detector validation as separate from cool-profile scoring. The runtime architecture should make that separation visible in the context packet.

Recommendation: every grounded entity should carry `identity_tier`:

| Tier | Meaning | LLM priority |
|---|---|---|
| `explicit_kb` | User/system inserted stable KB object with predicates. | Highest for recall and execution targets. |
| `interaction_sim` | Simulator-authored object or pose source. | High for validation fixtures and spatial proof. |
| `scene_grounding_stable` | Detector object with stable matched id across freshness window. | Medium. |
| `detector_ephemeral` | Raw or recently churned detector id. | Low; mention only as uncertain observation. |

This gives the chatbot a deterministic way to ignore `detected_blueberry_*` churn when the user asks about a stable `codex_probe_cup`.

## Aily Patterns To Import

| Aily pattern | Observed in | NAO adaptation |
|---|---|---|
| Scenario suites as first-class artifacts | `aily-context` semantic-layer `integration_suite.yaml` and `experimental_suite.yaml` | Add `docs/runtime_scenarios/` or `src/nao_chatbot/test/runtime_scenarios/` with YAML cases for dialogue, KB recall, execution, recovery, duplicate speech, and detector stress. |
| Trace feedback loop before prompt edits | `langfuse-autoimprove` | Require a runtime trace bundle before changing chatbot/planner prompt packs. |
| Expected tools / expected tables | Aily benchmark tests | Add expected `route`, `planner_request_published`, `expected_kb_subjects`, `expected_plan_steps`, and `expected_speech_events`. |
| Ordered evidence bundles | `test_evaluation_evidence.py` | Save per-turn evidence as JSON: input, trace, grounded context, planner request, plan, execution feedback, speech events. |
| Programmer-pinned scope | `aily-mcp-memory` | Pin KB provenance, scene source, interaction id, and subject scope in runtime nodes, not model output. |
| Minimal but complete prompt altitude | Aily prompt-engineering skill | Keep prompts as contracts and examples, not brittle per-case patch notes. |
| Holdout cases | SkillOpt loop | Keep favorite movie, future-action, wave-particle, reflective follow-up, and no-mutation questions as holdouts for every prompt mutation. |

## Proposed Target Architecture

```text
speech / rqt_chat
  -> dialogue_manager
  -> chatbot_llm
       -> route classifier
       -> subject/context resolver
       -> canonical grounded_context packet
       -> natural response or planner request
  -> nao_orchestrator planner gate
       -> duplicate-speech semantic ledger
       -> planner admission and execution lineage
  -> planner_llm
       -> executable plan over canonical ids and relation facts
  -> nao_orchestrator
       -> deterministic validation, fake/real skill dispatch, feedback
  -> chatbot_llm wording service
       -> completion/failure/progress text from structured evidence only
  -> dialogue_manager speech
```

The important change is the `subject/context resolver` before both dialogue answers and planner handoff. This resolver should not be an LLM. It should be a deterministic projection layer that picks likely subjects, relation groups, freshness, source tier, and detector/noise status.

## SkillOpt Setup For Prompt Changes

Do not edit prompt packs directly for the next pass unless this setup is locked first.

| Required input | Proposed value |
|---|---|
| `target_artifact` | `src/chatbot_llm/config/chat_prompt_pack.yaml`, `src/chatbot_llm/chatbot_llm/prompt_builders.py`, and only if needed `src/planner_llm/config/planner_prompt_pack.yaml`. |
| `objective` | Reduce KB subject-recall misses and duplicate utterances without regressing execution admission or social dialogue. |
| `train_set` | Recent failing KB relation/subject cases, duplicate-utterance traces, similar-object detector churn case, and maximal kitchen-cup-to-person wording. |
| `holdout_set` | Future-action "Could we navigate to the probe cup later?", "Wave-particle equation?", "What is your favorite movie?", reflective "How many directions did you move your head?", and non-mutating "Is cup_1 on table_1?" |
| `acceptance_gate` | Train failures improve; holdouts preserve route, planner handoff flag, and exactly-one speech; prompt diff is at most three focused edits per iteration. |

Baseline table for the first iteration:

| Case | Expected | Current expected status |
|---|---|---|
| Subject recall by id/name/color | Answer from KB relations if `/kb/query` has facts. | Degraded in tracker for `codex_arch_marker`. |
| Similar objects in view | Prefer stable explicit KB object over detector duplicate. | Suspected degraded under detector churn. |
| Duplicate utterance | One semantic event produces one speech event. | Regressed in latest user report; source classification pending. |
| Future action | Dialogue only, no planner request. | Holdout previously passed. |
| Favorite movie | Dialogue only, no planner request. | Holdout previously passed. |
| Wave-particle | Dialogue only, no `wave_greet`. | Holdout previously passed. |

## Concrete Implementation Tracks

### Track A: Grounded Context V3 Projection

Goal: make KB subject recall and similar-object disambiguation deterministic before the LLM sees the turn.

Changes:

| Package | Change |
|---|---|
| `kb_skills` | Add or expose a subject-focused query helper for ids, labels, aliases, names, and relation predicates. |
| `chatbot_llm` | Build `subject_context` for knowledge-query turns before response generation. |
| `planner_common` | Extend compact projection to include relation groups and identity tier while preserving current `entities` compatibility. |
| `nao_scene_grounding` | Mark detector entities with freshness, source, confidence, and identity tier; avoid allowing ephemeral detector ids to crowd stable KB fixtures. |

Acceptance tests:

| Test | Expected |
|---|---|
| Add `codex_probe_cup rdf:type Cup`, `dbp:name TITAS`, `dbp:color gold`, `oro:isOn codex_table`; ask name/color. | Chatbot answers TITAS/gold/table from relations. |
| Add two cups plus detector cup churn. | Stable requested cup remains selected by canonical id or alias. |
| Ask general "what can you see?" | People and objects are separate, stable facts summarized without overclaiming detector noise. |

### Track B: Exactly-Once Speech Ledger

Goal: prevent duplicate semantic utterances without making prompts carry all responsibility.

Changes:

| Package | Change |
|---|---|
| `nao_orchestrator` | Introduce a bounded semantic speech-event ledger keyed by `turn_id`, `goal_id`, `plan_id`, `step_id`, and event kind. |
| `chatbot_llm` | Include `speech_event_kind` in service responses where possible, or let orchestrator classify completion/progress/failure source. |
| `dialogue_manager` | No ownership change; continue as speech owner. Add only tests or docs if the relay contract needs clarification. |
| runtime-review scripts | Count `/debug/nao_say/speech` and `/dialogue_manager/closed_captions` events per injected turn. |

Acceptance tests:

| Case | Expected |
|---|---|
| Simple execution with ack and report_result. | One ack and one final report at most; no repeated final report. |
| Planner clarification. | One clarification, no chatbot extra ack for the same semantic event. |
| Report_result service failure. | One safe fallback, no crash, no duplicate completion. |

### Track C: Runtime Scenario Suite

Goal: make the robot stack behave more like Aily's evaluation workflow.

Create a durable scenario file, for example `docs/runtime_scenarios/nao_runtime_regression_suite.yaml`, with fields:

```yaml
cases:
  - id: kb_subject_recall_probe_cup
    input: "What is the name and color of the probe cup?"
    setup_facts:
      - "codex_probe_cup rdf:type Cup"
      - "codex_probe_cup dbp:name TITAS"
      - "codex_probe_cup dbp:color gold"
    expected_route: knowledge_query
    expected_planner_request: false
    expected_kb_subjects:
      - codex_probe_cup
    expected_speech_events:
      - kind: answer
        count: 1
    forbidden:
      - kb_mutation
      - duplicate_speech
```

Runtime-review scripts should load these cases and emit a JSON evidence bundle plus a small HTML report.

### Track D: Detector Churn Isolation

Goal: stop detector churn from lowering KB-facing behavior quality.

Changes:

| Package | Change |
|---|---|
| `nao_scene_grounding` | Add detector churn metrics: id changes per label/window, duplicate label clusters, stable-match count, and confidence distribution. |
| `planner_common` | Preserve `identity_tier` and `source` in compact `grounded_context`. |
| `chatbot_llm` | Prompt says stable KB/interaction-sim entities outrank ephemeral detector observations; code projection should already enforce this before prompt. |

Acceptance tests:

| Case | Expected |
|---|---|
| Three detector cups plus one explicit KB cup. | Explicit KB cup remains the answer target when user names it. |
| General visibility under churn. | Chatbot says it sees uncertain cups/objects without inventing stable identity. |
| Planner execution target under churn. | Planner receives canonical id or asks clarification. |

## Immediate Probe Plan

Run these before changing prompts:

1. Collect snapshot with topic samples:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/collect_runtime_snapshot.py \
  --container nao_ros2 \
  --since 30m \
  --sample-topics \
  --out /tmp/nao_runtime_snapshot_kb_speech_20260617.json
```

2. Run a small active questionnaire with a stable KB object and duplicate-speech counters:

```bash
python3 .codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py \
  --container nao_ros2 \
  --case-set smoke \
  --out /tmp/nao_active_questionnaire_kb_speech_20260617.json
```

3. Add a targeted direct architecture sweep for:

```text
subject recall by id/name
non-mutating relation question
simple execution exactly-once speech
report_result exactly-once speech
duplicate detector object stress if detector profile is enabled
```

4. Only after the evidence bundle exists, run the SkillOpt loop against chatbot prompt/code seams.

## Recommended File-Level Next Steps

| Priority | File | Action |
|---|---|---|
| P0 | `src/chatbot_llm/chatbot_llm/node_impl.py` | Confirm whether knowledge-query turns can trigger subject-specific KB query patterns from the current user text. |
| P0 | `src/chatbot_llm/chatbot_llm/knowledge_snapshot.py` | Add structured relation grouping and subject focus rather than only text summaries. |
| P0 | `src/chatbot_llm/chatbot_llm/turn_engine.py` | Separate route decision from speech commitment more explicitly; avoid allowing a repaired execution route to reuse an already-spoken dialogue ack without event classification. |
| P0 | `src/nao_orchestrator/nao_orchestrator/orchestrator.py` | Add duplicate semantic speech suppression around planner dialogue, progress, completion, and report_result. |
| P0 | `.codex/skills/robot-runtime-performance-review/scripts/run_active_questionnaire.py` | Count and classify speech events per turn. |
| P1 | `src/planner_common/planner_common/contracts.py` | Preserve relation groups, identity tier, freshness, and source in compact grounded context. |
| P1 | `src/nao_scene_grounding/nao_scene_grounding/scene_grounding_node.py` | Add churn metrics and stable/ephemeral identity tiering. |
| P1 | `docs/plans/ISSUES_BEFORE_17-06-26.html` | After implementation, update current score caps and proof artifacts. |

## Score Impact Forecast

| Fix | Expected score impact |
|---|---|
| Subject-focused KB recall and relation grouping | Removes the 7/10 cap for interaction-sim object-add grounding when facts are present but ignored. |
| Exactly-once speech ledger | Removes the 6/10 cap risk for duplicate speech. |
| Runtime scenario suite with trace evidence | Makes future 8.8 to 9.0 claims defensible and repeatable. |
| Detector churn isolation | Keeps detector-profile failures from contaminating cool-profile KB validation. |
| Failure/fail-once speech suite | Needed before claiming 9.0+. |

## Decision

Do not start with another broad prompt rewrite. Start with evidence and structure:

1. Add runtime scenario/evidence checks for KB subject recall and duplicate speech.
2. Implement subject-focused grounded context projection.
3. Implement exactly-once speech-event suppression.
4. Run SkillOpt only for small prompt changes that remain necessary after deterministic seams are fixed.
5. Rebuild, restart, and rerun the same scenario suite before changing the score.
