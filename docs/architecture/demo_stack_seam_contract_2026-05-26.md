# Demo Stack Seam Contract (2026-05-26)

## Scope

This document captures the contract-critical routing seams for the live demo
stack (`chatbot_llm`, `planner_llm`, `nao_orchestrator`, `dialogue_manager`,
`fake_skills`) and the expected behavior boundaries for ROS4HRI-safe operation.

## Contract-Critical Ownership

| Concern | Owner | Must Not |
| --- | --- | --- |
| User-facing wording and TTS path | `dialogue_manager` + `chatbot_llm` | `planner_llm` or `nao_orchestrator` speaking planner dialogue directly |
| Task planning and supervision | `planner_llm` | Emitting robot speech actions as mixed executable plan steps |
| Deterministic execution | `nao_orchestrator` | Re-owning dialogue decisions or bypassing planner/dialogue seams |
| KB boundary | `kb_skills` | Ad-hoc direct KB transport shortcuts in unrelated nodes |
| Scene grounding bridge | `nao_scene_grounding` | Planner-side detector coupling |

## Routing Seams (Authoritative)

### A) Planner request ingress

```text
user -> dialogue_manager -> chatbot_llm -> nao_orchestrator(planner gate) -> planner_llm
```

- `chatbot_llm` routes execution-mode turns toward planner request transport.
- `nao_orchestrator` planner gate validates/supersedes and forwards accepted
  planner requests.

### B) Planner dialogue act egress

```text
planner_llm -> /planner/dialogue_act -> dialogue_manager -> chatbot_llm(default) -> TTS
```

- `planner_llm` publishes structured dialogue acts only.
- `dialogue_manager` owns planner-act consumption and wording policy.
- `dialogue_manager` delegates planner dialogue wording to `chatbot_llm` by
  default.
- `nao_orchestrator` may observe planner dialogue acts for gate cleanup but is
  not a speech owner.

## Duplicate-Speech Guardrails

- `dialogue_manager` ignores planner `acknowledge` dialogue acts to avoid
  duplicate speech when an execution acknowledgement already happened.
- Planner models are prevented from mixing speech steps with executable skill
  steps in a single plan.
- Scan completion speaking should be chatbot-owned (`scan_report_after_success=false`
  in demo/sim profiles) unless explicitly testing direct execution speech.

## KB Visibility and Proactive Behavior

- Visibility-only questions default to `knowledge_query` unless the user
  explicitly asks for a fresh scan/action.
- Chatbot wording should answer current KB-grounded state and proactively offer
  an optional fresh scan.

## Fake Skills in Demo Contract

`scan` is **not** a fake skill in this stack contract; it is a first-party
runtime skill exposed by `nao_orchestrator` (`/skill/scan`).

### Runtime controls

- `active_scenario_id`
- `global_mode`: `scenario|always_success|always_fail|every_other|random_seeded`
- `random_failure_prob`
- `mode_overrides_json`

### Scenario precedence

```text
request scenario override > per-skill override mode > global_mode policy > scenario/default mode
```

### AB alignment

- AB=0: ROS/interface primitives and action/service/topic seams.
- AB=1: runtime callable skills (`scan`, `find_object`, `navigate_to`, `walk_to`,
  `wave_greet`, `report_result`, `look_at`, `perform_motion`).
- AB>=2: decomposition proposals, non-runtime-callable unless promoted.
- Planner can intentionally include `ask_user` / `ask_clarification` skill steps;
  orchestrator executes the prompt utterance and returns blocking feedback with
  `needs_user_input=true` so planner supervision can wait for user input.

## Operator Validation Checklist

1. Check planner dialogue path:
   - planner emits `/planner/dialogue_act`
   - dialogue_manager consumes it
   - chatbot produces final wording
2. Confirm no duplicate utterances for visibility questions and failure
   completions.
3. Confirm fake-skill scenario selection is reflected by `/fake_skill_server`
   parameters.
4. Confirm interaction traces include planner request/output/feedback,
   planner-dialogue acts, chatbot turn traces, and fake-skill events.

## Source-of-Truth Files

- `src/planner_llm/planner_llm/planner_node.py`
- `src/planner_llm/planner_llm/planner_engine.py`
- `src/dialogue_manager/dialogue_manager/manager_node.py`
- `src/dialogue_manager/dialogue_manager/chatbot_client.py`
- `src/nao_orchestrator/nao_orchestrator/orchestrator.py`
- `src/fake_skills/fake_skills/action_server.py`
- `src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json`
