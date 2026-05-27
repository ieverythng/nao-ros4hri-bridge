# Prompt-Hardening Validation Trace Report (2026-05-27)

## Scope

Container used: `nao_ros2`  
Goal: validate updated planner/chatbot prompt packs with 4 distinct runtime traces
across greeting routing and fake-skill scenario outcomes.

Prompt packs were rebuilt in-container before testing (`planner_llm`, `chatbot_llm`).

## Runtime context checked

- `dialogue_manager.planner_dialogue_act_topic = /nao_orchestrator/planner_dialogue_act`
- `dialogue_manager.planner_dialogue_wording_mode = direct`
- `dialogue_manager.planner_completion_wording_mode = direct`
- Available fake scenarios included:
  - `ambiguous_cup`
  - `path_blocked`
  - `social_wave_unavailable`
  - `head_motion_strict`
  - `area_person_found`

## Trace captures

Raw trace artifacts are stored under:

- `/home/juanbeck/nao-ros4hri-bridge/docs/traces/raw/2026-05-27/trace01_greeting_dialogue/interaction_trace_20260527_014415.jsonl`
- `/home/juanbeck/nao-ros4hri-bridge/docs/traces/raw/2026-05-27/trace02_ambiguous_cup/interaction_trace_20260527_014440.jsonl`
- `/home/juanbeck/nao-ros4hri-bridge/docs/traces/raw/2026-05-27/trace03_path_blocked/interaction_trace_20260527_014509.jsonl`
- `/home/juanbeck/nao-ros4hri-bridge/docs/traces/raw/2026-05-27/trace04_head_motion_strict/interaction_trace_20260527_014535.jsonl`

## Results summary

| Trace | Scenario | Input | Key observed seam events | Result |
| --- | --- | --- | --- | --- |
| `trace01_greeting_dialogue` | default | speech: `Hey Pop!` | `chatbot_turn_trace route=dialogue intent=greet planner_handoff_published=false`; no `planner_request` event | PASS |
| `trace02_ambiguous_cup` | `ambiguous_cup` | planner goal: `find the cup and tell me where it is` | `planner_request -> intents plan -> execution_feedback accepted/running/succeeded/running/failed`; fake skill: `find_object failed`; planner act: `ask_clarification` | PASS |
| `trace03_path_blocked` | `path_blocked` | planner goal: `navigate to the kitchen and report back` | `planner_request -> intents -> execution_feedback accepted/running/failed`; fake skill: `navigate_to failed`; planner act: `ask_clarification` | PASS |
| `trace04_head_motion_strict` | `head_motion_strict` + `allow_open_loop_without_joint_state=false` | planner goal: `look left` | `planner_request -> intents -> execution_feedback accepted/running/succeeded/completed`; planner act: `notify_completion` | PASS |

## Per-trace notes

1. Greeting routing behaved as intended after prompt hardening:
   - Greeting remained `dialogue`.
   - No planner handoff publish was observed for that turn.
2. Ambiguous and blocked fake-skill scenarios both produced deterministic failure
   feedback and planner clarification acts, preserving orchestrator/planner seams.
3. Head-motion strict run did not regress; the step completed under this runtime
   state even with open-loop fallback disabled.
4. No double utterance symptom was observed in these four captures.

## Verdict

The updated prompt-pack behavior is consistent with the intended seam constraints
for these four test traces:

- Greeting stays dialogue-first without planner handoff.
- Planner execution paths remain orchestrator-gated.
- Fake-skill scenario failures propagate through execution feedback and planner dialogue acts.

Further follow-up recommended: add a dedicated `social_wave_unavailable` trace in
the next pass to explicitly stress greeting-vs-wave boundary in a failure case.
