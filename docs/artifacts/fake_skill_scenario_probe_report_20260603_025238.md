# Live Fake-Skill Scenario Probe Report

- Generated: 2026-06-03 02:54:34 UTC
- Container: `nao_ros2`
- Planner request topic: `/planner/request`
- Trace dir (container): `/tmp/codex_fake_skill_probe_20260603_025238`
- Trace JSONL (local copy): `/home/juanbeck/nao-ros4hri-bridge/docs/artifacts/trace_captures/interaction_trace_20260603_025240.jsonl`

## Case `success_default`

- `mode`: `scenario`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_01_success_default`
- `turn_id`: `probe_turn_01_success_default`
- `goal_text`: `find the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts


## Case `deterministic_ambiguous`

- `mode`: `scenario`
- `scenario_id`: `ambiguous_cup`
- `goal_id`: `goal_probe_02_deterministic_ambiguous`
- `turn_id`: `probe_turn_02_deterministic_ambiguous`
- `goal_text`: `find the cup`
- `event_count`: `5`

### Flow

- `1780455197.273` `execution_feedback` `/planner/execution_feedback`: accepted
- `1780455197.301` `execution_feedback` `/planner/execution_feedback`: running
- `1780455198.234` `execution_feedback` `/planner/execution_feedback`: failed | I found multiple candidates for cup.
- `1780455199.728` `planner_dialogue_act` `/planner/dialogue_act`: /planner/dialogue_act {"act":"ask_clarification","await_user_response":true,"context":{"goal_text":"find the cup","plan_step_count":1,"requested_intents":["find_object"],"result_payload":{},"scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_02_deterministic_ambiguous","plan_id":"plan_1780455199725","plan_version":2,"priority":"normal","reason":"Multiple candidates found for cup; user clarification needed to disambiguate.","slots_needed":[],"text_hint":"Multiple candidates found for cup; user clarification needed to disambiguate."}
- `1780455199.731` `planner_dialogue_act` `/nao_orchestrator/planner_dialogue_act`: /nao_orchestrator/planner_dialogue_act {"act":"ask_clarification","await_user_response":true,"context":{"goal_text":"find the cup","plan_step_count":1,"requested_intents":["find_object"],"result_payload":{},"scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_02_deterministic_ambiguous","plan_id":"plan_1780455199725","plan_version":2,"priority":"normal","reason":"Multiple candidates found for cup; user clarification needed to disambiguate.","slots_needed":[],"text_hint":"Multiple candidates found for cup; user clarification needed to disambiguate."}

### JSON Excerpts

`final_execution_feedback` payload:
```json
{
  "blocking": true,
  "event_type": "step_failed",
  "goal_id": "goal_probe_02_deterministic_ambiguous",
  "intent": "raw_user_input",
  "needs_user_input": false,
  "plan_id": "plan_1780455197220",
  "plan_version": 1,
  "reason": "I found multiple candidates for cup.",
  "replan_hint": "",
  "result_payload": {},
  "result_summary": "",
  "retry_budget": 2,
  "scene_targets": [
    "cup"
  ],
  "source": "planner_llm",
  "status": "failed",
  "step": {
    "id": "step_1",
    "name": "find_object",
    "on_failure": "replan",
    "requires": [],
    "retry_budget": 2,
    "type": "skill"
  },
  "timestamp_sec": 1780455198.234,
  "unmet_preconditions": [],
  "validation_errors": [],
  "validation_status": "valid"
}
```
`planner_dialogue_act` payload:
```json
{
  "act": "ask_clarification",
  "await_user_response": true,
  "context": {
    "goal_text": "find the cup",
    "plan_step_count": 1,
    "requested_intents": [
      "find_object"
    ],
    "result_payload": {},
    "scene_targets": [
      "cup"
    ],
    "status": "waiting_user"
  },
  "goal_id": "goal_probe_02_deterministic_ambiguous",
  "plan_id": "plan_1780455199725",
  "plan_version": 2,
  "priority": "normal",
  "reason": "Multiple candidates found for cup; user clarification needed to disambiguate.",
  "slots_needed": [],
  "text_hint": "Multiple candidates found for cup; user clarification needed to disambiguate."
}
```

## Case `deterministic_path_blocked`

- `mode`: `scenario`
- `scenario_id`: `path_blocked`
- `goal_id`: `goal_probe_03_deterministic_path_blocked`
- `turn_id`: `probe_turn_03_deterministic_path_blocked`
- `goal_text`: `navigate to the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts


## Case `random_seeded_stress`

- `mode`: `random_seeded`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_04_random_seeded_stress`
- `turn_id`: `probe_turn_04_random_seeded_stress`
- `goal_text`: `find the cup`
- `event_count`: `16`

### Flow

- `1780455240.954` `execution_feedback` `/planner/execution_feedback`: accepted
- `1780455240.956` `execution_feedback` `/planner/execution_feedback`: running
- `1780455241.766` `execution_feedback` `/planner/execution_feedback`: failed | I could not find cup.
- `1780455243.358` `execution_feedback` `/planner/execution_feedback`: accepted
- `1780455243.360` `execution_feedback` `/planner/execution_feedback`: running
- `1780455247.442` `execution_feedback` `/planner/execution_feedback`: succeeded | {"backend":"emorobcare_cv","objects":[],"observer":"myself"}
- `1780455247.443` `execution_feedback` `/planner/execution_feedback`: running
- `1780455248.264` `execution_feedback` `/planner/execution_feedback`: failed | I could not find cup.
- `1780455249.836` `execution_feedback` `/planner/execution_feedback`: accepted
- `1780455249.839` `execution_feedback` `/planner/execution_feedback`: running
- `1780455253.532` `execution_feedback` `/planner/execution_feedback`: succeeded | {"backend":"emorobcare_cv","objects":[],"observer":"myself"}
- `1780455253.533` `execution_feedback` `/planner/execution_feedback`: running
- `1780455254.349` `execution_feedback` `/planner/execution_feedback`: succeeded | I found one cup.
- `1780455254.477` `execution_feedback` `/planner/execution_feedback`: running
- `1780455254.630` `execution_feedback` `/planner/execution_feedback`: succeeded | I have found the cup.
- `1780455254.630` `execution_feedback` `/planner/execution_feedback`: completed | I have found the cup.

### JSON Excerpts

`final_execution_feedback` payload:
```json
{
  "blocking": false,
  "event_type": "plan_completed",
  "goal_id": "goal_probe_04_random_seeded_stress",
  "intent": "raw_user_input",
  "needs_user_input": false,
  "plan_id": "plan_1780455249826",
  "plan_version": 3,
  "reason": "",
  "replan_hint": "perform scan to refresh scene perception before retrying find_object",
  "result_payload": {
    "skill": "report_result",
    "status": "completed",
    "summary_text": "I have found the cup."
  },
  "result_summary": "I have found the cup.",
  "retry_budget": 0,
  "scene_targets": [
    "cup"
  ],
  "source": "planner_llm",
  "status": "completed",
  "timestamp_sec": 1780455254.629,
  "unmet_preconditions": [],
  "validation_errors": [],
  "validation_status": "failed"
}
```

## Case `all_fail_always`

- `mode`: `always_fail`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_05_all_fail_always`
- `turn_id`: `probe_turn_05_all_fail_always`
- `goal_text`: `find the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts
