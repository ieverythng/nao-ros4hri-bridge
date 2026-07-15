# Live Fake-Skill Scenario Probe Report

- Generated: 2026-06-19 02:22:34 UTC
- Container: `nao_ros2`
- Planner request topic: `/planner/request`
- Trace dir (container): `/tmp/codex_fake_skill_probe_20260619_022011`
- Trace JSONL (local copy): `/home/juanbeck/.codex/worktrees/86f6/nao-ros4hri-bridge/docs/artifacts/trace_captures/interaction_trace_20260619_022012.jsonl`

## Case `success_default`

- `mode`: `scenario`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_01_success_default`
- `turn_id`: `probe_turn_01_success_default`
- `goal_text`: `find the cup`
- `event_count`: `10`

### Flow

- `1781835622.870` `execution_feedback` `/planner/execution_feedback`: accepted
- `1781835622.872` `execution_feedback` `/planner/execution_feedback`: running
- `1781835622.976` `execution_feedback` `/planner/execution_feedback`: failed | I could not confirm cup in the knowledge base.
- `1781835625.234` `execution_feedback` `/planner/execution_feedback`: accepted
- `1781835625.236` `execution_feedback` `/planner/execution_feedback`: running
- `1781835628.994` `execution_feedback` `/planner/execution_feedback`: succeeded | I looked around and can report the current scene summary.
- `1781835628.995` `execution_feedback` `/planner/execution_feedback`: running
- `1781835629.010` `execution_feedback` `/planner/execution_feedback`: failed | I could not confirm cup in the knowledge base.
- `1781835629.014` `planner_dialogue_act` `/planner/dialogue_act`: /planner/dialogue_act {"act":"ask_clarification","await_user_response":true,"context":{"goal_text":"find the cup","plan_step_count":3,"requested_intents":["find_object"],"result_payload":{"confidence_policy":"grounded_current_observation","objects":[],"people":[],"result_mode":"success","skill":"scan","skill_server":"scan_skill_server","summary_text":"I looked around and can report the current scene summary.","target":"","target_found":false,"target_kind":"scene"},"result_summary":"I looked around and can report the current scene summary.","scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_01_success_default","plan_id":"plan_1781835625232","plan_version":2,"priority":"normal","reason":"I could not confirm cup in the knowledge base.","slots_needed":["step_1"],"text_hint":"I could not confirm cup in the knowledge base."}
- `1781835629.021` `planner_dialogue_act` `/nao_orchestrator/planner_dialogue_act`: /nao_orchestrator/planner_dialogue_act {"act":"ask_clarification","await_user_response":true,"context":{"goal_text":"find the cup","plan_step_count":3,"requested_intents":["find_object"],"result_payload":{"confidence_policy":"grounded_current_observation","objects":[],"people":[],"result_mode":"success","skill":"scan","skill_server":"scan_skill_server","summary_text":"I looked around and can report the current scene summary.","target":"","target_found":false,"target_kind":"scene"},"result_summary":"I looked around and can report the current scene summary.","scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_01_success_default","plan_id":"plan_1781835625232","plan_version":2,"priority":"normal","reason":"I could not confirm cup in the knowledge base.","slots_needed":["step_1"],"text_hint":"I could not confirm cup in the knowledge base."}

### JSON Excerpts

`final_execution_feedback` payload:
```json
{
  "blocking": true,
  "event_type": "step_failed",
  "goal_id": "goal_probe_01_success_default",
  "intent": "raw_user_input",
  "needs_user_input": true,
  "plan_id": "plan_1781835625232",
  "plan_version": 2,
  "reason": "I could not confirm cup in the knowledge base.",
  "replan_hint": "The target 'cup' is not confirmed in the knowledge base. Re-evaluate the scene to confirm the presence of a cup or clarify the target.",
  "result_payload": {},
  "result_summary": "",
  "retry_budget": 0,
  "scene_targets": [
    "cup"
  ],
  "source": "planner_llm",
  "status": "failed",
  "step": {
    "id": "step_2",
    "name": "find_object",
    "on_failure": "clarify",
    "requires": [
      "step_1"
    ],
    "retry_budget": 1,
    "type": "skill"
  },
  "timestamp_sec": 1781835629.01,
  "unmet_preconditions": [],
  "validation_errors": [],
  "validation_status": "needs_replan"
}
```
`planner_dialogue_act` payload:
```json
{
  "act": "ask_clarification",
  "await_user_response": true,
  "context": {
    "goal_text": "find the cup",
    "plan_step_count": 3,
    "requested_intents": [
      "find_object"
    ],
    "result_payload": {
      "confidence_policy": "grounded_current_observation",
      "objects": [],
      "people": [],
      "result_mode": "success",
      "skill": "scan",
      "skill_server": "scan_skill_server",
      "summary_text": "I looked around and can report the current scene summary.",
      "target": "",
      "target_found": false,
      "target_kind": "scene"
    },
    "result_summary": "I looked around and can report the current scene summary.",
    "scene_targets": [
      "cup"
    ],
    "status": "waiting_user"
  },
  "goal_id": "goal_probe_01_success_default",
  "plan_id": "plan_1781835625232",
  "plan_version": 2,
  "priority": "normal",
  "reason": "I could not confirm cup in the knowledge base.",
  "slots_needed": [
    "step_1"
  ],
  "text_hint": "I could not confirm cup in the knowledge base."
}
```

## Case `deterministic_ambiguous`

- `mode`: `scenario`
- `scenario_id`: `ambiguous_cup`
- `goal_id`: `goal_probe_02_deterministic_ambiguous`
- `turn_id`: `probe_turn_02_deterministic_ambiguous`
- `goal_text`: `find the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts


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


## Case `spatial_near_object`

- `mode`: `scenario`
- `scenario_id`: `object_near_robot`
- `goal_id`: `goal_probe_04_spatial_near_object`
- `turn_id`: `probe_turn_04_spatial_near_object`
- `goal_text`: `find the cup and report how far away it is`
- `event_count`: `10`

### Flow

- `1781835684.338` `execution_feedback` `/planner/execution_feedback`: accepted
- `1781835684.340` `execution_feedback` `/planner/execution_feedback`: running
- `1781835684.384` `execution_feedback` `/planner/execution_feedback`: failed | I could not confirm cup in the knowledge base.
- `1781835686.833` `execution_feedback` `/planner/execution_feedback`: accepted
- `1781835686.835` `execution_feedback` `/planner/execution_feedback`: running
- `1781835690.489` `execution_feedback` `/planner/execution_feedback`: succeeded | I looked around and can report the current scene summary.
- `1781835690.489` `execution_feedback` `/planner/execution_feedback`: running
- `1781835690.500` `execution_feedback` `/planner/execution_feedback`: failed | I could not confirm cup in the knowledge base.
- `1781835690.501` `planner_dialogue_act` `/planner/dialogue_act`: /planner/dialogue_act {"act":"ask_for_help","await_user_response":true,"context":{"goal_text":"find the cup and report how far away it is","plan_step_count":3,"requested_intents":["find_object","report_result"],"result_payload":{"confidence_policy":"grounded_current_observation","objects":[],"people":[],"result_mode":"success","skill":"scan","skill_server":"scan_skill_server","summary_text":"I looked around and can report the current scene summary.","target":"","target_found":false,"target_kind":"scene"},"result_summary":"I looked around and can report the current scene summary.","scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_04_spatial_near_object","plan_id":"plan_1781835686830","plan_version":2,"priority":"normal","reason":"I could not confirm cup in the knowledge base.","slots_needed":["step_1"],"text_hint":"I could not confirm cup in the knowledge base."}
- `1781835690.502` `planner_dialogue_act` `/nao_orchestrator/planner_dialogue_act`: /nao_orchestrator/planner_dialogue_act {"act":"ask_for_help","await_user_response":true,"context":{"goal_text":"find the cup and report how far away it is","plan_step_count":3,"requested_intents":["find_object","report_result"],"result_payload":{"confidence_policy":"grounded_current_observation","objects":[],"people":[],"result_mode":"success","skill":"scan","skill_server":"scan_skill_server","summary_text":"I looked around and can report the current scene summary.","target":"","target_found":false,"target_kind":"scene"},"result_summary":"I looked around and can report the current scene summary.","scene_targets":["cup"],"status":"waiting_user"},"goal_id":"goal_probe_04_spatial_near_object","plan_id":"plan_1781835686830","plan_version":2,"priority":"normal","reason":"I could not confirm cup in the knowledge base.","slots_needed":["step_1"],"text_hint":"I could not confirm cup in the knowledge base."}

### JSON Excerpts

`final_execution_feedback` payload:
```json
{
  "blocking": true,
  "event_type": "step_failed",
  "goal_id": "goal_probe_04_spatial_near_object",
  "intent": "raw_user_input",
  "needs_user_input": false,
  "plan_id": "plan_1781835686830",
  "plan_version": 2,
  "reason": "I could not confirm cup in the knowledge base.",
  "replan_hint": "The target object 'cup' is not confirmed in the knowledge base. Consider scanning the environment to locate the cup or clarifying the user's request for the object's location.",
  "result_payload": {},
  "result_summary": "",
  "retry_budget": 0,
  "scene_targets": [
    "cup"
  ],
  "source": "planner_llm",
  "status": "failed",
  "step": {
    "id": "step_2",
    "name": "find_object",
    "on_failure": "replan",
    "requires": [
      "step_1"
    ],
    "retry_budget": 1,
    "type": "skill"
  },
  "timestamp_sec": 1781835690.499,
  "unmet_preconditions": [],
  "validation_errors": [],
  "validation_status": "needs_replan"
}
```
`planner_dialogue_act` payload:
```json
{
  "act": "ask_for_help",
  "await_user_response": true,
  "context": {
    "goal_text": "find the cup and report how far away it is",
    "plan_step_count": 3,
    "requested_intents": [
      "find_object",
      "report_result"
    ],
    "result_payload": {
      "confidence_policy": "grounded_current_observation",
      "objects": [],
      "people": [],
      "result_mode": "success",
      "skill": "scan",
      "skill_server": "scan_skill_server",
      "summary_text": "I looked around and can report the current scene summary.",
      "target": "",
      "target_found": false,
      "target_kind": "scene"
    },
    "result_summary": "I looked around and can report the current scene summary.",
    "scene_targets": [
      "cup"
    ],
    "status": "waiting_user"
  },
  "goal_id": "goal_probe_04_spatial_near_object",
  "plan_id": "plan_1781835686830",
  "plan_version": 2,
  "priority": "normal",
  "reason": "I could not confirm cup in the knowledge base.",
  "slots_needed": [
    "step_1"
  ],
  "text_hint": "I could not confirm cup in the knowledge base."
}
```

## Case `spatial_far_object`

- `mode`: `scenario`
- `scenario_id`: `object_far_from_robot`
- `goal_id`: `goal_probe_05_spatial_far_object`
- `turn_id`: `probe_turn_05_spatial_far_object`
- `goal_text`: `find the cup and report how far away it is`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts


## Case `random_seeded_stress`

- `mode`: `random_seeded`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_06_random_seeded_stress`
- `turn_id`: `probe_turn_06_random_seeded_stress`
- `goal_text`: `find the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts


## Case `all_fail_always`

- `mode`: `always_fail`
- `scenario_id`: `<none>`
- `goal_id`: `goal_probe_07_all_fail_always`
- `turn_id`: `probe_turn_07_all_fail_always`
- `goal_text`: `find the cup`
- `event_count`: `0`

### Flow

- No events matched this goal id in trace file.

### JSON Excerpts
