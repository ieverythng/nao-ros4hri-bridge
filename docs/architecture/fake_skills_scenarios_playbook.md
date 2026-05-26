# Fake Skills Scenarios Playbook

## Purpose

This is the operator/developer playbook for running fake skill scenarios with
deterministic controls and trace visibility in the same container session.

## What Is Controlled

`fake_skill_server` controls skill outcomes for:

- `find_object`
- `navigate_to`
- `walk_to`
- other fake-skill endpoints exposed by the package

Policy inputs are read from:

- scenario YAML (`fake_skill_scenario_file`)
- active named scenario (`active_scenario_id`)
- global policy mode (`global_mode`)
- per-skill mode overrides (`mode_overrides_json`)
- optional per-request `scenario_id` and `scenario` override payloads

## Launch-Time Controls

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_fake_skills:=true \
  fake_skill_active_scenario_id:=path_blocked \
  fake_skill_global_mode:=scenario \
  fake_skill_random_failure_prob:=0.35 \
  fake_skill_mode_overrides_json:='{}'
```

## Live Controls (No Relaunch Required)

Inspect:

```bash
ros2 param get /fake_skill_server available_scenario_ids
ros2 param get /fake_skill_server active_scenario_id
ros2 param get /fake_skill_server global_mode
ros2 param get /fake_skill_server mode_overrides_json
```

Switch scenario:

```bash
ros2 param set /fake_skill_server active_scenario_id ambiguous_cup
```

Reset named scenario:

```bash
ros2 param set /fake_skill_server active_scenario_id ""
```

Force policy mode:

```bash
ros2 param set /fake_skill_server global_mode always_fail
ros2 param set /fake_skill_server global_mode always_success
ros2 param set /fake_skill_server global_mode random_seeded
ros2 param set /fake_skill_server random_failure_prob 0.35
```

Per-skill override:

```bash
ros2 param set /fake_skill_server mode_overrides_json '{"find_object":"always_fail"}'
```

Semi-interactive selector:

```bash
./scripts/fake_skill_scenario_menu.sh /fake_skill_server
```

## Scenario Precedence Model

Outcome mode is resolved in this order:

1. Request-level `scenario` override (`evidence_policy` payload)
2. Request-level `scenario_id`
3. Runtime `mode_overrides_json`
4. Runtime `global_mode`
5. Scenario/default YAML skill mode

## Quick Action-Level Probes

`find_object` probe:

```bash
ros2 action send_goal /skill/fake/find_object communication_skills/action/ScanScene \
  "{target: 'apple', target_kind: 'object', max_sweeps: 1}"
```

`navigate_to` probe:

```bash
ros2 action send_goal /skill/fake/navigate_to communication_skills/action/ScanScene \
  "{target: 'kitchen', target_kind: 'location', max_sweeps: 1}"
```

## Trace Viewer Pairing (JSON-First)

Run viewer in a separate shell:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
pkill -f interaction_trace_viewer.trace_node || true
ros2 run interaction_trace_viewer trace_node --ros-args \
  -p compact_mode:=false \
  -p include_raw_payloads:=true \
  -p include_channels_csv:="planner/request,intents,planner/execution_feedback,planner/dialogue_act,chatbot_llm/turn_trace,fake_skills/events,world_model/enriched_snapshot,world_model/enriched_text" \
  -p include_event_types_csv:="planner_request,planner_output,execution_feedback,planner_dialogue_act,chatbot_turn_trace,skill_result,kb_snapshot"
```

## Troubleshooting

- If `available_scenario_ids` is empty:
  - verify `fake_skill_scenario_file` path and package share installation.
- If params cannot be read:
  - verify ROS domain and sourced workspace in that shell.
- If action goals fail immediately:
  - check fake skill server node state and action server availability.
- If duplicate speech appears:
  - verify `scan_report_after_success=false` for chatbot-owned completion wording.

## Related Contracts

- `docs/architecture/skill_registry_contract.md`
- `docs/architecture/demo_stack_seam_contract_2026-05-26.md`
- `docs/plans/ros4hri_integration_master_plan_2026-05-18.md`
