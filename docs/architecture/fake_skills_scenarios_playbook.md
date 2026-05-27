# Fake Skills Scenarios Playbook

## Purpose

Operator/developer playbook for running fake-skill scenarios with deterministic controls and copy-paste CLI commands.

## Fastest Path (CLI Only)

If you only want launch params + live `ros2 param set` controls, use this section.

### Launch-Time Parameter Set

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_fake_skills:=true \
  fake_skill_global_mode:=scenario \
  fake_skill_active_scenario_id:= \
  fake_skill_random_failure_prob:=0.50 \
  fake_skill_mode_overrides_json:='{}' \
  head_motion_allow_open_loop_without_joint_state:=true \
  head_motion_assume_success_on_convergence_timeout:=true
```

### Live Runtime Parameter Set (No Relaunch)

```bash
# fake_skill_server
ros2 param set /fake_skill_server global_mode scenario
ros2 param set /fake_skill_server active_scenario_id ""
ros2 param set /fake_skill_server random_failure_prob 0.50
ros2 param set /fake_skill_server mode_overrides_json '{}'

# head_motion_skill_server (open-loop enabled for no-robot testing)
ros2 param set /head_motion_skill_server allow_open_loop_without_joint_state true
ros2 param set /head_motion_skill_server assume_success_on_convergence_timeout true
```

## Available Choices (Copy/Paste)

### `/fake_skill_server` Global Mode

```bash
ros2 param set /fake_skill_server global_mode scenario
ros2 param set /fake_skill_server global_mode always_success
ros2 param set /fake_skill_server global_mode always_fail
ros2 param set /fake_skill_server global_mode every_other
ros2 param set /fake_skill_server global_mode random_seeded
```

### `/fake_skill_server` Scenarios (Current Stack)

Read current list:

```bash
ros2 param get /fake_skill_server available_scenario_ids
```

Current IDs:

- `ambiguous_cup`
- `area_person_found`
- `head_motion_strict`
- `path_blocked`
- `social_wave_unavailable`

Set/reset:

```bash
ros2 param set /fake_skill_server active_scenario_id ambiguous_cup
ros2 param set /fake_skill_server active_scenario_id area_person_found
ros2 param set /fake_skill_server active_scenario_id head_motion_strict
ros2 param set /fake_skill_server active_scenario_id path_blocked
ros2 param set /fake_skill_server active_scenario_id social_wave_unavailable
ros2 param set /fake_skill_server active_scenario_id ""
```

### `/fake_skill_server` Skill-Level Override Map

```bash
# fail only find_object regardless of scenario/global mode
ros2 param set /fake_skill_server mode_overrides_json '{"find_object":"always_fail"}'

# mix policies by skill
ros2 param set /fake_skill_server mode_overrides_json '{"find_object":"always_fail","navigate_to":"always_success","wave_greet":"always_fail"}'
```

### `/head_motion_skill_server` Open-Loop Controls

Default (good for no-robot testing):

```bash
ros2 param set /head_motion_skill_server allow_open_loop_without_joint_state true
ros2 param set /head_motion_skill_server assume_success_on_convergence_timeout true
```

Strict (disable open-loop fallback):

```bash
ros2 param set /head_motion_skill_server allow_open_loop_without_joint_state false
ros2 param set /head_motion_skill_server assume_success_on_convergence_timeout false
```

### Quick Interactive Menu

```bash
./scripts/fake_skill_scenario_menu.sh /fake_skill_server /head_motion_skill_server
```

## Scenario Effect Matrix

| Scenario ID | Main Effect |
|---|---|
| `ambiguous_cup` | `find_object` returns ambiguous |
| `area_person_found` | `inspect_area` returns person found |
| `head_motion_strict` | no fake-skill override; use with head-motion strict flags (open-loop off) |
| `path_blocked` | `navigate_to` returns path blocked |
| `social_wave_unavailable` | `wave_greet` returns motion unavailable |

## Planner/Dialogue Seam Checks

Verify the critical seam contract:

```bash
ros2 param get /dialogue_manager planner_dialogue_act_topic
ros2 param get /dialogue_manager planner_dialogue_wording_mode
ros2 param get /dialogue_manager planner_completion_wording_mode
ros2 param get /nao_orchestrator planner_dialogue_act_topic
ros2 param get /nao_orchestrator planner_dialogue_relay_topic
```

Expected:

- `dialogue_manager.planner_dialogue_act_topic = /nao_orchestrator/planner_dialogue_act`
- `dialogue_manager.planner_dialogue_wording_mode = direct`
- `dialogue_manager.planner_completion_wording_mode = direct`
- `nao_orchestrator.planner_dialogue_act_topic = /planner/dialogue_act`
- `nao_orchestrator.planner_dialogue_relay_topic = /nao_orchestrator/planner_dialogue_act`

## Investigation Note: Why "Hey Pop!" Triggered Planner + Wave

From live logs in the rebuilt container:

- `chatbot_llm` classified first turn as execution:
  - `ROUTE_RESOLVED | route=execution intent=greet`
- request was forwarded through orchestrator gate:
  - `Planner gate forwarded request | goal_id=goal_default___1`
- `planner_llm` received `intents=['greeting','greet']` and planned:
  - `step=skill/wave_greet`
  - then `step=skill/report_result`
- `dialogue_manager` spoke both:
  - initial chatbot response (`"Hello! How can I help you today?"`)
  - planner completion (`"I have greeted you with a wave..."`)

That specific first turn was not random: planner wave came from greeting being routed as execution.

## Trace Pairing

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
scripts/run_interaction_trace_viewer_compact_json.sh
```

## Related Contracts

- `docs/architecture/skill_registry_contract.md`
- `docs/architecture/demo_stack_seam_contract_2026-05-26.md`
- `docs/plans/ros4hri_integration_master_plan_2026-05-18.html`
