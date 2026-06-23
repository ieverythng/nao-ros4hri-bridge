# Fake Skills Runtime Probe (Container) — 2026-05-26

## Environment

- Container: `nao_ros2`
- ROS env: `/opt/ros/jazzy` + `/home/ubuntu/ws/install/setup.bash`

## What Was Checked

1. Node/action availability
2. Fake-skill scenario parameters
3. Scenario outcome behavior (`navigate_to`, `find_object`)
4. Planner-dialogue seam topology topics

## Key Findings

- `fake_skill_server` is running and fake skill actions are available:
  - `/skill/fake/navigate_to`
  - `/skill/fake/find_object`
  - `/skill/fake/walk_to`
  - `/skill/fake/inspect_area`
  - `/skill/fake/wave_greet`
- Scenario parameters present:
  - `active_scenario_id`
  - `available_scenario_ids`
- Runtime policy parameters expected by latest code are missing in the running
  container:
  - `global_mode`
  - `random_failure_prob`
  - `mode_overrides_json`
- This indicates the running container is on an older fake-skills build than
  the latest branch changes.

## Scenario Probes

### Navigate with `path_blocked`

- Set:
  - `ros2 param set /fake_skill_server active_scenario_id path_blocked`
- Goal:
  - `/skill/fake/navigate_to`
  - `target=kitchen`, `target_kind=location`
- Result:
  - action status `ABORTED`
  - payload `result_mode=path_blocked`
  - failure code `path_blocked`

### Find object with `ambiguous_cup`

- Set:
  - `ros2 param set /fake_skill_server active_scenario_id ambiguous_cup`
- Goal:
  - `/skill/fake/find_object`
  - `target=cup`, `target_kind=object`
- Result:
  - action status `ABORTED`
  - payload `result_mode=ambiguous`
  - failure code `ambiguous`

## Routing/Ownership Topology Probe

- `/planner/dialogue_act`:
  - publisher: `planner_llm`
  - subscribers include `dialogue_manager` and `nao_orchestrator` (gate observer)
- `/planner/request`:
  - publisher: `nao_orchestrator` (planner-gate forwarding seam)
  - subscriber: `planner_llm`
- `/dialogue_manager/robot_speech`:
  - publisher: `dialogue_manager`

## Notes

- A parallel test run changed `active_scenario_id` mid-flight once; scenario
  probes should be run sequentially for deterministic comparisons.
- Rebuild/re-source is required before validating runtime policy mode toggles in
  this container session.
