# nao_skills

`nao_skills` owns local NAO-specific ROS action definitions. It is an interface
package only: it does not launch nodes and it should not contain execution,
planning, or dialogue policy.

## Public ROS Interfaces

| Action | Intended endpoint | Owner |
| --- | --- | --- |
| `nao_skills/action/ReplayMotion` | `/skill/replay_motion` | Implemented by `nao_replay_motion`. |
| `nao_skills/action/DoPosture` | `/skill/do_posture` | Transitional compatibility action in `nao_replay_motion`. |
| `nao_skills/action/DoHeadMotion` | `/skill/do_head_motion` | Implemented by `nao_replay_motion`. |

## Action Shapes

`ReplayMotion.Goal`:

```text
string motion_name
float32 speed
```

`DoPosture.Goal`:

```text
string posture_name
float32 speed
```

`DoHeadMotion.Goal`:

```text
float32 yaw
float32 pitch
float32 speed
bool relative
```

All three actions return:

```text
bool success
string message
float32 duration
```

and publish feedback:

```text
string status
float32 progress
```

## Planner Contract Role

These actions are the NAO-side execution contracts that `nao_orchestrator` may
target after validating a planner step. Planner JSON should stay abstract
(`type`, `name`, `args`, `requires`, `on_failure`, `retry_budget`) and should
not hard-code NAO action internals unless the orchestrator route explicitly
supports them.

## Build And Check

```bash
colcon build --packages-select nao_skills
```

Inspect generated interfaces:

```bash
ros2 interface show nao_skills/action/ReplayMotion
ros2 interface show nao_skills/action/DoHeadMotion
```

## Notes

- First-party interface package.
- Keep additions small and justified by a real executor route.
- Do not add mock/demo-only actions here unless they are intended to become
  stable NAO interfaces.
