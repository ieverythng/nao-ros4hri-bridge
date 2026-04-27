# nao_look_at

`nao_look_at` implements the upstream ROS4HRI `interaction_skills/action/LookAt`
contract for NAO. It is a first-party robot adapter: policy stays above it,
while this package translates accepted gaze requests into head-joint commands.

## Public ROS Interfaces

| Interface | Type | Role |
| --- | --- | --- |
| `/skill/look_at` | `interaction_skills/action/LookAt` | Canonical look-at action endpoint. |
| `/joint_angles` | `naoqi_bridge_msgs/msg/JointAnglesWithSpeed` | Head yaw/pitch command output. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Runtime diagnostics. |

The node also uses TF when available to transform target-frame requests into
the configured head-reference frame.

## Important Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `look_at_action_name` | `/skill/look_at` | Action endpoint. |
| `joint_angles_topic` | `/joint_angles` | Head command output topic. |
| `require_joint_angles_subscribers` | `false` | Fail when no joint controller is present. |
| `reset_yaw` / `reset_pitch` | `0.0` / `0.0` | Neutral reset pose. |
| `default_speed` | `0.2` | Default head command speed. |
| `look_from_frame` | `CameraTop_frame` | Primary TF frame for target projection. |
| `fallback_look_from_frame` | `base_link` | Secondary TF frame. |
| `tf_lookup_timeout_sec` | `0.2` | TF lookup timeout. |
| `glance_hold_sec` | `0.7` | Hold before `GLANCE` returns to reset. |
| `minimum_target_distance_m` | `0.05` | Reject unstable near-zero targets. |
| `max_yaw_abs` | `1.5` | Yaw clamp. |
| `min_pitch` / `max_pitch` | `-0.67` / `0.51` | Pitch clamps. |

## Planner Contract Role

`nao_orchestrator` can route `type: "look_at"` or supported skill routes into
this action. The planner should express the target abstractly; this package
owns only the robot-side head-command conversion and lifecycle behavior.

Supported practical behaviors:

- `RESET`: publish the neutral head pose.
- target-frame request: transform the `PointStamped` target and command yaw and
  pitch.
- `GLANCE`: track the target briefly, then reset.

Policy modes such as social/random/auto remain conservative placeholders.

## Launch And Test

```bash
ros2 launch nao_look_at nao_look_at.launch.py
```

As part of the robot stack:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py nao_ip:=<robot_ip>
```

Targeted unit check:

```bash
python3 -m pytest -q src/nao_look_at/test/test_nao_look_at_unit.py
```

## Notes

- First-party lifecycle skill package.
- `DoHeadMotion` remains in `nao_replay_motion` until this path is fully
  validated across the robot and simulator demo surfaces.
- Do not move social gaze policy into this package until the planner and
  person/object grounding contracts are stable.
