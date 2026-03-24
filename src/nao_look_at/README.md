# nao_look_at

`nao_look_at` is the NAO-side implementation of the upstream ROS4HRI
`interaction_skills/look_at` contract.

It now supports two practical behaviors:

- `RESET` is executed by publishing a neutral head pose to `/joint_angles`
- target-frame requests transform the requested `PointStamped` target into the
  configured head-reference frame and publish the matching yaw/pitch command
- `GLANCE` uses the same target tracking path, then returns to reset after a
  short hold time

The remaining policy-based gaze modes such as `social`, `random`, and `auto`
are still placeholders for later work.

## ROS API

- action: `/skill/look_at`
- type: `interaction_skills/action/LookAt`

## Launch

Standalone:

```bash
ros2 launch nao_look_at nao_look_at.launch.py
```

As part of the migrated stack:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py
```

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `look_at_action_name` | `/skill/look_at` | Action endpoint |
| `joint_angles_topic` | `/joint_angles` | Transitional head-control output |
| `require_joint_angles_subscribers` | `false` | Fail fast when no joint controller is present |
| `reset_yaw` | `0.0` | Neutral yaw used by `RESET` |
| `reset_pitch` | `0.0` | Neutral pitch used by `RESET` |
| `default_speed` | `0.2` | Transitional joint-speed value |
| `look_from_frame` | `CameraTop_frame` | Primary TF frame used for gaze projection |
| `fallback_look_from_frame` | `base_link` | Secondary TF frame if the primary frame is unavailable |
| `tf_lookup_timeout_sec` | `0.2` | TF lookup timeout for target-frame requests |
| `glance_hold_sec` | `0.7` | Hold time before `GLANCE` returns to reset |
| `minimum_target_distance_m` | `0.05` | Reject unstable near-zero target vectors |
| `max_yaw_abs` | `1.5` | Absolute clamp for computed head yaw |
| `min_pitch` | `-0.67` | Upper-looking pitch clamp |
| `max_pitch` | `0.51` | Lower-looking pitch clamp |

## Migration Notes

- the canonical ROS4HRI `look_at` definition lives in the upstream
  `interaction_skills` package; this package only implements that contract for
  NAO
- target-frame look-at is now available for real robot TF validation and
  planned orchestrator execution
- higher-level policy selection is still conservative and intentionally does not
  auto-expand into `social` or `random` gaze yet
- `DoHeadMotion` remains active in `nao_replay_motion` until this package is
  validated more thoroughly across robot and simulator demo paths
