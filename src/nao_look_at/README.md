# nao_look_at

`nao_look_at` is the scaffold package for the ROS4HRI `look_at` skill on NAO.

It already exposes the canonical `/skill/look_at` action using
`interaction_skills/action/LookAt`, but the first implementation pass is
intentionally limited:

- `RESET` is executed by publishing a neutral head pose to `/joint_angles`
- target-based requests are accepted into the server but currently return
  `ROS_ENOTSUP`
- policy-based gaze modes such as `social`, `random`, and `auto` are scaffolded
  but not yet implemented

The later completion phase will add RViz validation, target projection, and
interaction-simulator-based gaze policies.

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
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
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

## Migration Notes

- this package exists now so the repo already carries the canonical ROS4HRI
  `look_at` skill surface
- full target projection and gaze-policy support are intentionally deferred
- `DoHeadMotion` remains active in `nao_replay_motion` until this package is
  validated with RViz and the interaction simulator
