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
