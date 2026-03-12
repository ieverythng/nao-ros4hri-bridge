# nao_replay_motion

`nao_replay_motion` is the NAO-specific motion execution package introduced by
the ROS4HRI refactor.

It provides:

- `/skill/replay_motion` using `nao_skills/action/ReplayMotion`
- temporary `/skill/do_posture` compatibility on top of replay-motion
- retained `/skill/do_head_motion` using `nao_skills/action/DoHeadMotion`
- posture fallback bridge executable `nao_posture_bridge_node`

The initial replay-motion catalog is posture-oriented:

- `stand`
- `standinit`
- `sit`
- `kneel`
- `crouch`

Additional motion primitives can be added later without changing the canonical
replay-motion entry point.
