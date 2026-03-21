# nao_skills

`nao_skills` is the local ROS interface package for NAO-specific action
definitions.

It does not launch nodes by itself. Runtime implementations live in packages
such as `nao_replay_motion`.

## Provided Actions

- `nao_skills/action/ReplayMotion`
  - canonical local motion entry point exposed at `/skill/replay_motion`
- `nao_skills/action/DoPosture`
  - transitional posture compatibility interface exposed at `/skill/do_posture`
- `nao_skills/action/DoHeadMotion`
  - retained head-motion interface exposed at `/skill/do_head_motion`

## Runtime Ownership

- `nao_replay_motion` currently implements `ReplayMotion` and `DoHeadMotion`
- `DoPosture` remains a compatibility surface layered on top of replay-motion
  behavior during migration cleanup

## Notes

- this package exists to keep the NAO-specific action contracts versioned and
  buildable inside the workspace
- higher-level orchestration belongs in `nao_orchestrator`, not in this
  interface package
