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

## Launch

Standalone:

```bash
ros2 launch nao_replay_motion nao_replay_motion.launch.py
```

As part of the migrated stack:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py
```

## Runtime Notes

- direct NAOqi posture execution and topic fallback are both supported
- `/skill/do_posture` remains temporary and simply maps posture names onto the
  replay-motion action
- `DoHeadMotion` intentionally stays in this package during the migration so
  the existing head-motion path remains stable
- the fallback bridge still uses `/chatbot/posture_command` until the full ASR
  and orchestration cleanup is complete

## Test Surface

- unit tests cover motion-name normalization and the compatibility layer
- launch verification is done through the migrated stack smoke tests
