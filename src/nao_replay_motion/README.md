# nao_replay_motion

`nao_replay_motion` owns the local NAO motion execution adapters used by the
planner/orchestrator stack. It is first-party code and is safe to evolve when a
demo needs deterministic robot-side motion behavior.

The package currently exposes posture replay, a temporary posture compatibility
action, and the retained head-motion action. It does not own planning policy;
`nao_orchestrator` decides which action to call.

## Public ROS Interfaces

| Interface | Type | Role |
| --- | --- | --- |
| `/skill/replay_motion` | `nao_skills/action/ReplayMotion` | Canonical local named motion/posture action. |
| `/skill/do_posture` | `nao_skills/action/DoPosture` | Transitional compatibility action mapped onto replay-motion behavior. |
| `/skill/do_head_motion` | `nao_skills/action/DoHeadMotion` | Retained head yaw/pitch action used by older planner routes. |
| `/joint_angles` | `naoqi_bridge_msgs/msg/JointAnglesWithSpeed` | Head-motion command output. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Optional convergence feedback for head motion. |
| `/chatbot/posture_command` | `std_msgs/msg/String` | Fallback posture command topic when NAOqi is unavailable. |
| `/chatbot/posture_command_result` | `std_msgs/msg/String` | Fallback posture result topic. |

## Important Parameters

Replay/posture parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `nao_ip` | `127.0.0.1` in node, `172.26.112.62` in launch | NAOqi host. |
| `nao_port` | `9559` | NAOqi port. |
| `action_name` | `/skill/replay_motion` | Replay-motion action name. |
| `posture_compat_action_name` | `/skill/do_posture` | Temporary posture action name. |
| `default_speed` | `0.8` | Speed used when the request omits one. |
| `fallback_to_posture_topic` | `true` | Use topic bridge when NAOqi is unavailable. |
| `posture_result_timeout_sec` | `5.0` | Wait time for a result-bearing posture bridge before configured open-loop fallback. |

Head-motion parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `action_name` | `/skill/do_head_motion` | Head-motion action name. |
| `default_speed` | `0.2` | Default joint command speed. |
| `yaw_min` / `yaw_max` | `-2.0857` / `2.0857` | Head yaw clamp. |
| `pitch_min` / `pitch_max` | `-0.6720` / `0.5149` | Head pitch clamp. |
| `require_joint_angles_subscribers` | `false` | Fail when no joint controller is subscribed. |
| `convergence_timeout_sec` | `3.0` | Wait time for joint-state convergence. |
| `retry_on_convergence_timeout` | `true` | Retry once on convergence timeout. |
| `allow_open_loop_without_joint_state` | `false` | Profile-controlled fallback: publish absolute goals when no head joint state is available. |
| `assume_success_on_convergence_timeout` | `false` | Demo/sim fallback: report success after publishing when convergence cannot be observed. |

### Head motion node lifecycle

`head_motion_skill_server` is a plain `rclpy` node (not `rclpy.lifecycle`). It is
started by launch, runs until process exit, and tears down in `main()` after
executor spin. Configure convergence and open-loop fallbacks via parameters
above rather than lifecycle transitions.

## Planner Contract Role

`nao_orchestrator` can execute planner steps with `type: "skill"` and
`name: "perform_motion"` / `"motion"` by sending action goals into this package.
For Monday demo work, this package is useful as a real interface reference, but
mock/demo behavior should live in first-party demo code rather than modifying
`motions_skills`.

Supported replay names are normalized aliases such as `stand`, `standinit`,
`sit`, `kneel`, and `crouch`.

## Launch And Test

```bash
ros2 launch nao_replay_motion nao_replay_motion.launch.py
```

Demo/simulator head-motion fallback:

```bash
ros2 launch nao_replay_motion nao_replay_motion.launch.py \
  head_motion_allow_open_loop_without_joint_state:=true \
  head_motion_assume_success_on_convergence_timeout:=true
```

With robot details:

```bash
ros2 launch nao_replay_motion nao_replay_motion.launch.py \
  nao_ip:=<robot_ip> nao_port:=9559
```

Targeted unit check:

```bash
python3 -m pytest -q src/nao_replay_motion/test/test_nao_replay_motion_unit.py
```

## Notes

- First-party NAO adapter package.
- `/skill/do_posture` is transitional and should not become the long-term
  planner-facing surface.
- Head motion remains a real adapter path in the integrated profiles, but
  hardware convergence can be unavailable. The open-loop parameters permit a
  command to be published without claiming physical convergence.
- Connect-time autonomous-life disable and wake-up behavior are opt-in through
  launch parameters.
