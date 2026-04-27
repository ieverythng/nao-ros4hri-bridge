# nao_say_skill

`nao_say_skill` owns the NAO-side speech execution hook. It intentionally
exposes `/nao/say`, not canonical `/skill/say`; dialogue ownership remains in
`dialogue_manager`, and planner/executor routing should treat this package as a
robot adapter.

## Public ROS Interfaces

| Interface | Type | Role |
| --- | --- | --- |
| `/nao/say` | `communication_skills/action/Say` | NAO-specific speech action. |
| `/tts_engine/tts` | `communication_skills/action/Say` | Compatibility TTS action used by `dialogue_manager`. |
| `/debug/say` | `communication_skills/action/Say` | Optional operator/debug TTS action. |
| `/speech` | `std_msgs/msg/String` | Direct robot speech fallback topic. |
| `/debug/nao_say/speech` | `std_msgs/msg/String` | Debug speech mirror. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Runtime diagnostics. |

## Important Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `say_action_name` | `/nao/say` | NAO-specific action endpoint. |
| `tts_action_name` | `/tts_engine/tts` | Compatibility TTS action endpoint. |
| `tts_backend_action_name` | `""` | Optional downstream TTS action server. |
| `debug_tts_action_name` | `/debug/say` | Debug action endpoint. |
| `speech_topic` | `/speech` | Direct speech fallback. |
| `debug_speech_topic` | `/debug/nao_say/speech` | Debug mirror topic. |
| `default_language` | `en-US` | Default language metadata. |
| `default_volume` | `1.0` | Default volume metadata. |
| `tts_server_wait_sec` | `0.5` | Wait for downstream TTS server. |
| `fallback_to_speech_topic` | `true` | Publish to `/speech` when TTS is unavailable. |
| `forward_debug_tts_action` | `true` | Forward utterances to debug TTS when available. |
| `fallback_to_debug_topic` | `true` | Publish debug text when TTS is unavailable. |

## Planner Contract Role

Planner steps should normally request `type: "say"` and let
`nao_orchestrator` route deterministic speech behavior. This package is the
final robot-side speech executor; it should not parse planner goals or own
dialogue policy.

## Launch And Test

```bash
ros2 launch nao_say_skill nao_say_skill.launch.py
```

As part of the full stack:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
ros2 launch nao_chatbot nao_chatbot_robot.launch.py nao_ip:=<robot_ip>
```

Targeted unit check:

```bash
python3 -m pytest -q src/nao_say_skill/test/test_nao_say_skill_unit.py
```

## Notes

- First-party lifecycle skill package based on the local `rpk` pattern.
- Do not claim `/skill/say`; that remains a dialogue-manager contract.
- Simulator-side TTS should stay disabled unless explicitly needed to avoid
  multiple `/tts_engine/tts` servers.
