# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for the NAO ROS4HRI migration.

## Current Runtime

Primary migrated packages:

- `dialogue_manager`: upstream ROS4HRI lifecycle runtime and canonical owner of
  `/skill/chat`, `/skill/ask`, and `/skill/say`
- `chatbot_llm`: upstream-aligned chatbot backend contract using the local
  Ollama prompt/history pipeline
- `nao_orchestrator`: downstream `/intents` consumer replacing the old
  mission-controller execution role
- `nao_say_skill`: NAO-specific speech execution on `/nao/say`
- `nao_replay_motion`: NAO posture/replay-motion execution and retained
  `/skill/do_head_motion`
- `nao_look_at`: scaffolded `/skill/look_at` lifecycle package

Legacy or still-migrating pieces:

- `asr_vosk`: active cutover to the upstream ROS4HRI contract is still pending
- `simple_audio_capture`: still used as the local microphone source and will
  eventually sit behind the upstream-style ASR adapter path
- retained topic fallbacks such as `/chatbot/posture_command` exist only as
  transition bridges for motion execution

Current migrated flow:

```text
speech input -> dialogue_manager -> chatbot_llm
    -> /intents -> nao_orchestrator
    -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

This split is deliberate:

- `dialogue_manager` owns dialogue and canonical communication skills
- `chatbot_llm` owns backend model interaction
- `nao_orchestrator` owns robot-side intent dispatch only

## Launch Profiles

Primary launch files live in `src/nao_chatbot/launch/`:

- `nao_chatbot_ros4hri_migration.launch.py`: primary migrated runtime
- `nao_chatbot_asr_only.launch.py`: isolated ASR pipeline

Quick reference:

- [docs/launch_profiles.md](docs/launch_profiles.md)
- [docs/current_workflow.md](docs/current_workflow.md)
- [docs/node_interactions_map.md](docs/node_interactions_map.md)
- [docs/ollama_chatbot_architecture.md](docs/ollama_chatbot_architecture.md)
- [docs/asr_vosk_setup.md](docs/asr_vosk_setup.md)
- [docs/nao_camera_vlm_research.md](docs/nao_camera_vlm_research.md)

## Build And Test

Build selected packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  chatbot_llm dialogue_manager nao_orchestrator nao_say_skill \
  nao_replay_motion nao_look_at nao_chatbot asr_vosk simple_audio_capture
```

Run the local validation suite:

```bash
./scripts/run_tests.sh
./.venv/bin/pre-commit run --all-files
```

## Notes

- `src/dialogue_manager/` and `src/chatbot_llm/` are nested fork repos; review
  and PR them in their own histories, not through the monorepo diff.
- legacy `mission_controller`, `ollama_chatbot`, and `nao_skill_servers`
  runtime surfaces have been removed from the active workspace
- Vendored/local overlay repos under `src/motions_skills/` and `src/std_skills/` are intentionally left untouched.
- `naoqi_driver` already exposes camera topics and joint interfaces; future vision work should build on those published ROS interfaces instead of adding a parallel capture stack.
