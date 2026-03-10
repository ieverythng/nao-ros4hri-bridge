# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI orchestration, local ASR, and Ollama-backed chat.

## Current Runtime

Core runtime packages:

- `dialogue_manager`: bridges `hri_msgs/LiveSpeech` into `/chatbot/user_text`, merges short ASR bursts into one turn, and dispatches `/skill/say`
- `nao_chatbot`: mission controller, `/skill/chat` server, prompt/config pipeline, push-to-talk CLI
- `nao_skill_servers`: execution package for posture, head motion, say, and the posture fallback bridge
- `asr_vosk`: local Vosk ASR lifecycle node publishing ROS4HRI `LiveSpeech`
- `simple_audio_capture`: microphone capture to `audio_common_msgs/AudioData`
- `nao_skills`, `communication_skills`, `std_skills`: canonical interfaces

Main live flow:

```text
simple_audio_capture -> asr_vosk -> /humans/voices/anonymous_speaker/speech
    -> dialogue_manager_node -> /chatbot/user_text
    -> mission_controller_node
        -> /skill/chat -> ollama_chatbot_node
        -> /chatbot/assistant_text + /chatbot/intent
    -> dialogue_manager_node -> /skill/say -> say_skill_server_node -> /tts_engine/tts
```

Robot execution flow:

```text
mission_controller_node
    -> /skill/do_posture -> posture_skill_server_node
       fallback -> /chatbot/posture_command -> nao_posture_bridge_node
    -> /skill/do_head_motion -> head_motion_skill_server_node -> /joint_angles
```

## Launch Profiles

Primary launch files live in `src/nao_chatbot/launch/`:

- `nao_chatbot_stack.launch.py`: full configurable stack surface
- `nao_chatbot_skills.launch.py`: skills-first runtime without local ASR
- `nao_chatbot_skills_asr.launch.py`: skills runtime plus `simple_audio_capture` + `asr_vosk`
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
  std_skills communication_skills nao_skills dialogue_manager \
  asr_vosk simple_audio_capture nao_skill_servers nao_chatbot
```

Run the local validation suite:

```bash
./scripts/run_tests.sh
./.venv/bin/pre-commit run --all-files
```

## Notes

- Vendored/local overlay repos under `src/motions_skills/` and `src/std_skills/` are intentionally left untouched.
- `naoqi_driver` already exposes camera topics and joint interfaces; future vision work should build on those published ROS interfaces instead of adding a parallel capture stack.
