# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI + chatbot orchestration with an ongoing migration to skill actions.

This repository uses `src/` as the canonical source tree.

## Migration Status (As of February 25, 2026)

- Phase 0 complete: `nao_skills` now exposes `DoPosture.action`, `SayText.action`, and `Chat.action`.
- Phase 1 complete: skill servers available for `/skill/do_posture`, `/skill/say`, and `/skill/chat`.
- Phase 2 complete: mission controller can dispatch posture and chat over actions.
- Dialogue migration complete: `dialogue_manager_node` replaces bridge logic while preserving LiveSpeech input flow.
- Backward compatibility preserved: topic fallback remains active when action servers are unavailable.

## Current Runtime Architecture

Presentation diagram (legacy topic flow): [`docs/architecture_diagram.md`](docs/architecture_diagram.md)

Main nodes:

- `dialogue_manager_node`: turn orchestration (`LiveSpeech` in, `/chatbot/user_text` out, `/skill/say` dispatch)
- `nao_rqt_bridge_node`: compatibility alias to `dialogue_manager_node`
- `asr_vosk_node`: local microphone ASR (Vosk) -> `LiveSpeech`
- `mission_controller_node`: intent/routing logic (`rules` or `backend`) + `/skill/chat` client
- `chat_skill_server_node`: `/skill/chat` action server backed by Ollama
- `ollama_responder_node`: legacy backend-topic responder (optional compatibility mode)
- `nao_posture_bridge_node`: legacy topic posture bridge (`/chatbot/posture_command` -> NAOqi `ALRobotPosture`)
- `posture_skill_server_node`: action posture bridge (`/skill/do_posture` -> NAOqi `ALRobotPosture`)
- `say_skill_server_node`: action speech bridge (`/skill/say` -> `/tts_engine/tts`)

Active flows today:

```text
Speech/Text input
  -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
  -> dialogue_manager_node -> /chatbot/user_text
  -> mission_controller_node
       rules mode   -> /chatbot/assistant_text
       backend mode -> /skill/chat (Chat action) -> chat_skill_server_node -> /chatbot/assistant_text
       posture intent -> /skill/do_posture (DoPosture action)
  -> posture_skill_server_node -> ALRobotPosture.goToPosture
     fallback (if qi unavailable) -> /chatbot/posture_command -> nao_posture_bridge_node -> ALRobotPosture.goToPosture
  -> dialogue_manager_node -> /skill/say (SayText action) -> say_skill_server_node -> /tts_engine/tts
  -> dialogue_manager_node -> /speech (compatibility + ASR guard pathway)
```

ASR note:

- In this stack, ASR is a ROS4HRI percept pipeline (publisher of `LiveSpeech`),
  not an action skill.
- Skills are used for execution APIs (for example posture via `/skill/do_posture`).
- `naoqi_driver` robot microphone source is `/audio` (`naoqi_bridge_msgs/msg/AudioBuffer`),
  and NAO built-in recognition is exposed via `/listen`
  (`naoqi_bridge_msgs/action/Listen`).
- `bridge_input_speech_topic` must remain a `LiveSpeech` topic, so `/audio`
  requires an adapter node before it can be bridged to `/chatbot/user_text`.

```text
Manual/External skill client
  -> /skill/do_posture (nao_skills/action/DoPosture)
  -> posture_skill_server_node
  -> ALRobotPosture.goToPosture
```

## Repository Layout

```text
.
├── docker/
│   ├── Dockerfile
│   ├── Dockerfile.full
│   ├── keys/socialminds.gpg
│   └── ros_entrypoint.sh
├── scripts/
│   ├── build_docker.sh
│   ├── export_docker_image.sh
│   ├── import_docker_image.sh
│   ├── setup_dev_tools.sh
│   └── run_precommit.sh
├── src/
│   ├── nao_skills/
│   │   ├── action/DoPosture.action
│   │   ├── action/SayText.action
│   │   ├── action/Chat.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── std_skills/
│   │   ├── msg/
│   │   ├── srv/
│   │   ├── action/
│   │   └── package.xml
│   ├── nao_posture_bridge/
│   │   ├── nao_posture_bridge/posture_skill_server.py
│   │   ├── nao_posture_bridge/say_skill_server.py
│   │   ├── src/nao_posture_bridge_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── nao_chatbot/
│       ├── launch/nao_chatbot_stack.launch.py
│       ├── launch/nao_chatbot_legacy.launch.py
│       ├── launch/nao_chatbot_skills.launch.py
│       ├── launch/nao_chatbot_skills_asr.launch.py
│       ├── nao_chatbot/
│       │   ├── intent_rules.py
│       │   ├── asr_vosk.py
│       │   ├── laptop_asr.py
│       │   ├── dialogue_manager.py
│       │   ├── mission_controller.py
│       │   ├── chat_skill_client.py
│       │   ├── chat_skill_server.py
│       │   ├── say_skill_client.py
│       │   ├── nao_rqt_bridge.py
│       │   └── ollama_responder.py
│       └── test/unit/
├── .pre-commit-config.yaml
├── requirements-dev.txt
└── README.md
```

## Docker Build Modes

### 1) Overlay build (default, fastest)

Reuses an existing validated runtime image (default `iiia:nao`) and rebuilds local workspace packages:

- `nao_skills`
- `nao_chatbot`
- `nao_posture_bridge`

```bash
./scripts/build_docker.sh overlay iiia:nao
```

Override base image:

```bash
BASE_IMAGE=my_base_image ./scripts/build_docker.sh overlay my_overlay_tag
```

### 2) Full build

Builds from `ros:jazzy-ros-base`, installs ROS4HRI/NAO dependencies, then builds workspace packages.

```bash
./scripts/build_docker.sh full iiia:nao
```

## Run Container

On host:

```bash
xhost +SI:localuser:root
```

Run:

```bash
docker run -it --rm --network host \
  --name nao_ros2 \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  iiia:nao bash
```

Inside container:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
```

When done on host:

```bash
xhost -SI:localuser:root
```

## Launch Chatbot Stack (Skills Enabled by Default)

Show launch arguments:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py --show-args
```

Backend mode example:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  start_naoqi_driver:=true \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0 \
  mission_mode:=backend \
  use_chat_skill:=true \
  chat_skill_server_enabled:=true \
  use_say_skill:=true \
  say_skill_server_enabled:=true \
  use_posture_skill:=true \
  backend_posture_from_response_enabled:=false \
  ollama_enabled:=true \
  backend_fallback_to_rules:=false \
  ollama_model:=llama3.2:1b
```

In `backend` mode, mission controller defaults to `backend_execute_posture_after_response:=true`
so the robot executes posture only after assistant text is received.

Important shell note: never leave trailing spaces after a line-continuation `\`.

## Launch Profiles (Recommended)

Use short profile launches for demos:

```bash
# Legacy/fallback path
ros2 launch nao_chatbot nao_chatbot_legacy.launch.py

# Skills-first path
ros2 launch nao_chatbot nao_chatbot_skills.launch.py

# Skills + Vosk laptop ASR
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  asr_min_words:=2
```

ASR behavior defaults in `skills_asr`:

- Speech guard ON: microphone input is muted while robot speech is active (`/speech`)
- Filler filter ON: one-word fillers (`uh`, `um`, `huh`, ...) are dropped
- Warning throttle ON: repeated microphone overflow warnings are rate-limited

Useful ASR tuning parameters:

```bash
ros2 launch nao_chatbot nao_chatbot_skills_asr.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  asr_block_duration_ms:=350 \
  asr_min_words:=2 \
  asr_status_warn_period_sec:=3.0
```

Each profile still allows `mission_mode:=rules|backend`.

Full profile matrix and examples:

- [`docs/launch_profiles.md`](docs/launch_profiles.md)
- [`docs/ros4hri_skills_audit.md`](docs/ros4hri_skills_audit.md)

## Run Posture Skill Action Server (Phase 1)

Start server:

```bash
ros2 run nao_posture_bridge posture_skill_server_node \
  --ros-args \
  -p nao_ip:=10.10.200.149 \
  -p nao_port:=9559 \
  -p default_speed:=0.85
```

If you see `python qi is missing`, the server now falls back to publishing
`/chatbot/posture_command` (enabled by default via `fallback_to_posture_topic:=true`).
In that case, run the legacy bridge node in parallel:

```bash
ros2 run nao_posture_bridge nao_posture_bridge_node \
  --ros-args -p nao_ip:=10.10.200.149 -p nao_port:=9559
```

Quick check for direct NAOqi Python support in container:

```bash
python3 -c "import qi; print('qi import OK')"
```

Confirm a single posture action server is active (avoid duplicate execution):

```bash
ros2 action info /skill/do_posture
```

Send goal with feedback:

```bash
ros2 action send_goal /skill/do_posture nao_skills/action/DoPosture \
  "{posture_name: 'Sit', speed: 0.85}" \
  --feedback
```

## Quality Gates

Host/VS Code setup (recommended for fast feedback):

```bash
./scripts/setup_dev_tools.sh
```

Run hooks from VS Code integrated terminal:

```bash
./scripts/run_precommit.sh
```

Optional VS Code tasks (if `.vscode/tasks.json` is present in your local workspace):

- `Dev: Setup Tools`
- `Pre-commit: All Files`
- `Tests: Chatbot Unit`

Enforcement:

- `setup_dev_tools.sh` installs both `pre-commit` and `pre-push` git hooks.
- VS Code source-control commits will trigger the same hooks automatically.

Manual equivalent:

```bash
source .venv/bin/activate
python -m pre_commit run --all-files
```

Run unit tests:

```bash
PYTHONPATH=src/nao_chatbot pytest -q src/nao_chatbot/test/unit
```

Local syntax checks (also covered by pre-commit hooks):

```bash
python3 -m py_compile src/nao_chatbot/launch/nao_chatbot_stack.launch.py
python3 -m py_compile src/nao_posture_bridge/nao_posture_bridge/posture_skill_server.py
```

## Known Good Baseline

- `nao_skills` action interfaces build and are discoverable
- `nao_posture_bridge` exports both:
  - `nao_posture_bridge_node` (topic bridge)
  - `posture_skill_server_node` (action server)
- `nao_chatbot` stack supports action-first posture flow with topic fallback
