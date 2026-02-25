# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI + chatbot orchestration with an ongoing migration to skill actions.

This repository uses `src/` as the canonical source tree.

## Migration Status (As of February 23, 2026)

- Phase 0 complete: `nao_skills` action interface package exists with `DoPosture.action`.
- Phase 1 complete: `posture_skill_server_node` is available in `nao_posture_bridge`.
- Phase 2 complete: mission controller can dispatch posture via `/skill/do_posture`.
- Backend-first posture timing: in `backend` mode, posture execution is deferred until backend response by default.
- Backward compatibility preserved: topic fallback remains active when action server is unavailable.

## Current Runtime Architecture

Presentation diagram (legacy topic flow): [`docs/architecture_diagram.md`](docs/architecture_diagram.md)

Main nodes:

- `nao_rqt_bridge_node`: ROS4HRI I/O bridge (`LiveSpeech` in, TTS + `/speech` out)
- `asr_vosk_node`: local microphone ASR (Vosk) -> `LiveSpeech`
- `mission_controller_node`: intent/routing logic (`rules` or `backend`)
- `ollama_responder_node`: Ollama backend client with prompt/context controls
- `nao_posture_bridge_node`: legacy topic posture bridge (`/chatbot/posture_command` -> NAOqi `ALRobotPosture`)
- `posture_skill_server_node`: action posture bridge (`/skill/do_posture` -> NAOqi `ALRobotPosture`)

Active flows today:

```text
Speech/Text input
  -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
  -> nao_rqt_bridge_node -> /chatbot/user_text
  -> mission_controller_node
       rules mode   -> /chatbot/assistant_text
       backend mode -> /chatbot/backend/request -> ollama_responder_node -> /chatbot/backend/response -> /chatbot/assistant_text
       posture intent -> /skill/do_posture (DoPosture action)
  -> posture_skill_server_node -> ALRobotPosture.goToPosture
     fallback (if qi unavailable) -> /chatbot/posture_command -> nao_posture_bridge_node -> ALRobotPosture.goToPosture
  -> nao_rqt_bridge_node -> /tts_engine/tts + /speech
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
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── std_skills/
│   │   ├── msg/
│   │   ├── srv/
│   │   ├── action/
│   │   └── package.xml
│   ├── nao_posture_bridge/
│   │   ├── nao_posture_bridge/posture_skill_server.py
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
│       │   ├── mission_controller.py
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
  use_posture_skill:=true \
  backend_posture_from_response_enabled:=false \
  ollama_enabled:=true \
  backend_fallback_to_rules:=false \
  ollama_model:=llama3.2:1b \
  ollama_context_window_tokens:=4096 \
  ollama_first_request_timeout_sec:=60.0 \
  ollama_request_timeout_sec:=20.0
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
