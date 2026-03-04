# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI + chatbot orchestration with canonical skill-action interfaces.

This repository uses `src/` as the canonical source tree.

## Migration Status (As of March 3, 2026)

- Canonical communication skills are active:
  - `/skill/say` -> `communication_skills/action/Say`
  - `/skill/chat` -> `communication_skills/action/Chat`
- NAO-specific skill remains:
  - `/skill/do_posture` -> `nao_skills/action/DoPosture`
- Compatibility endpoints removed:
  - `/skill/say_text_compat` removed
  - `/skill/chat_compat` removed
- Local alias wrappers removed:
  - `nao_chatbot.dialogue_manager`
  - `nao_chatbot.nao_rqt_bridge`

## Current Runtime Architecture

Presentation diagram: [`docs/whiteboard_migration_diagram.md`](docs/whiteboard_migration_diagram.md)

Main nodes:

- `dialogue_manager_node` (package `dialogue_manager`): turn orchestration (`LiveSpeech` in, `/chatbot/user_text` out, `/skill/say` dispatch)
- `asr_vosk_node`: local microphone ASR (Vosk) -> `LiveSpeech`
- `mission_controller_node`: intent/routing logic (`rules` or `backend`) + `/skill/chat` client
- `ollama_chatbot_node`: `/skill/chat` action server backed by Ollama
- `nao_posture_bridge_node`: posture topic bridge (`/chatbot/posture_command` -> NAOqi `ALRobotPosture`)
- `posture_skill_server_node`: action posture bridge (`/skill/do_posture` -> NAOqi `ALRobotPosture`)
- `say_skill_server_node`: action speech bridge (`/skill/say` -> `/tts_engine/tts`)

Active flows today:

```text
Speech/Text input
  -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
  -> dialogue_manager_node -> /chatbot/user_text
  -> mission_controller_node
       rules mode   -> /chatbot/assistant_text
       backend mode -> /skill/chat (communication_skills/action/Chat)
                    -> ollama_chatbot_node (two-stage: response then intent extraction)
                    -> /chatbot/assistant_text + /chatbot/intent
       posture intent -> /skill/do_posture (DoPosture action)
  -> posture_skill_server_node -> ALRobotPosture.goToPosture
     fallback (if qi unavailable) -> /chatbot/posture_command -> nao_posture_bridge_node -> ALRobotPosture.goToPosture
  -> dialogue_manager_node -> /skill/say (communication_skills/action/Say) -> say_skill_server_node -> /tts_engine/tts
  -> dialogue_manager_node -> /speech (compatibility + ASR guard pathway)
```

## LLM Intent Pipeline (Template-Aligned, Two-Stage)

`ollama_chatbot_node` uses a two-stage pipeline inspired by the ROS4HRI chatbot
template:

1. Stage 1 (`response`): LLM generates natural acknowledgement/answer (`verbal_ack`)
2. Stage 2 (`intent`): LLM extracts structured `user_intent` from user text + stage-1 reply

`mission_controller_node` remains the action authority:

- publishes assistant text
- publishes normalized intent
- decides posture execution from normalized intent

Normalized intent labels expected by mission controller:

- `posture_stand`
- `posture_sit`
- `posture_kneel`
- `greet`
- `identity`
- `wellbeing`
- `help`
- `fallback`

Chat skill intent modes:

- `ollama_intent_detection_mode:=llm_with_rules_fallback` (default)
  - use LLM stage-2 intent, fallback to rule intent if intent extraction fails
- `ollama_intent_detection_mode:=llm`
  - strict LLM-only intent extraction
- `ollama_intent_detection_mode:=rules`
  - bypass LLM intent extraction and resolve intent from rules

Turn traceability (`turn_id`):

- `dialogue_manager_node` assigns a monotonic `turn_id` and forwards it with `/chatbot/user_text`
- `mission_controller_node` propagates `turn_id` into `/skill/chat` goal configuration
- `chat_skill_server` returns the same `turn_id` in `Chat.Result.role_results`
- logs in dialogue, mission, and chat skill nodes include `[turn:<id>]` for end-to-end tracing

Modular backend components:

- `chat_skill_server.py`: `/skill/chat` action server lifecycle/wiring
- `chat_turn_engine.py`: two-stage execution policy (`response` then `intent`)
- `chat_config.py`: ROS parameter declarations and config loading
- `chat_goal_codec.py`: canonical goal/result payload handling
- `chat_history.py`: history serialization and trimming
- `chat_prompts.py`: stage-specific prompt builders
- `prompt_pack.py`: YAML prompt-pack + schema defaults/loader
- `ollama_transport.py`: Ollama HTTP transport adapter
- `skill_catalog.py`: prompt-time skill-catalog extraction from package manifests

Prompt/schema editing:

- Default prompt pack file:
  - `src/nao_chatbot/config/chat_prompt_pack.yaml`
- Launch arg to override file:
  - `ollama_prompt_pack_path:=/absolute/path/to/chat_prompt_pack.yaml`
- Prompt pack content controls:
  - `system_prompt`
  - `response_prompt_addendum`
  - `intent_prompt_addendum`
  - `response_schema`
  - `intent_schema`
- Skill-catalog prompt injection defaults:
  - `ollama_use_skill_catalog:=true`
  - `ollama_skill_catalog_packages:=communication_skills,nao_skills`
  - `ollama_skill_catalog_max_entries:=16`
  - `ollama_skill_catalog_max_chars:=3000`

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
│   ├── run_tests.sh
│   ├── setup_dev_tools.sh
│   └── run_precommit.sh
├── src/
│   ├── communication_skills/
│   │   ├── action/Ask.action
│   │   ├── action/Say.action
│   │   ├── action/Chat.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── dialogue_manager/
│   │   ├── dialogue_manager/
│   │   ├── launch/dialogue_manager.launch.py
│   │   ├── setup.py
│   │   └── package.xml
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
│   │   ├── nao_posture_bridge/say_skill_server.py
│   │   ├── src/nao_posture_bridge_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── nao_chatbot/
│       ├── launch/nao_chatbot_stack.launch.py
│       ├── launch/nao_chatbot_skills.launch.py
│       ├── launch/nao_chatbot_skills_asr.launch.py
│       ├── nao_chatbot/
│       │   ├── intent_rules.py
│       │   ├── asr_vosk.py
│       │   ├── mission_controller.py
│       │   ├── chat_skill_client.py
│       │   ├── chat_skill_server.py
│       │   ├── ollama_chatbot.py
│       │   ├── say_skill_client.py
│       │   └── ...
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
  nao_ip:=172.26.112.62 \
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
  ollama_model:=llama3.2:1b \
  ollama_intent_detection_mode:=llm_with_rules_fallback
```

In `backend` mode, mission controller defaults to `backend_execute_posture_after_response:=true`
so the robot executes posture only after assistant text is received.

To force strict rules behavior while still using backend wiring:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  mission_mode:=backend \
  ollama_enabled:=false \
  ollama_intent_detection_mode:=rules
```

To try `gpt-oss` again later:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  mission_mode:=backend \
  ollama_enabled:=true \
  ollama_model:=gpt-oss:20b-cloud
```

Important shell note: never leave trailing spaces after a line-continuation `\`.

## Launch Profiles (Recommended)

Use short profile launches for demos:

```bash
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
  -p nao_ip:=172.26.112.62 \
  -p nao_port:=9559 \
  -p default_speed:=0.85
```

If you see `python qi is missing`, the server now falls back to publishing
`/chatbot/posture_command` (enabled by default via `fallback_to_posture_topic:=true`).
In that case, run the posture topic bridge node in parallel:

```bash
ros2 run nao_posture_bridge nao_posture_bridge_node \
  --ros-args -p nao_ip:=172.26.112.62 -p nao_port:=9559
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

Run the local verification suite directly:

```bash
./scripts/run_tests.sh
```

`run_tests.sh` always runs syntax checks + `nao_chatbot` unit tests, and runs
`dialogue_manager` unit tests automatically when `chatbot_msgs` is available in
the active shell.

To enforce strict full-suite behavior (fail if `chatbot_msgs` is missing):

```bash
./scripts/run_tests.sh --strict-dialogue
```

Enforcement:

- `setup_dev_tools.sh` installs both `pre-commit` and `pre-push` git hooks.
- VS Code source-control commits will trigger the same hooks automatically.

Manual equivalent:

```bash
source .venv/bin/activate
python -m pre_commit run --all-files
```

Run only chatbot unit tests:

```bash
PYTHONPATH=src/nao_chatbot pytest -q src/nao_chatbot/test/unit
```

Local syntax checks (also covered by pre-commit hooks):

```bash
python3 -m py_compile src/nao_chatbot/launch/nao_chatbot_stack.launch.py
python3 -m py_compile src/nao_posture_bridge/nao_posture_bridge/posture_skill_server.py
```

## Known Good Baseline

- `nao_skills` posture action interface builds and is discoverable
- `nao_posture_bridge` exports both:
  - `nao_posture_bridge_node` (topic bridge)
  - `posture_skill_server_node` (action server)
- `nao_chatbot` stack supports action-first posture flow with topic fallback
