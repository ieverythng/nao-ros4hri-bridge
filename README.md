# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI + modular chatbot orchestration.

This repository uses `src/` as the canonical source tree and builds the runtime image `iiia:nao`.

## Current Architecture

Main nodes:

- `nao_rqt_bridge_node`: ROS4HRI I/O bridge (`LiveSpeech` in, TTS + `/speech` out)
- `mission_controller_node`: intent/routing logic (`rules` or `backend`)
- `ollama_responder_node`: Ollama backend client with prompt/context controls
- `ollama_node`: legacy compatibility entrypoint (bridge + mission controller)

High-level flow:

```text
User/rqt_chat -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
  -> nao_rqt_bridge_node -> /chatbot/user_text
  -> mission_controller_node
       rules mode   -> /chatbot/assistant_text
       backend mode -> /chatbot/backend/request -> ollama_responder_node -> /chatbot/backend/response -> /chatbot/assistant_text
  -> nao_rqt_bridge_node -> /tts_engine/tts + /speech
```

Extra control channel:

- `mission_controller_node` publishes posture commands to `/chatbot/posture_command` (`stand`, `sit`, `kneel`).

## Repository Layout

```text
.
├── docker/
│   ├── Dockerfile
│   └── ros_entrypoint.sh
├── scripts/
│   └── build_docker.sh
├── src/
│   └── nao_chatbot/
│       ├── launch/nao_chatbot_stack.launch.py
│       ├── nao_chatbot/
│       │   ├── nao_rqt_bridge.py
│       │   ├── mission_controller.py
│       │   ├── ollama_responder.py
│       │   └── ollama_node.py
│       ├── package.xml
│       └── setup.py
└── README.md
```

## Docker

Build:

```bash
./scripts/build_docker.sh
```

Build output tag:

```bash
iiia:nao
```

Current Docker strategy:

- `docker/Dockerfile` extends the proven local base image `iiia:nao`
- overlays repo `src/` into `/home/ubuntu/ws/src`
- rebuilds `nao_chatbot` package only

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

## Launch Stack

Show launch arguments:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py --show-args
```

### Rules mode (deterministic)

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  start_naoqi_driver:=true \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0 \
  mission_mode:=rules \
  ollama_enabled:=false
```

### Backend mode (Ollama-first)

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  start_naoqi_driver:=true \
  nao_ip:=10.10.200.149 \
  network_interface:=wlp1s0 \
  mission_mode:=backend \
  ollama_enabled:=true \
  backend_fallback_to_rules:=false \
  ollama_model:=llama3.2:1b \
  ollama_context_window_tokens:=4096 \
  ollama_first_request_timeout_sec:=60.0 \
  ollama_request_timeout_sec:=20.0
```

## Key Launch Arguments (Current)

Core:

- `start_naoqi_driver`
- `nao_ip`
- `nao_port`
- `network_interface`
- `qi_listen_url`
- `mission_mode` (`rules` | `backend`)

Mission controller:

- `backend_fallback_to_rules` (default `false`)
- `backend_response_timeout_sec` (default `30.0`)
- `posture_command_topic` (default `/chatbot/posture_command`)

Ollama responder:

- `ollama_enabled`
- `ollama_model` (default `llama3.2:1b`)
- `ollama_url` (default `http://localhost:11434/api/chat`)
- `ollama_context_window_tokens` (maps to Ollama `num_ctx`)
- `ollama_request_timeout_sec`
- `ollama_first_request_timeout_sec`
- `ollama_temperature`
- `ollama_top_p`
- `ollama_robot_name`
- `ollama_persona_prompt_path` (optional file prompt)
- `ollama_prompt_addendum` (inline extra instruction)

## Prompt and Personality

`ollama_responder` supports layered prompt composition:

1. optional persona file (`ollama_persona_prompt_path`)
2. templated `system_prompt` (supports `{robot_name}`)
3. optional `prompt_addendum`

It also supports periodic identity reinforcement (`identity_reminder_every_n_turns`) to keep long-horizon personality consistency.

## Context Window

Context window is explicitly configurable through:

- `ollama_context_window_tokens` -> Ollama `options.num_ctx`

So yes, context window is now controlled by launch args (recommended for Docker workflows).

## rqt_chat Launch

Inside container:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
ros2 run rqt_gui rqt_gui --standalone rqt_chat.chat.ChatPlugin --force-discover
```

If display fails:

```bash
xhost +SI:localuser:root
```

## Development Loop

Edit:

```text
src/nao_chatbot/
```

Rebuild package in running container:

```bash
docker exec -it nao_ros2 bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /home/ubuntu/ws && colcon build --symlink-install --packages-select nao_chatbot'
```

Optional snapshot:

```bash
docker commit nao_ros2 iiia:nao
```

## Current Known-Good Status

- rqt_chat works with both rules and backend modes
- Ollama integration works with `llama3.2:1b`
- first-request warmup handled with longer timeout
- no mandatory hard locks for duplicate launches
- posture command extraction scaffold is live (`/chatbot/posture_command`)

## Next Steps Roadmap

Near-term:

1. Wire `/chatbot/posture_command` to actual NAO posture actions (`stand`, `sit`, `kneel`).
2. Add structured LLM output mode (intent JSON) so mission controller consumes typed intents, not only keyword heuristics.
3. Add dedicated launch profile presets (`rules_demo`, `backend_demo`, `backend_strict_no_fallback`).

Audio / multimodal:

4. Add NAO microphone/ASR ingestion path into `/chatbot/user_text`.
5. Tag inputs by source (`typed`, `asr`) for logs and evaluation.
6. Optionally mirror ASR inputs into rqt_chat UI for operator visibility.

Research-oriented (IIIA-05 alignment):

7. Integrate symbolic context (knowledge base facts) into prompt builder.
8. Add intent-to-plan intermediary (goal + slots) before action execution.
9. Add evaluation logging (latency, fallback count, success, clarification rate).

Handoff note:

- This README is the current operational baseline for continuing work in another assistant/chat environment.

