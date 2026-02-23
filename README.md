# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for NAO + ROS4HRI + modular chatbot orchestration.

This repository uses `src/` as the canonical source tree.

## Current Architecture

Presentation-ready diagram: [`docs/architecture_diagram.md`](docs/architecture_diagram.md)

Main nodes:

- `nao_rqt_bridge_node`: ROS4HRI I/O bridge (`LiveSpeech` in, TTS + `/speech` out)
- `laptop_asr_node`: local microphone ASR (Vosk) -> `LiveSpeech`
- `mission_controller_node`: intent/routing logic (`rules` or `backend`)
- `ollama_responder_node`: Ollama backend client with prompt/context controls
- `nao_posture_bridge_node`: posture command bridge (`/chatbot/posture_command` -> NAOqi `ALRobotPosture.goToPosture`)
- `ollama_node`: legacy compatibility entrypoint (bridge + mission controller)

High-level flow:

```text
Operator text or laptop mic ASR
  -> /humans/voices/anonymous_speaker/speech (LiveSpeech)
  -> nao_rqt_bridge_node -> /chatbot/user_text
  -> mission_controller_node
       rules mode   -> /chatbot/assistant_text
       backend mode -> /chatbot/backend/request -> ollama_responder_node -> /chatbot/backend/response -> /chatbot/assistant_text
       posture cmd  -> /chatbot/posture_command -> nao_posture_bridge_node -> ALRobotPosture.goToPosture
  -> nao_rqt_bridge_node -> /tts_engine/tts + /speech
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
│   └── import_docker_image.sh
├── src/
│   ├── nao_chatbot/
│   │   ├── launch/nao_chatbot_stack.launch.py
│   │   ├── nao_chatbot/
│   │   │   ├── asr_utils.py
│   │   │   ├── intent_rules.py
│   │   │   ├── laptop_asr.py
│   │   │   ├── mission_controller.py
│   │   │   ├── nao_rqt_bridge.py
│   │   │   └── ollama_responder.py
│   │   └── test/unit/
│   └── nao_posture_bridge/
├── .pre-commit-config.yaml
├── requirements-dev.txt
└── README.md
```

## Docker Build Modes

### 1) Overlay build (default, fastest)

Reuses an existing validated base image (default `iiia:nao`) and rebuilds local packages.

```bash
./scripts/build_docker.sh overlay iiia:nao
```

You can override the base image:

```bash
BASE_IMAGE=my_base_image ./scripts/build_docker.sh overlay my_overlay_tag
```

### 2) Full build 

Builds from `ros:jazzy-ros-base`, adds ROS4HRI/NAO apt dependencies, and builds workspace packages.

```bash
./scripts/build_docker.sh full iiia:nao
```

## Share Prebuilt Image (Optional)

Export:

```bash
./scripts/export_docker_image.sh iiia:nao docker/iiia_nao_image.tar.gz
```

Import on teammate machine:

```bash
./scripts/import_docker_image.sh docker/iiia_nao_image.tar.gz
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

### Backend mode (Ollama)

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

### Laptop microphone ASR mode

`laptop_asr_node` is disabled by default. Enable it and provide a local Vosk model path:

```bash
ros2 launch nao_chatbot nao_chatbot_stack.launch.py \
  laptop_asr_enabled:=true \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15 \
  start_rqt_chat:=false
```

Useful ASR args:

- `asr_output_speech_topic` (default `/humans/voices/anonymous_speaker/speech`)
- `asr_device_index` (default `-1`, system default mic)
- `asr_sample_rate_hz` (default `16000`)
- `asr_block_duration_ms` (default `250`)
- `asr_publish_partial` (default `false`)

## Quality Gates

Install dev tooling:

```bash
python3 -m pip install -r requirements-dev.txt
pre-commit install
```

Run hooks manually:

```bash
pre-commit run --all-files
```

Run unit tests directly:

```bash
PYTHONPATH=src/nao_chatbot pytest -q src/nao_chatbot/test/unit
```

## Robot Audio Integration Notes

Current local ASR publishes ROS4HRI `LiveSpeech`, so the rest of the stack is unchanged.

For robot microphone integration, keep the same downstream topic contract and add an adapter from robot audio/listen outputs to `LiveSpeech`.

References:

- ROS4HRI voice convention (`/humans/voices/<voice_id>/audio`, `/speech`): <https://www.ros.org/reps/rep-0155.html>
- NAOqi driver exposes speech-related actions including `listen`: <https://github.com/ros-naoqi/naoqi_driver2>

## Current Known-Good Status

- Full stack launches offline without robot when NAO-dependent nodes are disabled
- Rules + backend mission modes are available
- Ollama integration still works with configurable context and timeouts
- Posture command bridge remains active
- Laptop ASR pipeline is now integrated and optional
