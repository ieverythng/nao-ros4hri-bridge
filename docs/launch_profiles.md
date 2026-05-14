# Launch Profiles

Last updated: 2026-05-08

This file is the active launch guide. Historical launch notes are under
`docs/artifacts/`.

## Profile Matrix

| Launch file | Default purpose | Notes |
| --- | --- | --- |
| `nao_chatbot_sim.launch.py` | Simulator stack and operator tools | Planner and planner gate on by default; laptop camera on `/camera/image_raw`; `rqt` console on |
| `nao_chatbot_robot.launch.py` | Real robot camera/RViz/HRI overlays | Planner mode on; robot TF and RViz in profile defaults |
| `nao_chatbot_demo.launch.py` | Sim-only demo with mock scan and demo-oriented defaults | Extends sim profile with demo skills and grounding |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR | No dialogue/planner/executor |

## Common Commands

Simulator:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

Simulator with planner opt-out:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=false \
  chatbot_planner_mode_enabled:=false
```

Simulator with object grounding and camera/GStreamer perception:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Simulator with laptop-side TTS playback for robot utterances:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  sim_use_laptop_tts:=true
```

Demo:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py
```

Minimal planner + orchestrator harness (sim profile, most runtime nodes off):

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_chatbot_llm:=false \
  start_dialogue_manager:=false \
  start_knowledge_core:=false \
  start_interaction_sim:=false \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=false \
  start_object_detection:=false \
  start_scene_grounding:=false \
  start_rqt_console:=false \
  start_robot_speech_debug:=false
```

Planner fixtures (optional):

```bash
ros2 run planner_llm publish_fixture request
ros2 run planner_llm publish_fixture feedback
```

Robot:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py nao_ip:=<robot_ip>
```

Robot with object grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip> \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

ASR-only:

```bash
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

## Key Planner Arguments

- `start_planner_llm`: starts `planner_llm`.
- `chatbot_planner_mode_enabled`: makes `chatbot_llm` publish `/planner/request`.
- `scan_result_mode`: deterministic scan skill result mode (`success` or
  `failure`) for no-robot validation.
- `scan_summary`: success summary returned by the scan skill.
  Targeted scans only use this as final factual content when it is attached as
  an explicit scan-step summary; otherwise they report that no confirmed target
  detection was available.
- `scan_report_after_success`: defaults to `false` so scan completion wording
  routes through `chatbot_llm` instead of being spoken directly by
  `nao_orchestrator`.
- `planner_request_topic`: defaults to `/planner/request`.
- `planner_request_intent`: defaults to `planner_request`.
- `planner_dialogue_act_topic`: defaults to `/planner/dialogue_act`.
- `planner_skill_registry_path`: optional planner skill registry overlay.
- `planner_llm_provider`: `ollama` by default.
- `planner_llm_model`: planner model name.
- `chatbot_server_url`: full chatbot backend chat endpoint, for example
  `http://127.0.0.1:11434/api/chat`.
- `planner_llm_base_url`: planner provider base URL, for example
  `http://127.0.0.1:11435`. For the Ollama provider, `planner_llm`
  appends `/api/chat`.
- `planner_llm_default_retry_budget`: default plan retry budget.
- `planner_llm_auto_replan`: enables supervisor auto-replan policy.
- `planner_workbench_enabled`: disabled by default; when enabled, `planner_llm`
  tries an optional Neural Workbench candidate program before provider planning.
- `planner_workbench_required`: disabled by default; when enabled, Workbench
  unavailability or invalid candidates become planner failures instead of
  provider fallbacks.
- `planner_workbench_desired_ab_level`: desired abstraction level passed to the
  Workbench selector.
- `planner_workbench_python_path`: optional `os.pathsep`-separated Python paths
  for external Workbench packages when they are not installed in the active ROS
  environment.
- `planner_workbench_trace_candidates`: attaches Workbench candidate-program
  metadata to planner output for research logging.

## ASR And Perception Startup

Main profiles expose ASR arguments but keep ASR disabled by default. Pass
`start_asr:=true` when local speech recognition is desired, or use
`nao_chatbot_asr_only.launch.py` for isolated ASR testing.

The interaction-sim tools can run without camera/GStreamer perception, but
`gscam` itself is part of `start_interaction_sim_perception`. `start_naoqi_driver`
does not start GScam.

## Planner Dialogue Flow

Execution-oriented turns use this ownership split:

```text
user -> dialogue_manager -> chatbot_llm -> nao_orchestrator planner gate
     -> planner_llm -> nao_orchestrator -> skills
     -> planner feedback -> planner_llm dialogue act
     -> dialogue_manager -> chatbot_llm completion pass -> TTS
```

`chatbot_llm` owns natural language for the initial acknowledgement and final
task-relative utterance. `planner_llm` owns abstract plan structure and
supervision only. `nao_orchestrator` executes deterministic skill steps and
publishes feedback; it should not invent user-facing wording. Executable
planner outputs must not include `say` steps. If the planner model mixes
speech with robot actions, `planner_llm` rejects the output and retries with
validation feedback.

For the local split-endpoint experiment, start two host-side Ollama servers
before launching ROS. This must run on the host, not from inside the container,
so both servers share your normal Ollama cloud identity and model access:

```bash
./scripts/start_demo_ollama_endpoints.sh
```

The script starts or reuses:

- chatbot endpoint: `http://127.0.0.1:11434/api/chat`
- planner endpoint: `http://127.0.0.1:11435`

It also probes the configured model on both endpoints before returning. The
default demo model is `gemma4:31b-cloud`; override with `OLLAMA_MODEL=...` if
needed. On machines where the authenticated cloud models belong to the system
`ollama` service user, the script starts the planner endpoint with
`sudo -u ollama` so it can read the same model manifests and cloud identity as
the existing `11434` service.

Then launch with the profile defaults:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py
```

Equivalent manual startup:

```bash
OLLAMA_HOST=127.0.0.1:11434 ollama serve
sudo -u ollama env HOME=/usr/share/ollama \
  OLLAMA_HOST=127.0.0.1:11435 \
  OLLAMA_MODELS=/usr/share/ollama/.ollama/models \
  ollama serve
```

Launch can also start container-managed Ollama servers with
`start_managed_ollama:=true`, but keep that disabled for cloud models unless you
have explicitly copied/provisioned the same Ollama identity inside the container.
A fresh container-managed server can fail cloud requests with `401 Unauthorized`.

Then launch with explicit endpoints when you do not want profile defaults:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py \
  chatbot_server_url:=http://127.0.0.1:11434/api/chat \
  planner_llm_base_url:=http://127.0.0.1:11435
```

For a vLLM or other OpenAI-compatible backend, first probe the API:

```bash
./scripts/probe_vllm_chat.py --lab-pc --list-models
./scripts/probe_vllm_chat.py \
  --base-url http://<vllm-host>:8004 \
  --model <served-model-name>
```

Then route the planner to vLLM:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://<vllm-host>:<port> \
  planner_llm_model:=<served-model-name> \
  planner_llm_api_key_env:=VLLM_API_KEY
```

To route both chatbot and planner to the current lab PC endpoint:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py \
  chatbot_server_url:=http://10.7.138.215:8004/v1/chat/completions \
  ollama_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://10.7.138.215:8004 \
  planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_api_key_env:=VLLM_API_KEY
```

## Demo Log Window

Demo profiles can start a filtered operator console that only shows the
dialogue/planner/executor path and ignores noisy perception, KnowledgeCore,
face, object-detection, and visualization nodes by default. In operation it also
prints concise summaries of `/intents`, `/planner/request`,
`/planner/execution_feedback`, `/planner/dialogue_act`, and
`/dialogue_manager/closed_captions`, so intent routing, planner requests,
planner decisions, and robot speech are visible without opening raw topic
echoes.

- `start_demo_log_window`: starts `nao_chatbot demo_rosout_filter`.
- `demo_log_nodes`: comma-separated node allowlist.
- `demo_log_min_level`: minimum severity (`debug`, `info`, `warn`, `error`, or
  `fatal`).

Standalone:

```bash
ros2 run nao_chatbot demo_rosout_filter \
  --nodes chatbot_llm,planner_llm,nao_orchestrator,dialogue_manager,nao_say_skill,head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug \
  --min-level info
```

Use `--no-topics` if you only want the filtered `/rosout` stream.

## Key Grounding Arguments

- `start_object_detection`: starts selected detector backend.
- `object_detection_backend`: `emorobcare_cv` or `yolo_ros`.
- `object_detection_input_image_topic`: camera image topic for detector.
- `object_detection_threshold`: detector confidence threshold.
- `start_scene_grounding`: starts `nao_scene_grounding`.
- `scene_grounding_detector_topic`: detector output consumed by grounding.
- `scene_grounding_summary_topic`: summary output, normally `/scene/summary`.
- `scene_grounding_allowed_labels`: object allowlist.
- `scene_grounding_knowledge_lifespan_sec`: transient KB fact lifetime.

## Key Robot Arguments

- `nao_ip`: canonical robot IP argument forwarded through robot launch surfaces.
- `start_nao_robot`: packaged robot bring-up.
- `start_naoqi_driver`: NAOqi driver bring-up.
- `start_rviz`: RViz view for robot/camera/TF validation.
- `start_nao_orchestrator`: deterministic executor.
- `start_nao_say_skill`: robot-side speech hook.
- `start_nao_replay_motion`: replay/posture/head-motion skills.
- `start_nao_look_at`: upstream-style look-at implementation.
- `tts_backend_action_name`: explicit downstream robot TTS action name (empty keeps `/speech` topic fallback).
- `sim_use_laptop_tts`: sim-only helper that reroutes `nao_say_skill` speech to `debug_tts_action_name` (default `false` for sim/robot/demo).

## Docker Demo Path

Preferred overlay image:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Laptop camera:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

## Show Arguments

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_robot.launch.py --show-args
ros2 launch nao_chatbot nao_chatbot_demo.launch.py --show-args
```
