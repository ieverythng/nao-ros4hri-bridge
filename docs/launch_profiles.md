# Launch Profiles

Last updated: 2026-07-28

This file is the active launch guide. Historical launch notes are under
`docs/artifacts/`.

## Profile Matrix

| Launch file | Default purpose | Notes |
| --- | --- | --- |
| `nao_chatbot_sim.launch.py` | Simulator stack and operator tools | Planner and planner gate on by default; laptop camera on `/camera/image_raw`; full interaction-sim `rqt` perspective and standalone dialogue rqt plugin on; interaction trace viewer off by default (start manually when needed) |
| `nao_chatbot_robot.launch.py` | Real robot camera/RViz/HRI overlays | Planner mode on; robot TF and RViz in profile defaults |
| `nao_chatbot_demo.launch.py` | Sim-only demo with mock scan and demo-oriented defaults | Extends sim profile with demo skills and grounding |
| `nao_chatbot_asr_only.launch.py` | Isolated ASR | No dialogue/planner/executor |

`perform_motion` uses the real motion adapter by default in every profile. Its
head-motion branch publishes an honest open-loop command when no recent head
joint state is available, while convergence-as-success remains disabled. The
controlled fake perform-motion seam remains available for validation. A real
ROS4HRI-compatible `nao_look_at` adapter is installed and launched, but
`look_at` continues to default to its fake action server until the physical
target-frame and reset paths complete the v2 robot validation gate:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  perform_motion_execution_mode:=real \
  look_at_execution_mode:=fake
```

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

Response-first cool-profile validation launch:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  posture_bridge_wake_up_on_connect:=true \
  start_naoqi_driver:=true \
  start_object_detection:=false \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  chatbot_turn_pipeline_mode:=response_first \
  chatbot_grounded_context_digest_enabled:=true \
  start_fake_skills:=true \
  start_interaction_trace_viewer:=true
```

Canonical supervisor-video launch (detector disabled):

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  posture_bridge_wake_up_on_connect:=true \
  start_naoqi_driver:=true \
  start_object_detection:=false \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=172.26.112.143 \
  network_interface:=wlp1s0 \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  chatbot_turn_pipeline_mode:=response_first \
  chatbot_server_url:=http://10.7.138.215:8004/v1/chat/completions \
  planner_llm_provider:=openai_compatible \
  planner_llm_base_url:=http://10.7.138.215:8004 \
  planner_llm_model:=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ \
  planner_llm_api_key_env:=VLLM_API_KEY \
  start_fake_skills:=true \
  start_interaction_trace_viewer:=true \
  start_demo_log_window:=true \
  chatbot_grounded_context_digest_enabled:=false
```

Preloaded semantic environment:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  preloaded_environment_ids:=kitchen_delivery \
  preloaded_environment_lifespan_sec:=3600 \
  start_fake_skills:=true
```

The scoreable state is the KnowledgeCore fixture injected through `/kb/revise`.
The SVG files are rqt-loader-compatible operator aids installed by
`nao_chatbot`; use
`ros2 run nao_chatbot preloaded_environment_viewer` to print the packaged
HTML/SVG viewer path, or add `--open` inside a rebuilt container.

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
- `chatbot_turn_pipeline_mode`: `response_first` for the current demo and
  validation baseline; `intent_first` remains an ablation until it passes the
  same runtime-review holdouts.
- `chatbot_grounded_context_digest_enabled`: defaults to `true` and prepends a
  compact natural-language scene digest before the authoritative
  `grounded_context` JSON. Set it to `false` for JSON-only grounding ablations
  when testing whether lossy digest wording is affecting dialogue or planning.
  After the 7 July launch patch, `grounded_context_digest_enabled` is accepted
  as a compatibility alias. Either flag set to `false` disables the digest, but
  the `chatbot_`-prefixed name remains the canonical demo-script argument.
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
- `dialogue_manager_say_action`: defaults to `/nao/say` in this stack so
  Dialogue Manager talks through `nao_say_skill`.
- Planner dialogue acts are relayed to `dialogue_manager`, which routes their
  structured facts through `chatbot_llm` for user-facing wording and owns the
  single Say dispatch. The removed `planner_dialogue_wording_mode` and
  `planner_completion_wording_mode` parameters are not launch controls in the
  frozen v1 source.
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
- `start_fake_skills`: launches `fake_skills/fake_skill_server`.
- `fake_skill_scenario_file`: optional fake-skill scenario YAML (default uses
  `fake_skills/config/fake_skill_scenarios.yaml` from package share).
- `fake_skill_active_scenario_id`: optional named scenario id applied by
  `fake_skill_server` by default unless a per-request `scenario_id` override is provided.
- `fake_skill_global_mode`: fake-skill policy mode
  (`scenario|always_success|always_fail|every_other|random_seeded`).
- `fake_skill_random_failure_prob`: failure probability for
  `fake_skill_global_mode=random_seeded`.
- `fake_skill_mode_overrides_json`: per-skill override map, e.g.
  `{"find_object":"always_fail"}`.
- `start_interaction_trace_viewer`: launches `interaction_trace_viewer/trace_node`.
- `preloaded_environment_ids`: comma-separated semantic environment fixtures to
  inject into KnowledgeCore at startup.
- `preloaded_environment_fixtures_path`: optional JSON fixture override. Empty
  uses `nao_chatbot/config/preloaded_environments.json`.
- `preloaded_environment_lifespan_sec`: KnowledgeCore lifespan for launch-time
  preloaded facts.
- `preloaded_environment_kb_models`: optional CSV model list for preloaded facts.
- `interaction_trace_compact_mode`: compact terminal output (`true`) or verbose payload view (`false`, default in sim profile so full JSON payloads are visible).
- `interaction_trace_write_jsonl`: writes JSONL traces under `interaction_trace_jsonl_output_dir`.
- `interaction_trace_write_html_on_shutdown`: writes static HTML report on shutdown under `interaction_trace_html_output_dir`.
- `interaction_trace_include_raw_payloads`: keeps raw payload strings in trace events.
- `interaction_trace_max_payload_chars`: max summary chars per rendered event.
- `interaction_trace_include_channels_csv`: optional CSV allowlist for channels.
- `interaction_trace_exclude_channels_csv`: optional CSV denylist for channels.
- `interaction_trace_include_event_types_csv`: optional CSV allowlist for event types.
- `interaction_trace_exclude_event_types_csv`: optional CSV denylist for event types.

## Fake Skill Scenario Switching

Launch with an initial named scenario:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_fake_skills:=true \
  fake_skill_active_scenario_id:=path_blocked \
  fake_skill_global_mode:=scenario
```

Inspect available and active scenario ids:

```bash
ros2 param get /fake_skill_server available_scenario_ids
ros2 param get /fake_skill_server active_scenario_id
```

Switch scenario live (same container session):

```bash
ros2 param set /fake_skill_server active_scenario_id ambiguous_cup
```

Reset to defaults (no named scenario):

```bash
ros2 param set /fake_skill_server active_scenario_id ""
```

Force all fake skills to fail for stress testing:

```bash
ros2 param set /fake_skill_server global_mode always_fail
```

Use deterministic random policy:

```bash
ros2 param set /fake_skill_server global_mode random_seeded
ros2 param set /fake_skill_server random_failure_prob 0.35
```

Override one skill mode without changing others:

```bash
ros2 param set /fake_skill_server mode_overrides_json '{"find_object":"always_fail"}'
```

Semi-interactive selector (same container, same running stack):

```bash
./scripts/fake_skill_scenario_menu.sh /fake_skill_server
```

Demo alternation mode (one run shows both success/failure paths):

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py \
  fake_skill_global_mode:=every_other \
  fake_skill_mode_overrides_json:='{}'
```

Switch global policy live:

```bash
ros2 param set /fake_skill_server global_mode always_success
ros2 param set /fake_skill_server global_mode always_fail
ros2 param set /fake_skill_server global_mode random_seeded
ros2 param set /fake_skill_server random_failure_prob 0.35
```

Override one skill live:

```bash
ros2 param set /fake_skill_server mode_overrides_json '{"find_object":"always_fail"}'
```

If `global_mode` or `mode_overrides_json` is reported as "Parameter not set",
the running container is using an older fake-skills build; rebuild/re-source
before validating runtime policy seams.

If launch fails with `Got dict for "mode_overrides_json"`, the runtime is using
an outdated `fake_skills.launch.py` that does not force string typing for the
JSON override parameter. Rebuild `fake_skills` with the latest launch fix.

Standalone interaction trace viewer (separate window, full JSON payload view):

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
pkill -f interaction_trace_viewer.trace_node || true
ros2 run interaction_trace_viewer trace_node --ros-args \
  -p compact_mode:=false \
  -p include_raw_payloads:=true \
  -p enable_scene_summary_channel:=false \
  -p rosout_min_level:=warn \
  -p rosout_node_allowlist_csv:="chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug" \
  -p include_channels_csv:="planner/request,intents,planner/execution_feedback,planner/dialogue_act,chatbot_llm/turn_trace,fake_skills/events" \
  -p include_event_types_csv:="planner_request,planner_output,execution_feedback,planner_dialogue_act,chatbot_turn_trace,skill_result"
```

## Launch TUI (SocialMinds operator GUI)

`launch_tui` is a terminal GUI around ROS 2 launch that visualizes the launch
graph, node lifecycle, and log stream while a profile is running. It is shipped
as the SocialMinds apt package `socialminds-ros-jazzy-launch-tui` and exposed
as a ros2cli extension:

```bash
ros2 launch_tui <package_name> <launch_file> [launch_arguments...]
```

Common sim profile:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
ros2 launch_tui nao_chatbot nao_chatbot_sim.launch.py
```

Helper wrapper (repo script):

```bash
./scripts/run_launch_tui.sh nao_chatbot nao_chatbot_sim.launch.py
./scripts/run_launch_tui.sh nao_chatbot nao_chatbot_robot.launch.py start_asr:=false
```

Notes:

- Use `ros2 launch_tui`, not `ros2 launch launch_tui ...`.
- The overlay Dockerfiles refresh `socialminds-ros-jazzy-launch-tui` on rebuild
  so the container tracks the latest SocialMinds apt release.
- `textual>=0.50` is pinned in Docker because Ubuntu 24.04's default Textual is
  too old for the current `launch_tui` API.
- Run inside a TTY (`docker exec -it nao_ros2 bash`) so the Textual UI renders
  correctly.

Trace-viewer-first one-copy demo command:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py \
  start_interaction_trace_viewer:=true \
  interaction_trace_compact_mode:=false \
  interaction_trace_include_raw_payloads:=true \
  interaction_trace_include_channels_csv:="planner/request,intents,planner/execution_feedback,planner/dialogue_act,chatbot_llm/turn_trace,fake_skills/events" \
  interaction_trace_include_event_types_csv:="planner_request,planner_output,execution_feedback,planner_dialogue_act,chatbot_turn_trace,skill_result"
```

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
     -> dialogue_manager -> chatbot_llm dialogue-act wording pass -> TTS
```

`chatbot_llm` owns natural language for user-facing planner acknowledgements,
clarifications, failures, and completions. `planner_llm` owns abstract plan
structure and supervision only. `nao_orchestrator` executes deterministic skill
steps and publishes feedback; it should not invent user-facing wording. Executable
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

For a demo that should prefer a live vLLM model and then use a tested Ollama
model when vLLM is unavailable, use the model-priority wrapper:

```bash
./scripts/launch_model_priority_demo.sh \
  posture_bridge_wake_up_on_connect:=true \
  start_naoqi_driver:=true \
  start_managed_ollama:=false
```

The wrapper queries vLLM `/v1/models`, probes an advertised model, and only
then falls back to Ollama `/api/tags` plus a small non-thinking completion.
The default preferences are Qwen3-VL then the observed Qwen3.5 vLLM id, followed
by `gemma4:31b-cloud`, `nemotron-3-super:cloud`, and `gemma4:cloud`. Override
the preferred candidates with `VLLM_MODEL_PREFERENCE` or
`OLLAMA_MODEL_PREFERENCE`. A model passed through `chatbot_model:=`,
`ollama_model:=`, or `planner_llm_model:=` is tried first and becomes the main
CLI override. The resolved model is pinned for the launch, so a slow request
does not cause a mid-run model change. A failed resolver is a hard launch error
with the tested backend diagnostics. When a fallback is used, the wrapper emits
`fallback model used, current model <model> unavailable` and appends a JSONL
event to `${MODEL_SELECTION_LOG:-/tmp/nao_model_selection.jsonl}`.

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
- `posture_allow_open_loop_without_naoqi`: keep real NAOqi posture execution as
  the primary route, but acknowledge validated posture goals as `open_loop`
  when no NAOqi connection exists. Demo profiles enable this so disconnected
  posture requests behave consistently with disconnected absolute head motion.
- `head_motion_allow_open_loop_without_joint_state`: allow validated absolute
  head-motion commands before robot joint state is available.
- `start_nao_look_at`: upstream-style look-at implementation.
- `tts_backend_action_name`: explicit downstream robot TTS action name (empty keeps `/speech` topic fallback).
- `sim_use_laptop_tts`: sim-only helper that reroutes `nao_say_skill` speech to `debug_tts_action_name` (default `false` for sim/robot/demo).

## Docker Demo Path

The project Dockerfiles build `emorobcare_cv_msgs`,
`emorobcare_cv_object_detection`, and `my_game_interface` whenever their source
directories are present. Container startup does not compile these optional
packages by default. Source-mounted development containers may opt in with
`-e AUTO_BUILD_OPTIONAL_WS_PACKAGES=1`; normal demo containers should use the
packages installed in the image.

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
