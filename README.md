# NAO ROS4HRI Bridge

ROS 2 Jazzy workspace for the NAO ROS4HRI migration. The active tree mixes
local robot-side runtime packages, fork-tracked upstream dialogue packages,
local interface packages, and optional external KnowledgeCore/simulator
dependencies.

## Workspace Layout

Robot-side runtime packages in this repo:

- `nao_chatbot`: launch surfaces and operator utilities
- `nao_orchestrator`: downstream `/intents` consumer and NAO skill dispatcher
- `kb_skills`: dedicated KnowledgeCore client boundary and KB skill metadata
- `planner_common`: shared planner request, plan-envelope, and feedback contracts
- `planner_llm`: planner node that turns `/planner/request` into executable `/intents`
- `nao_world_model_enricher`: planner-facing action-conditioned world model that turns scene summaries, KB rows, and execution feedback into enriched context
- `nao_say_skill`: NAO-specific `/nao/say` execution bridge
- `nao_replay_motion`: replay-motion, posture compatibility, and head motion
- `nao_look_at`: NAO implementation of `interaction_skills/look_at`
- `nao_scene_grounding`: detector-to-KnowledgeCore grounding bridge and scene summary node
- `asr_vosk`: local lifecycle ASR node
- `simple_audio_capture`: local microphone source for the ASR path

Fork-tracked upstream runtime repos carried locally:

- `dialogue_manager`: canonical owner of `/skill/chat`, `/skill/ask`, and
  `/skill/say`
- `chatbot_llm`: backend dialogue contract using the local Ollama-based
  response pipeline plus direct-or-planner routing

Interface-only packages shipped in the workspace:

- `communication_skills`
- `nao_skills`
- `motions_skills`
- `std_skills`

Optional external integrations used for KB/simulator testing:

- `knowledge_core`
- `kb_msgs`
- `interaction_sim`
- `oro`

Those KnowledgeCore and simulator packages are not vendored into the active
`src/` tree here. Runtime normally uses the official SocialMinds Jazzy
packages, while `./scripts/bootstrap_socialminds_sources.sh` clones
reference-only copies into `ref_src/knowledge_sources/`.

## Dialogue And KB Grounding

Current migrated direct-execution flow:

```text
speech input -> dialogue_manager -> chatbot_llm
    -> /intents -> nao_orchestrator
    -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

Optional planner-enabled flow:

```text
speech input -> dialogue_manager -> chatbot_llm
    -> /planner/request -> planner_llm
scene grounding + KB + execution feedback -> nao_world_model_enricher
    -> /world_model/enriched_snapshot + /world_model/enriched_text -> planner_llm
    -> /intents -> nao_orchestrator
    -> /planner/execution_feedback -> planner_llm + nao_world_model_enricher
    -> /nao/say | /skill/replay_motion | /skill/do_head_motion | /skill/look_at
```

This split is deliberate:

- `dialogue_manager` owns dialogue state and canonical communication skills
- `chatbot_llm` owns model interaction, grounded dialogue turns, and planner-mode routing
- `nao_world_model_enricher` owns short-horizon world-state enrichment for planner-facing context
- `planner_llm` owns planner request interpretation, executable plan generation, and replan decisions
- `nao_orchestrator` stays downstream-only and dispatches robot-side intents
- `knowledge_core` remains an upstream symbolic store accessed through public
  ROS APIs

Planner mode currently needs two launch flags together:

- `start_planner_llm:=true`
- `chatbot_planner_mode_enabled:=true`

`nao_ip` is the canonical robot-IP argument in the launch surface. It is the
single override forwarded to `naoqi_driver`, the replay-motion launch, and the
temporary posture bridge.

Knowledge grounding is local to `chatbot_llm`, not to `knowledge_core`
itself:

- `knowledge_core` exposes `/kb/query` through `kb_msgs/srv/Query`
- the local `kb_skills` package is the dedicated ROS-facing client boundary for
  KnowledgeCore interactions
- `chatbot_llm` uses `kb_skills` to call `/kb/query` once per response turn
- the returned JSON bindings are formatted into a bounded text snapshot
- that snapshot is appended to the response and intent prompts as grounded
  scene context

The default query shipped by `chatbot_llm` is visibility-focused:

- `myself sees ?entity && ?entity rdf:type ?type`

If a dialogue role passes a JSON `knowledge_snapshot` block,
`chatbot_llm` merges those role-level overrides with the node defaults for
patterns, variables, models, and output limits.

## Object Grounding And Scene Memory

The migrated runtime now also supports a detector-first grounding path for
demo-time object awareness:

```text
camera image -> external detector backend
    -> nao_scene_grounding
    -> /kb/revise + /scene/summary
    -> knowledge snapshots inside chatbot_llm prompts
```

High-level ownership split:

- the external detector owns raw object detection and debug images
- `nao_scene_grounding` normalizes detector outputs, refreshes transient KB
  facts, and publishes a compact JSON scene summary
- `chatbot_llm` keeps using `knowledge_core` as the read-only source for
  grounded scene context in response and intent prompts
- `nao_orchestrator` consumes enriched intent metadata such as `ack_text`,
  `scene_targets`, and optional execution `plan` steps without owning prompt
  logic

Two detector backends are supported behind the same grounding seam:

- `emorobcare_cv_object_detection`: emorobcare object detection package publishing
  `/detected_objects` and optional `/debug/object_detection`
- `yolo_ros`: fallback path publishing `/yolo/tracking` and
  `/yolo/debug_image`

The launch default is `emorobcare_cv`, but the grounding node stays backend
agnostic so the detector can be swapped later without rewriting the KB bridge.

Current emorobcare object detection expectations:

- keep `emorobcare_cv_object_detection` and `emorobcare_cv_msgs` available in
  the active workspace when you use the `emorobcare_cv` backend
- keep `use_knowledge_base: false` in the detector package so
  `nao_scene_grounding` remains the single writer of detector-derived KB facts
- keep `use_human_radar: false` unless you explicitly want the older radar path
  active too
- set `draw_image: true` when you want `/debug/object_detection` in RViz or
  `rqt_image_view`
- `cpu` is the safest laptop default for the detector runtime
- the shipped detector model is currently biased toward labels such as
  blueberry, corn, pear, tomato, and zucchini, so prop mismatch is the first
  thing to revisit if detections look weak in demo prep

## Launch Profiles

Primary operator-facing launch files live in `src/nao_chatbot/launch/`:

- `nao_chatbot_sim.launch.py`: simulator stack with the official `interaction_sim` perspective
- `nao_chatbot_sim_asr.launch.py`: simulator stack plus local ASR
- `nao_chatbot_robot.launch.py`: real-robot camera, RViz, and HRI overlays
- `nao_chatbot_robot_asr.launch.py`: real-robot camera, RViz, HRI overlays, and local ASR
- `nao_chatbot_planner_local.launch.py`: local WME/planner harness with `nao_world_model_enricher`, `planner_llm`, and `nao_orchestrator`
- `nao_chatbot_asr_only.launch.py`: isolated local ASR pipeline

Useful launch combinations:

Simulator stack:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
```

Simulator stack with emorobcare object detection + scene grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Simulator stack with planner handoff enabled:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true
```

Simulator stack with planner handoff plus object grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Planner-only local harness:

```bash
ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py
ros2 run planner_llm publish_fixture scene
ros2 run planner_llm publish_fixture request
ros2 run planner_llm publish_fixture feedback
```

Real robot + RViz:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip>
```

Real robot + RViz + ASR:

```bash
ros2 launch nao_chatbot nao_chatbot_robot_asr.launch.py \
  nao_ip:=<robot_ip>
```

Real robot + RViz + emorobcare object detection:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=<robot_ip>
```

Real robot + RViz + simulator-side operator tools only:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true \
  nao_ip:=<robot_ip>
```

Real robot with a passive posture bridge on connect:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip> \
  posture_bridge_disable_autonomous_life_on_connect:=false \
  posture_bridge_wake_up_on_connect:=false
```

Those bridge defaults are already safe in the shipped launch wrappers. Override
them only when you intentionally want connect-time state changes on the robot.

Current robot-camera and overlay topics in the packaged stack:

- raw camera: `/camera/front/image_raw`
- HRI overlay: `/image/hri_overlay` with `compressed` transport
- emorobcare debug image: `/debug/object_detection`

Quick reference:

- [docs/demo_status_and_contracts.md](docs/demo_status_and_contracts.md)
- [docs/launch_profiles.md](docs/launch_profiles.md)
- [docs/current_workflow.md](docs/current_workflow.md)
- [docs/node_interactions_map.md](docs/node_interactions_map.md)
- [docs/ollama_chatbot_architecture.md](docs/ollama_chatbot_architecture.md)
- [docs/knowledge_core_integration_scope.md](docs/knowledge_core_integration_scope.md)
- [docs/asr_vosk_setup.md](docs/asr_vosk_setup.md)
- [docs/nao_camera_vlm_research.md](docs/nao_camera_vlm_research.md)

## Build And Test

Build the local packages shipped in this repo:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  std_skills communication_skills motions_skills kb_skills nao_skills \
  planner_common planner_llm nao_world_model_enricher chatbot_llm dialogue_manager nao_orchestrator nao_say_skill \
  nao_replay_motion nao_look_at nao_scene_grounding nao_chatbot \
  asr_vosk simple_audio_capture
```

Run the local validation suite:

```bash
./scripts/run_tests.sh
./.venv/bin/pre-commit run --all-files
```

## Docker Rebuild And Demo Prep

Preferred image for tomorrow's demo work:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Why this is the preferred path:

- it overlays the repo on top of the validated `iiia:nao` runtime image
- it now rebuilds `planner_common`, `planner_llm`, `nao_world_model_enricher`, `nao_scene_grounding`, and the kept launch surfaces
- it picks up `src/interaction_skills` directly instead of relying on the old
  `ref_src/interaction_skills` copy path
- it will also build `emorobcare_cv_msgs` and
  `emorobcare_cv_object_detection` when those packages are present under `src/`

Laptop-camera object-detection smoke path:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

Planner logs are standard ROS 2 node logs. They are visible in terminal output,
log files under `~/.ros/log`, and `rqt_console` when you
launch it or start the stack with `start_rqt_console:=true`.

Inside the container:

```bash
ros2 launch nao_chatbot nao_chatbot_planner_local.launch.py \
  start_rqt_console:=true

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=true \
  chatbot_planner_mode_enabled:=true

ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Real-robot object-detection follow-up:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip> \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Notes for the robot phase:

- `start_nao_look_at` is already `true` by default in the robot wrappers
- `nao_look_at` is the NAO implementation of the upstream
  `interaction_skills/look_at` contract, not a separate local API
- the preferred next integration step is to validate detector-grounded object
  awareness first, then tighten the live target-frame wiring into `nao_look_at`

Important detector caveat:

- the emorobcare path still needs both `emorobcare_cv_object_detection` and
  `emorobcare_cv_msgs` available in the workspace
- `ultralytics` is still required at runtime
- `my_game_interface` is only needed when you intentionally enable the older
  detector-side human-radar path; the default raw-detections flow used by
  `nao_scene_grounding` does not require it
- the overlay image based on `iiia:nao` remains the safest path for tomorrow;
  `docker/Dockerfile.full` should be treated as a fuller rebuild path, not the
  first demo choice

## KnowledgeCore References

Bootstrap the upstream reference clones if you need to inspect the official
sources locally:

```bash
./scripts/bootstrap_socialminds_sources.sh
```

Those clones are reference-only. The current supported runtime path for
KnowledgeCore and `interaction_sim` is the official SocialMinds Jazzy package
feed, not ad hoc local overlays under `src/`.

## Notes

- `src/dialogue_manager/` and `src/chatbot_llm/` are nested git repos; review
  and PR them in their own histories, not through the monorepo diff.
- `knowledge_core` does not natively inject prompt text into the LLM. The
  read-only KB-to-prompt seam lives in the local `chatbot_llm` fork.
- old `mission_controller`, `ollama_chatbot`, and `nao_skill_servers`
  runtime surfaces have been removed from the active workspace.
- `naoqi_driver` already exposes camera topics and joint interfaces, so future
  vision work should build on those ROS interfaces instead of adding a parallel
  capture stack.
- the emorobcare object detection package is intentionally not hard-vendored into this
  repo; place it in the Linux workspace `src/` tree and keep its own
  `config/config.yaml` aligned with the launch path you want to demo.
- the upstream `interaction_skills` package is also intentionally kept outside
  the monorepo history even when it is present under `src/`; local packages
  should implement its contracts rather than redefining them.
- `emorobcare_cv_object_detection` and `emorobcare_cv_msgs` are intentionally
  ignored by the monorepo so they can stay in their own histories while still
  being discovered by `colcon` from the shared workspace.
