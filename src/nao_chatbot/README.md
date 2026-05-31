# nao_chatbot

`nao_chatbot` is the launch and operator-utility package for the migrated NAO
ROS4HRI stack. It composes the dialogue, planner, executor, perception, ASR, and
robot adapter packages.

## Owns

- shared launch builder in `nao_chatbot/stack_launch.py`
- simulator, robot, demo, and ASR utility launch surfaces (planner is on by default in sim/robot/demo)
- push-to-talk and speech-debug operator utilities

It does not own chatbot inference, planner logic, or robot skill execution.

## Launch Profiles

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py
ros2 launch nao_chatbot nao_chatbot_robot.launch.py nao_ip:=<robot_ip>
ros2 launch nao_chatbot nao_chatbot_demo.launch.py
ros2 launch nao_chatbot nao_chatbot_asr_only.launch.py
```

ASR opt-in:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_asr:=true \
  asr_vosk_model_path:=/models/vosk-model-small-en-us-0.15
```

Demo profile:

```bash
ros2 launch nao_chatbot nao_chatbot_demo.launch.py
```

Planner is enabled by default in sim, robot, and demo. To turn it off:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_planner_llm:=false \
  chatbot_planner_mode_enabled:=false
```

Object grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Planner plus object grounding (defaults already include planner; only perception flags are required):

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Robot camera plus object grounding:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip> \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Robot with simulator tools only:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=<robot_ip> \
  start_interaction_sim:=true \
  start_interaction_sim_perception:=false \
  start_interaction_sim_tools:=true
```

## Important Launch Arguments

- `start_chatbot_llm`
- `start_dialogue_manager`
- `start_planner_llm`
- `chatbot_planner_mode_enabled`
- `start_nao_orchestrator`
- `start_nao_say_skill`
- `start_nao_replay_motion`
- `start_nao_look_at`
- `start_object_detection`
- `start_scene_grounding`
- `object_detection_backend`
- `nao_ip`
- `start_rviz`
- `start_asr`
- `asr_push_to_talk_enabled`
- `start_interaction_sim`
- `start_interaction_sim_perception`
- `start_interaction_sim_tools`
- `interaction_sim_hri_log_profile`
- `scene_grounding_knowledge_lifespan_sec`
- `scene_grounding_fallback_match_distance_px`

See `../../docs/launch_profiles.md` for the full operator guide.

## Operator Utilities

Push-to-talk:

```bash
ros2 run nao_chatbot asr_push_to_talk_cli
```

Speech debug:

```bash
ros2 run nao_chatbot robot_speech_debug
```

## Planner Contract Role

This package wires:

- `chatbot_llm` planner request publication.
- `planner_llm` planner node startup.
- `nao_orchestrator` planner feedback topic.
- local planner harnesses for diagnostics.

It should keep launch wiring explicit rather than hiding planner behavior inside
unrelated packages.

Planner dialogue ownership stays split:

- `chatbot_llm` generates user-facing wording for planner dialogue acts.
- `planner_llm` produces abstract plans and completion dialogue-act facts.
- `nao_orchestrator` executes skills and reports feedback.
- `dialogue_manager` routes planner dialogue-act facts back through
  `chatbot_llm` before speaking through TTS.

## Perception And Scene Grounding

`nao_chatbot` composes the object-aware demo path:

- detector backend: `emorobcare_cv_object_detection` by default, `yolo_ros` as
  fallback.
- `nao_scene_grounding`: detector normalization, transient KB facts, and
  `/scene/summary`.
- `chatbot_llm`: reads grounded facts through `kb_skills` and `/kb/query`.

Operational notes:

- Keep detector-side direct KB writes disabled when using
  `nao_scene_grounding`.
- Use `/debug/object_detection` to inspect detector overlays.
- Current emorobcare demo props are strongest around tomato, pear, zucchini,
  corn, and blueberry.
- The simulator profile now runs `hri_person_manager` against `base_link`
  rather than `map`, because the sim TF tree only publishes
  `base_link -> sellion_link -> camera`.

## Docker Demo Path

Typical rebuild:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Laptop-camera run:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

## Tests

```bash
PYTHONPATH=src/nao_chatbot python3 -m pytest -q src/nao_chatbot/test/unit
```
