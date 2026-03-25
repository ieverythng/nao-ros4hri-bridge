# nao_scene_grounding

`nao_scene_grounding` is the local bridge between detector outputs and the
symbolic scene state used by the rest of the NAO ROS4HRI stack.

For a demo-focused summary of the current runtime, grounded-scene flow, and
ROS contracts, see [`docs/demo_status_and_contracts.md`](../../docs/demo_status_and_contracts.md).

It keeps object detection modular on purpose:

- detector backends can change without rewriting KB logic
- grounded scene facts keep flowing through the same `/kb/revise` path
- downstream consumers can read the compact `/scene/summary` topic instead of
  depending on detector-specific message types

## Where It Fits

`nao_scene_grounding` does not feed `chatbot_llm` directly. The current flow is:

1. the external detector publishes raw detections such as `/detected_objects`
2. `nao_scene_grounding` subscribes to that detector topic and normalizes the
   backend-specific message into shared observations
3. the node refreshes transient object facts in `knowledge_core` through
   `/kb/revise` and publishes `/scene/summary` for debug and future consumers
4. on the next turn, `chatbot_llm` still reads the grounded scene through its
   normal `/kb/query` plus `knowledge_snapshot` path

This means detector integration stays modular: detector packages own inference,
`nao_scene_grounding` owns detector-to-KB grounding, and `chatbot_llm` remains
the read-side prompt consumer.

## What The Node Does

`nao_scene_grounding` subscribes to one detector topic, normalizes the incoming
detections into a shared internal format, and then:

- filters by allowed labels and score threshold
- assigns stable entity ids when a tracker id or bounding-box center is
  available
- refreshes transient object facts in `knowledge_core`
- publishes a JSON scene summary for debug and future VLM use

The node does not run the detector itself. The detector backend is launched
outside the package and selected from the main `nao_chatbot` launch surface.

## Supported Detector Backends

### `emorobcare_cv`

Expected input:

- topic: `/detected_objects`
- type: `emorobcare_cv_msgs/msg/ObjectDetections`

Typical debug image:

- `/debug/object_detection`

This is the current demo-first default because it was authored with ROS4HRI in
mind and already fits the NAO pipeline well. It also requires the
`emorobcare_cv_msgs` package to be available in the same workspace.

### `yolo_ros`

Expected input:

- topic: `/yolo/tracking`
- type: `yolo_msgs/msg/DetectionArray`

Typical debug image:

- `/yolo/debug_image`

This remains as the fallback backend and is useful when you want a more general
ROS 2 detector path or a quick alternative if the in-house detector needs more
tuning.

## Runtime Outputs

- `/scene/summary`
  - type: `std_msgs/msg/String`
  - JSON payload containing the currently tracked grounded objects
- `/kb/revise`
  - service calls to refresh transient object facts such as:
    - `myself sees detected_tomato_123`
    - `detected_tomato_123 rdf:type Tomato`

The node uses KnowledgeCore fact lifespans rather than maintaining a second KB
TTL mechanism locally. Repeated observations refresh the lifespan before expiry.

## Main Parameters

Defaults live in `config/00-defaults.yml`.

Most useful knobs:

- `detector_backend`
- `detector_topic`
- `summary_topic`
- `min_detection_score`
- `allowed_labels`
- `label_class_overrides`
- `knowledge_enabled`
- `knowledge_revise_service_name`
- `knowledge_lifespan_sec`
- `knowledge_refresh_interval_sec`

## Launch From The Main Stack

Simulator + emorobcare object detection:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

The sim wrappers keep `expressive_face` disabled by default now so the webcam +
HRI + detector path stays lean and does not spawn an extra simulator-side TTS
action server. If you explicitly want the simulator face UI back, add:

```bash
start_interaction_sim_expressive_face:=true
```

Real robot + RViz + emorobcare object detection:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  nao_ip:=172.26.112.62
```

## Docker Demo Path

Preferred rebuild for the detector demo:

```bash
docker build -f docker/Dockerfile \
  --build-arg BASE_IMAGE=iiia:nao \
  -t nao-ros4hri-bridge:demo .
```

Primary laptop-camera validation:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nao-ros4hri-bridge:demo
```

Then launch:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

For the follow-up robot phase:

```bash
ros2 launch nao_chatbot nao_chatbot_robot.launch.py \
  nao_ip:=172.26.112.62 \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

## Launch `nao_scene_grounding` On Its Own

This is the fastest way to debug detector-to-KB grounding without the rest of
`chatbot_llm`, `dialogue_manager`, or the robot wrappers.

1. Start the detector by itself:

```bash
ros2 run emorobcare_cv_object_detection object_detector_node
```

2. In a second terminal, start the grounding bridge:

```bash
ros2 run nao_scene_grounding start_node --ros-args \
  -p detector_backend:=emorobcare_cv \
  -p detector_topic:=/detected_objects \
  -p summary_topic:=/scene/summary
```

3. In a third terminal, watch the runtime outputs:

```bash
ros2 node list | egrep 'object_detector_node|nao_scene_grounding'
ros2 topic list | egrep '/detected_objects|/debug/object_detection|/scene/summary'
ros2 topic echo /scene/summary
```

If you want to inspect grounding without touching KnowledgeCore, add:

```bash
-p knowledge_enabled:=false
```

to the `nao_scene_grounding` command.

## Debug Checklist

If the object-detect nodes do not appear in the graph, check these in order:

1. Package discovery:

```bash
ros2 pkg list | egrep 'emorobcare_cv_object_detection|emorobcare_cv_msgs|nao_scene_grounding'
```

2. Detector Python runtime:

```bash
python3 -c "import ultralytics"
```

3. Detector node exposure:

```bash
ros2 node list | egrep 'object_detector_node|nao_scene_grounding'
```

4. Detector topics:

```bash
ros2 topic list | egrep '/detected_objects|/debug/object_detection|/scene/summary'
```

5. Message flow:

```bash
ros2 topic hz /detected_objects
ros2 topic echo /scene/summary
```

If the detector source is present under `src/` but `ros2 pkg list` does not
show `emorobcare_cv_object_detection` or `emorobcare_cv_msgs`, the workspace
has not been rebuilt yet. Mounting the source tree into the container is not
enough by itself; rebuild the workspace or rebuild the overlay image so those
packages exist under `install/`.

If `object_detector_node` starts and exits immediately with `ModuleNotFoundError:
No module named 'ultralytics'`, you are running in an environment that has not
installed the detector runtime yet. Use the rebuilt overlay image from this repo
or install the CPU detector dependencies before launching.

## Demo Notes

- Keep `use_knowledge_base: false` in the emorobcare object detection package if you
  want this node to remain the single writer of detector-derived KB facts.
- Turn `draw_image: true` on in the emorobcare object detection package when you want
  the `/debug/object_detection` image in RViz or `rqt_image_view`.
- Keep `use_human_radar: false` in the emorobcare object detection package unless you
  intentionally want its older radar integration active.
- On the laptop, `cpu` is the safest default. GPU acceleration can be explored
  later on the Linux machine if available.
- The current model is still biased toward labels such as blueberry, corn,
  pear, tomato, and zucchini, so unusual demo props may require model or label
  configuration updates before the run.
- If you only need one or two demo objects, tomato and pear are the safest
  first props to try, with zucchini, corn, and blueberry as the next most
  likely labels based on the current detector-side configuration.
- The current detector package still expects `emorobcare_cv_msgs` plus some
  detector-side runtime imports to be available. In the default
  `use_knowledge_base: false` and `use_human_radar: false` setup, the important
  runtime requirements are the detector itself, `emorobcare_cv_msgs`, and
  `ultralytics`; `my_game_interface` is only needed if you re-enable the older
  detector-side radar integration. For tomorrow's demo, the overlay image on
  top of `iiia:nao` is still the safer path than a from-scratch container
  rebuild.
