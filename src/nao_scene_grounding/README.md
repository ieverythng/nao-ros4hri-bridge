# nao_scene_grounding

`nao_scene_grounding` is the local bridge between detector outputs and the
symbolic scene state used by the rest of the NAO ROS4HRI stack.

It keeps object detection modular on purpose:

- detector backends can change without rewriting KB logic
- grounded scene facts keep flowing through the same `/kb/revise` path
- downstream consumers can read the compact `/scene/summary` topic instead of
  depending on detector-specific message types

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

Colleague detector path:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv \
  scene_grounding_detector_topic:=/detected_objects
```

Fallback `yolo_ros` path:

```bash
ros2 launch nao_chatbot nao_chatbot_ros4hri_migration.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=yolo_ros \
  scene_grounding_detector_topic:=/yolo/tracking
```

## Demo Notes

- Keep `use_knowledge_base: false` in the colleague detector package if you
  want this node to remain the single writer of detector-derived KB facts.
- Turn `draw_image: true` on in the colleague detector package when you want
  the `/debug/object_detection` image in RViz or `rqt_image_view`.
- Keep `use_human_radar: false` in the colleague detector unless you
  intentionally want its older radar integration active.
- On the laptop, `cpu` is the safest default. GPU acceleration can be explored
  later on the Linux machine if available.
- The current model is still biased toward labels such as blueberry, corn,
  pear, tomato, and zucchini, so unusual demo props may require model or label
  configuration updates before the run.
