# nao_scene_grounding

`nao_scene_grounding` bridges detector outputs into symbolic object facts and a
compact `/scene/summary`.

## Owns

- detector message normalization
- object allowlist/score filtering
- local object identity stabilization
- transient KnowledgeCore writes through `kb_skills`
- `/scene/summary`

It does not own person-manager identity, chatbot prompting, planner policy, or
robot execution.

## Public ROS Interfaces

| Direction | Interface | Type | Purpose |
| --- | --- | --- | --- |
| subscribe | detector topic | backend-specific | Raw detector outputs |
| publish | `/scene/summary` | `std_msgs/msg/String` | Current grounded object summary |
| service client | `/kb/revise` | `kb_msgs/srv/Revise` | Transient object facts |

Supported detector backends:

- `emorobcare_cv`: `/detected_objects`, `emorobcare_cv_msgs/msg/ObjectDetections`.
- `yolo_ros`: `/yolo/tracking`, `yolo_msgs/msg/DetectionArray`.

## Scene Summary Contract

```json
{
  "observer": "myself",
  "backend": "emorobcare_cv",
  "objects": [
    {
      "entity_id": "detected_cup_320_240",
      "label": "cup",
      "kb_class": "Cup",
      "score": 0.91,
      "tracker_id": "",
      "source": "emorobcare_cv",
      "center_x": 320.0,
      "center_y": 240.0,
      "last_seen_sec": 1777040000.0
    }
  ]
}
```

This differs from `knowledge_snapshot`: `/scene/summary` is a current object
feed, while `knowledge_snapshot` is chatbot prompt text derived from KB queries.

## Important Parameters

Defaults live in `config/00-defaults.yml`.

- `detector_backend`
- `detector_topic`
- `summary_topic`
- `min_detection_score`
- `allowed_labels`
- `label_class_overrides`
- `entity_prefix`
- `observer_name`
- `knowledge_enabled`
- `knowledge_revise_service_name`
- `knowledge_lifespan_sec`
- `knowledge_refresh_interval_sec`
- `local_stale_after_sec`

## Launch

As part of the stack:

```bash
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  start_object_detection:=true \
  start_scene_grounding:=true \
  object_detection_backend:=emorobcare_cv
```

Standalone:

```bash
ros2 run nao_scene_grounding start_node --ros-args \
  -p detector_backend:=emorobcare_cv \
  -p detector_topic:=/detected_objects \
  -p summary_topic:=/scene/summary
```

Disable KB writes while debugging:

```bash
-p knowledge_enabled:=false
```

## Tests

```bash
PYTHONPATH=src/nao_scene_grounding:src/kb_skills \
python3 -m pytest -q src/nao_scene_grounding/test
```

## Design Note

Do not rename or broaden this package into an object manager before the planner
loop is stable. If an `object_manager` concept is needed, document whether it
belongs as a future wrapper or as part of a WME layer.
