"""Continuous object grounding node for detector outputs.

The node sits between detector-specific topics and the rest of the NAO stack:
it normalizes backend messages, refreshes transient KnowledgeCore facts, and
publishes a compact `/scene/summary` for operators and future consumers.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from std_msgs.msg import String

from nao_scene_grounding.detector_adapters import DEFAULT_ALLOWED_LABELS
from nao_scene_grounding.detector_adapters import EmorobcareDetectionAdapter
from nao_scene_grounding.detector_adapters import YoloRosDetectionAdapter
from nao_scene_grounding.detector_adapters import coerce_str_list
from nao_scene_grounding.detector_adapters import load_label_class_map

try:  # pragma: no cover - runtime dependency
    from kb_msgs.srv import Revise
except ImportError:  # pragma: no cover - runtime dependency
    Revise = None

try:  # pragma: no cover - runtime dependency
    from yolo_msgs.msg import DetectionArray as YoloDetectionArray
except ImportError:  # pragma: no cover - runtime dependency
    YoloDetectionArray = None

try:  # pragma: no cover - runtime dependency
    from emorobcare_cv_msgs.msg import ObjectDetections as EmorobcareObjectDetections
except ImportError:  # pragma: no cover - runtime dependency
    EmorobcareObjectDetections = None


@dataclass(slots=True)
class _TrackedObject:
    entity_id: str
    label: str
    kb_class: str
    score: float
    tracker_id: str
    source: str
    center_x: float
    center_y: float
    last_seen_sec: float
    last_revised_sec: float = 0.0

    def summary_dict(self) -> dict:
        return {
            'entity_id': self.entity_id,
            'label': self.label,
            'kb_class': self.kb_class,
            'score': round(float(self.score), 3),
            'tracker_id': self.tracker_id,
            'source': self.source,
            'center_x': round(float(self.center_x), 1),
            'center_y': round(float(self.center_y), 1),
            'last_seen_sec': round(float(self.last_seen_sec), 3),
        }


class NaoSceneGrounding(Node):
    """Ground object detections into KnowledgeCore using transient facts."""

    def __init__(self) -> None:
        super().__init__('nao_scene_grounding')

        # Configure the detector-facing and KnowledgeCore-facing seams first so
        # the node can run with either detector backend without code changes.
        self.declare_parameter('detector_backend', 'emorobcare_cv')
        self.declare_parameter('detector_topic', '/detected_objects')
        self.declare_parameter('summary_topic', '~/summary')
        self.declare_parameter('min_detection_score', 0.35)
        self.declare_parameter('allowed_labels', ','.join(DEFAULT_ALLOWED_LABELS))
        self.declare_parameter('label_class_overrides', '{}')
        self.declare_parameter('entity_prefix', 'detected')
        self.declare_parameter('observer_name', 'myself')
        self.declare_parameter('knowledge_enabled', True)
        self.declare_parameter('knowledge_revise_service_name', '/kb/revise')
        self.declare_parameter('knowledge_models', ['default'])
        self.declare_parameter('knowledge_lifespan_sec', 4.0)
        self.declare_parameter('knowledge_refresh_interval_sec', 1.0)
        self.declare_parameter('local_stale_after_sec', 4.5)

        self._detector_backend = str(self.get_parameter('detector_backend').value).strip()
        self._detector_topic = str(self.get_parameter('detector_topic').value).strip()
        self._summary_topic = str(self.get_parameter('summary_topic').value).strip() or '~/summary'
        self._min_detection_score = max(
            0.0,
            float(self.get_parameter('min_detection_score').value),
        )
        self._allowed_labels = coerce_str_list(
            self.get_parameter('allowed_labels').value,
            fallback=list(DEFAULT_ALLOWED_LABELS),
        )
        self._label_class_map = load_label_class_map(
            self.get_parameter('label_class_overrides').value
        )
        self._entity_prefix = str(self.get_parameter('entity_prefix').value).strip() or 'detected'
        self._observer_name = str(self.get_parameter('observer_name').value).strip() or 'myself'
        self._knowledge_enabled = bool(self.get_parameter('knowledge_enabled').value)
        self._knowledge_revise_service_name = str(
            self.get_parameter('knowledge_revise_service_name').value
        ).strip() or '/kb/revise'
        try:
            raw_knowledge_models = self.get_parameter('knowledge_models').value
        except ParameterUninitializedException:
            raw_knowledge_models = ['default']
        self._knowledge_models = coerce_str_list(
            raw_knowledge_models,
            fallback=['default'],
        )
        self._knowledge_lifespan_sec = max(
            0.5,
            float(self.get_parameter('knowledge_lifespan_sec').value),
        )
        self._knowledge_refresh_interval_sec = max(
            0.2,
            float(self.get_parameter('knowledge_refresh_interval_sec').value),
        )
        self._local_stale_after_sec = max(
            self._knowledge_lifespan_sec,
            float(self.get_parameter('local_stale_after_sec').value),
        )

        # Create publishers and service clients before subscriptions so early
        # detections can immediately publish summaries and revise KB facts.
        self._summary_pub = self.create_publisher(String, self._summary_topic, 10)
        self._revise_client = None
        if self._knowledge_enabled and Revise is not None:
            self._revise_client = self.create_client(
                Revise,
                self._knowledge_revise_service_name,
            )
        elif self._knowledge_enabled:
            self.get_logger().warn(
                'kb_msgs is unavailable; KnowledgeCore revise writes are disabled'
            )

        self._tracked_objects: dict[str, _TrackedObject] = {}
        self._last_summary_payload = ''
        self._last_missing_dependency_notice = ''

        self._adapter = self._make_adapter()
        self._create_detector_subscription()
        self.create_timer(0.5, self._housekeeping_tick)

        self.get_logger().info(
            'nao_scene_grounding ready | backend=%s topic=%s labels=%s revise=%s'
            % (
                self._detector_backend,
                self._detector_topic,
                ','.join(self._allowed_labels),
                self._knowledge_revise_service_name,
            )
        )

    # -------------------------------------------------------------------------
    # Detector subscription setup
    # -------------------------------------------------------------------------

    def _create_detector_subscription(self) -> None:
        """Subscribe to the selected detector backend if its message types exist."""
        if self._detector_backend == 'yolo_ros':
            if YoloDetectionArray is None:
                self._log_missing_dependency_once(
                    'yolo_msgs is unavailable; yolo_ros detections cannot be subscribed yet'
                )
                return
            self.create_subscription(
                YoloDetectionArray,
                self._detector_topic,
                self._on_detections,
                10,
            )
            return

        if self._detector_backend == 'emorobcare_cv':
            if EmorobcareObjectDetections is None:
                self._log_missing_dependency_once(
                    'emorobcare_cv_msgs is unavailable; emorobcare detections cannot be subscribed yet'
                )
                return
            self.create_subscription(
                EmorobcareObjectDetections,
                self._detector_topic,
                self._on_detections,
                10,
            )
            return

        self.get_logger().warn(
            'Unsupported detector_backend=%s. Supported backends are yolo_ros and emorobcare_cv.'
            % self._detector_backend
        )

    def _make_adapter(self):
        """Build the normalization adapter matching the active backend."""
        adapter_kwargs = {
            'allowed_labels': self._allowed_labels,
            'label_class_map': self._label_class_map,
            'entity_prefix': self._entity_prefix,
            'source': self._detector_backend,
        }
        if self._detector_backend == 'yolo_ros':
            return YoloRosDetectionAdapter(**adapter_kwargs)
        if self._detector_backend == 'emorobcare_cv':
            return EmorobcareDetectionAdapter(**adapter_kwargs)
        return YoloRosDetectionAdapter(**adapter_kwargs)

    # -------------------------------------------------------------------------
    # Detection ingestion and KB refresh
    # -------------------------------------------------------------------------

    def _on_detections(self, msg) -> None:
        """Merge one detector message into tracked objects and summary output."""
        now_sec = self._clock_now_sec()
        observations = self._adapter.parse_detections(
            msg,
            min_score=self._min_detection_score,
        )
        revised_any = False
        for observation in observations:
            tracked = self._tracked_objects.get(observation.entity_id)
            if tracked is None:
                tracked = _TrackedObject(
                    entity_id=observation.entity_id,
                    label=observation.label,
                    kb_class=observation.kb_class,
                    score=observation.score,
                    tracker_id=observation.tracker_id,
                    source=observation.source,
                    center_x=observation.center_x,
                    center_y=observation.center_y,
                    last_seen_sec=now_sec,
                )
                self._tracked_objects[observation.entity_id] = tracked
            else:
                tracked.label = observation.label
                tracked.kb_class = observation.kb_class
                tracked.score = observation.score
                tracked.tracker_id = observation.tracker_id
                tracked.source = observation.source
                tracked.center_x = observation.center_x
                tracked.center_y = observation.center_y
                tracked.last_seen_sec = now_sec

            if now_sec - tracked.last_revised_sec >= self._knowledge_refresh_interval_sec:
                revised_any = self._revise_observation(tracked, now_sec) or revised_any

        if observations or revised_any:
            self._publish_summary()

    def _revise_observation(self, tracked: _TrackedObject, now_sec: float) -> bool:
        """Refresh transient KnowledgeCore facts for one tracked object."""
        if not self._knowledge_enabled or self._revise_client is None or Revise is None:
            tracked.last_revised_sec = now_sec
            return False
        if not self._revise_client.service_is_ready():
            self._log_missing_dependency_once(
                'KnowledgeCore revise service is not ready yet; object facts will start once it appears'
            )
            return False

        request = Revise.Request()
        request.method = 'update'
        request.statements = [
            f'{self._observer_name} sees {tracked.entity_id}',
            f'{tracked.entity_id} rdf:type {tracked.kb_class}',
        ]
        request.models = list(self._knowledge_models)
        request.lifespan.sec = int(math.floor(self._knowledge_lifespan_sec))
        request.lifespan.nanosec = int(
            (self._knowledge_lifespan_sec - request.lifespan.sec) * 1_000_000_000
        )
        self._revise_client.call_async(request)
        tracked.last_revised_sec = now_sec
        return True

    # -------------------------------------------------------------------------
    # Local housekeeping and operator-facing summary output
    # -------------------------------------------------------------------------

    def _housekeeping_tick(self) -> None:
        """Drop locally stale tracked objects and republish the summary if needed."""
        if not self._tracked_objects:
            return
        now_sec = self._clock_now_sec()
        stale_ids = [
            entity_id
            for entity_id, tracked in self._tracked_objects.items()
            if now_sec - tracked.last_seen_sec > self._local_stale_after_sec
        ]
        if not stale_ids:
            return
        for entity_id in stale_ids:
            self._tracked_objects.pop(entity_id, None)
        self._publish_summary()

    def _publish_summary(self) -> None:
        """Publish a stable JSON scene snapshot only when it actually changes."""
        payload = json.dumps(
            {
                'observer': self._observer_name,
                'backend': self._detector_backend,
                'objects': [
                    tracked.summary_dict()
                    for tracked in sorted(
                        self._tracked_objects.values(),
                        key=lambda item: (item.label, item.entity_id),
                    )
                ],
            },
            sort_keys=True,
            separators=(',', ':'),
        )
        if payload == self._last_summary_payload:
            return
        msg = String()
        msg.data = payload
        self._summary_pub.publish(msg)
        self._last_summary_payload = payload

    def _log_missing_dependency_once(self, message: str) -> None:
        """Avoid flooding logs when optional detector or KB deps are unavailable."""
        if message == self._last_missing_dependency_notice:
            return
        self._last_missing_dependency_notice = message
        self.get_logger().warn(message)

    def _clock_now_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + float(now_msg.nanosec) / 1_000_000_000.0


def main(args=None) -> None:
    """Run the scene-grounding node as a normal rclpy process."""
    rclpy.init(args=args)
    node = NaoSceneGrounding()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # pragma: no cover - manual shutdown
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
