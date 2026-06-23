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
from kb_skills.mutation_client import KnowledgeCoreMutationClient

from nao_scene_grounding.detector_adapters import DEFAULT_ALLOWED_LABELS
from nao_scene_grounding.detector_adapters import EmorobcareDetectionAdapter
from nao_scene_grounding.detector_adapters import ObjectObservation
from nao_scene_grounding.detector_adapters import YoloRosDetectionAdapter
from nao_scene_grounding.detector_adapters import coerce_str_list
from nao_scene_grounding.detector_adapters import load_label_class_map
from nao_scene_grounding.identity_matching import reconcile_observation_entity_ids

try:  # pragma: no cover - runtime dependency
    from yolo_msgs.msg import DetectionArray as YoloDetectionArray
except ImportError:  # pragma: no cover - runtime dependency
    YoloDetectionArray = None

try:  # pragma: no cover - runtime dependency
    from emorobcare_cv_msgs.msg import ObjectDetections as EmorobcareObjectDetections
except ImportError:  # pragma: no cover - runtime dependency
    EmorobcareObjectDetections = None


@dataclass(frozen=True, slots=True)
class _BackendSpec:
    adapter_class: type
    message_type: object
    missing_dependency_message: str


_BACKEND_SPECS = {
    'yolo_ros': _BackendSpec(
        adapter_class=YoloRosDetectionAdapter,
        message_type=YoloDetectionArray,
        missing_dependency_message=(
            'yolo_msgs is unavailable; yolo_ros detections cannot be subscribed yet'
        ),
    ),
    'emorobcare_cv': _BackendSpec(
        adapter_class=EmorobcareDetectionAdapter,
        message_type=EmorobcareObjectDetections,
        missing_dependency_message=(
            'emorobcare_cv_msgs is unavailable; emorobcare detections cannot be subscribed yet'
        ),
    ),
}


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
    frame_id: str = ''
    position_x: float | None = None
    position_y: float | None = None
    position_z: float | None = None

    @classmethod
    def from_observation(
        cls,
        observation: ObjectObservation,
        now_sec: float,
    ) -> '_TrackedObject':
        return cls(
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

    def update_from_observation(
        self,
        observation: ObjectObservation,
        now_sec: float,
    ) -> None:
        self.label = observation.label
        self.kb_class = observation.kb_class
        self.score = observation.score
        self.tracker_id = observation.tracker_id
        self.source = observation.source
        self.center_x = observation.center_x
        self.center_y = observation.center_y
        self.last_seen_sec = now_sec

    def summary_dict(self) -> dict:
        payload = {
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
        if self.has_metric_position:
            payload['frame_id'] = self.frame_id
            payload['position'] = {
                'x': round(float(self.position_x), 3),
                'y': round(float(self.position_y), 3),
                'z': round(float(self.position_z), 3),
            }
            payload['distance_m'] = round(self.distance_m, 3)
        return payload

    @property
    def has_metric_position(self) -> bool:
        return bool(self.frame_id) and all(
            value is not None
            for value in (self.position_x, self.position_y, self.position_z)
        )

    @property
    def distance_m(self) -> float:
        if not self.has_metric_position:
            return 0.0
        return math.sqrt(
            float(self.position_x) ** 2
            + float(self.position_y) ** 2
            + float(self.position_z) ** 2
        )

    def apply_spatial_overlay(self, payload: dict) -> bool:
        frame_id = str(payload.get('frame_id', '')).strip()
        position = payload.get('position', {})
        if not frame_id or not isinstance(position, dict):
            return False
        coordinates = tuple(_optional_float(position.get(axis)) for axis in ('x', 'y', 'z'))
        if any(value is None for value in coordinates):
            return False
        next_position = (frame_id, *coordinates)
        current_position = (
            self.frame_id,
            self.position_x,
            self.position_y,
            self.position_z,
        )
        if next_position == current_position:
            return False
        self.frame_id = frame_id
        self.position_x, self.position_y, self.position_z = coordinates
        return True


def _kb_spatial_statements(observer_name: str, tracked: _TrackedObject) -> list[str]:
    """Build stable KB statements for one tracked object observation."""
    observer = str(observer_name or '').strip() or 'myself'
    source_token = _kb_atom(str(tracked.source or '').strip() or 'detector')
    statements = [
        f'{observer} sees {tracked.entity_id}',
        f'{tracked.entity_id} rdf:type {tracked.kb_class}',
        f'{tracked.entity_id} hasVisualCenterX {round(float(tracked.center_x), 3)}',
        f'{tracked.entity_id} hasVisualCenterY {round(float(tracked.center_y), 3)}',
        f'{tracked.entity_id} hasDetectionScore {round(float(tracked.score), 6)}',
        f'{tracked.entity_id} lastSeenSec {round(float(tracked.last_seen_sec), 3)}',
        f'{tracked.entity_id} detectionSource {source_token}',
    ]
    if tracked.has_metric_position:
        statements.extend(
            [
                f'{tracked.entity_id} spatialFrame {_kb_atom(tracked.frame_id)}',
                f'{tracked.entity_id} positionX {round(float(tracked.position_x), 6)}',
                f'{tracked.entity_id} positionY {round(float(tracked.position_y), 6)}',
                f'{tracked.entity_id} positionZ {round(float(tracked.position_z), 6)}',
                f'{tracked.entity_id} distanceFromObserverM {round(tracked.distance_m, 6)}',
            ]
        )
    return statements


def _kb_atom(value: str) -> str:
    """Normalize free-form strings into conservative KB atom tokens."""
    clean = ''.join(
        char if (char.isalnum() or char in ('_', '-')) else '_'
        for char in str(value or '').strip()
    ).strip('_')
    return clean or 'unknown'


def _optional_float(value) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class NaoSceneGrounding(Node):
    """Ground object detections into KnowledgeCore using transient facts."""

    def __init__(self) -> None:
        super().__init__('nao_scene_grounding')

        # Configure the detector-facing and KnowledgeCore-facing seams first so
        # the node can run with either detector backend without code changes.
        self.declare_parameter('detector_backend', 'emorobcare_cv')
        self.declare_parameter('detector_topic', '/detected_objects')
        self.declare_parameter('summary_topic', '~/summary')
        self.declare_parameter('spatial_overlay_topic', '')
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
        self.declare_parameter('fallback_match_distance_px', 64.0)
        self.declare_parameter('fallback_match_max_age_sec', 2.0)

        self._detector_backend = str(self.get_parameter('detector_backend').value).strip()
        self._detector_topic = str(self.get_parameter('detector_topic').value).strip()
        self._summary_topic = str(self.get_parameter('summary_topic').value).strip() or '~/summary'
        self._spatial_overlay_topic = str(
            self.get_parameter('spatial_overlay_topic').value
        ).strip()
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
        self._fallback_match_distance_px = max(
            0.0,
            float(self.get_parameter('fallback_match_distance_px').value),
        )
        self._fallback_match_max_age_sec = max(
            0.0,
            float(self.get_parameter('fallback_match_max_age_sec').value),
        )
        self._backend_spec = _BACKEND_SPECS.get(self._detector_backend)

        # Create publishers and service clients before subscriptions so early
        # detections can immediately publish summaries and revise KB facts.
        self._summary_pub = self.create_publisher(String, self._summary_topic, 10)
        self._mutation_client = None
        if self._knowledge_enabled:
            self._mutation_client = KnowledgeCoreMutationClient(
                node=self,
                callback_group=None,
                service_name=self._knowledge_revise_service_name,
                timeout_sec=0.5,
            )

        self._tracked_objects: dict[str, _TrackedObject] = {}
        self._last_summary_payload = ''
        self._last_missing_dependency_notice = ''

        self._adapter = self._make_adapter()
        self._create_detector_subscription()
        if self._spatial_overlay_topic:
            self.create_subscription(
                String,
                self._spatial_overlay_topic,
                self._on_spatial_overlay,
                10,
            )
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
        if self._backend_spec is None:
            self.get_logger().warn(
                'Unsupported detector_backend=%s. Supported backends are yolo_ros and emorobcare_cv.'
                % self._detector_backend
            )
            return

        if self._backend_spec.message_type is None:
            self._log_missing_dependency_once(
                self._backend_spec.missing_dependency_message
            )
            return

        self.create_subscription(
            self._backend_spec.message_type,
            self._detector_topic,
            self._on_detections,
            10,
        )

    def _make_adapter(self):
        """Build the normalization adapter matching the active backend."""
        adapter_kwargs = {
            'allowed_labels': self._allowed_labels,
            'label_class_map': self._label_class_map,
            'entity_prefix': self._entity_prefix,
            'source': self._detector_backend,
        }
        if self._backend_spec is None:
            return _UnsupportedDetectionAdapter()
        return self._backend_spec.adapter_class(**adapter_kwargs)

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
        observations = reconcile_observation_entity_ids(
            observations,
            self._tracked_objects.values(),
            now_sec=now_sec,
            max_match_distance_px=self._fallback_match_distance_px,
            max_match_age_sec=self._fallback_match_max_age_sec,
        )
        revised_any = False
        for observation in observations:
            tracked = self._upsert_tracked_object(observation, now_sec)

            if now_sec - tracked.last_revised_sec >= self._knowledge_refresh_interval_sec:
                revised_any = self._revise_observation(tracked, now_sec) or revised_any

        if observations or revised_any:
            self._publish_summary()

    def _on_spatial_overlay(self, msg: String) -> None:
        """Merge frame-qualified simulator/object poses into tracked entities."""
        try:
            payload = json.loads(str(msg.data or '').strip() or '{}')
        except json.JSONDecodeError:
            self._log_missing_dependency_once('Ignoring invalid spatial overlay JSON')
            return
        entries = payload.get('objects', []) if isinstance(payload, dict) else []
        if not isinstance(entries, list):
            return
        changed = False
        now_sec = self._clock_now_sec()
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            entity_id = str(entry.get('entity_id', entry.get('id', ''))).strip()
            tracked = self._tracked_objects.get(entity_id)
            if tracked is not None:
                entry_changed = tracked.apply_spatial_overlay(entry)
                if entry_changed:
                    self._revise_observation(tracked, now_sec)
                changed = entry_changed or changed
        if changed:
            self._publish_summary()

    def _upsert_tracked_object(
        self,
        observation: ObjectObservation,
        now_sec: float,
    ) -> _TrackedObject:
        tracked = self._tracked_objects.get(observation.entity_id)
        if tracked is None:
            tracked = _TrackedObject.from_observation(observation, now_sec)
            self._tracked_objects[observation.entity_id] = tracked
            return tracked
        tracked.update_from_observation(observation, now_sec)
        return tracked

    def _revise_observation(self, tracked: _TrackedObject, now_sec: float) -> bool:
        """Refresh transient KnowledgeCore facts for one tracked object."""
        if not self._knowledge_enabled or self._mutation_client is None:
            tracked.last_revised_sec = now_sec
            return False

        result = self._mutation_client.revise_facts(
            _kb_spatial_statements(self._observer_name, tracked),
            models=list(self._knowledge_models),
            lifespan_sec=self._knowledge_lifespan_sec,
            wait_for_result=False,
        )
        tracked.last_revised_sec = now_sec
        if result.success:
            return True
        self._log_missing_dependency_once(
            'KnowledgeCore revise service is not ready yet; object facts will start once it appears'
        )
        return False

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


class _UnsupportedDetectionAdapter:
    """Fallback adapter used when a configured backend is unknown."""

    def parse_detections(self, _msg, min_score: float = 0.0) -> list[ObjectObservation]:
        del min_score
        return []


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
