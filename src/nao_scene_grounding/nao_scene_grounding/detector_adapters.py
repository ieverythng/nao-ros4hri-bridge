"""Detector adapter helpers for scene grounding."""

from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
import json
import re


DEFAULT_ALLOWED_LABELS = (
    'bottle',
    'cup',
    'book',
    'cell phone',
    'backpack',
    'remote',
    'laptop',
    'keyboard',
    'mouse',
    'chair',
    'blueberry',
    'corn',
    'pear',
    'tomato',
    'zucchini',
)

DEFAULT_LABEL_CLASS_MAP = {
    'backpack': 'Backpack',
    'book': 'Book',
    'bottle': 'Bottle',
    'cell phone': 'CellPhone',
    'chair': 'Chair',
    'cup': 'Cup',
    'keyboard': 'Keyboard',
    'laptop': 'Laptop',
    'mouse': 'Mouse',
    'pear': 'Pear',
    'remote': 'Remote',
    'tomato': 'Tomato',
    'zucchini': 'Zucchini',
    'blueberry': 'Blueberry',
    'corn': 'Corn',
}


# -----------------------------------------------------------------------------
# Shared normalized detector representation
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class ObjectObservation:
    """Normalized detector observation used by the grounding node."""

    entity_id: str
    label: str
    kb_class: str
    score: float
    tracker_id: str
    source: str
    center_x: float
    center_y: float

    def as_summary_dict(self) -> dict:
        payload = asdict(self)
        payload['score'] = round(float(self.score), 3)
        payload['center_x'] = round(float(self.center_x), 1)
        payload['center_y'] = round(float(self.center_y), 1)
        return payload


# -----------------------------------------------------------------------------
# Shared parsing and naming helpers
# -----------------------------------------------------------------------------


def coerce_str_list(value, fallback=None) -> list[str]:
    """Normalize a parameter-like value into a string list."""
    if isinstance(value, str):
        cleaned = [item.strip() for item in value.split(',') if item.strip()]
        if cleaned:
            return cleaned
    elif isinstance(value, (list, tuple)):
        cleaned = [str(item).strip() for item in value if str(item).strip()]
        if cleaned:
            return cleaned
    return list(fallback or [])


def load_label_class_map(raw_value) -> dict[str, str]:
    """Parse label to KB class overrides from a JSON string or mapping."""
    if isinstance(raw_value, dict):
        parsed = raw_value
    else:
        try:
            parsed = json.loads(str(raw_value).strip() or '{}')
        except json.JSONDecodeError:
            parsed = {}

    label_map = dict(DEFAULT_LABEL_CLASS_MAP)
    if isinstance(parsed, dict):
        for key, value in parsed.items():
            label = normalize_label(key)
            kb_class = sanitize_kb_class(value)
            if label and kb_class:
                label_map[label] = kb_class
    return label_map


def normalize_label(raw_label: str) -> str:
    """Normalize detector labels into a consistent lookup key."""
    return ' '.join(str(raw_label or '').strip().lower().split())


def sanitize_kb_class(raw_value: str) -> str:
    """Convert a raw label or override into a KnowledgeCore-friendly class name."""
    parts = re.split(r'[^A-Za-z0-9]+', str(raw_value or '').strip())
    return ''.join(part[:1].upper() + part[1:] for part in parts if part)


def build_entity_id(
    *,
    entity_prefix: str,
    label: str,
    tracker_id: str,
    center_x: float,
    center_y: float,
) -> str:
    """Build a stable entity id from tracking data or a bbox fallback."""
    label_slug = re.sub(r'[^a-z0-9]+', '_', normalize_label(label)).strip('_') or 'object'
    if tracker_id:
        tracker_slug = re.sub(r'[^A-Za-z0-9]+', '_', tracker_id).strip('_') or 'track'
        return f'{entity_prefix}_{label_slug}_{tracker_slug}'
    return f'{entity_prefix}_{label_slug}_{int(center_x)}_{int(center_y)}'


# -----------------------------------------------------------------------------
# Backend-specific adapters
# -----------------------------------------------------------------------------


class YoloRosDetectionAdapter:
    """Translate `yolo_ros` detection messages into normalized observations."""

    def __init__(
        self,
        *,
        allowed_labels,
        label_class_map: dict[str, str],
        entity_prefix: str = 'detected',
        source: str = 'yolo_ros',
    ) -> None:
        self._allowed_labels = {normalize_label(item) for item in allowed_labels if item}
        self._label_class_map = dict(label_class_map)
        self._entity_prefix = str(entity_prefix or 'detected').strip() or 'detected'
        self._source = str(source or 'yolo_ros').strip() or 'yolo_ros'

    def parse_detections(self, msg, min_score: float = 0.0) -> list[ObjectObservation]:
        """Convert a `yolo_msgs/DetectionArray` message to normalized observations."""
        detections = getattr(msg, 'detections', [])
        observations: list[ObjectObservation] = []
        for detection in detections:
            label = normalize_label(getattr(detection, 'class_name', ''))
            if not label:
                continue
            if self._allowed_labels and label not in self._allowed_labels:
                continue

            score = float(getattr(detection, 'score', 0.0))
            if score < float(min_score):
                continue

            bbox = getattr(detection, 'bbox', None)
            center = getattr(getattr(bbox, 'center', None), 'position', None)
            center_x = float(getattr(center, 'x', 0.0))
            center_y = float(getattr(center, 'y', 0.0))
            tracker_id = str(getattr(detection, 'id', '')).strip()
            kb_class = self._label_class_map.get(label, sanitize_kb_class(label))
            observations.append(
                ObjectObservation(
                    entity_id=build_entity_id(
                        entity_prefix=self._entity_prefix,
                        label=label,
                        tracker_id=tracker_id,
                        center_x=center_x,
                        center_y=center_y,
                    ),
                    label=label,
                    kb_class=kb_class,
                    score=score,
                    tracker_id=tracker_id,
                    source=self._source,
                    center_x=center_x,
                    center_y=center_y,
                )
            )
        return observations


class EmorobcareDetectionAdapter:
    """Translate `emorobcare_cv_msgs/ObjectDetections` messages into observations."""

    def __init__(
        self,
        *,
        allowed_labels,
        label_class_map: dict[str, str],
        entity_prefix: str = 'detected',
        source: str = 'emorobcare_cv',
    ) -> None:
        self._allowed_labels = {normalize_label(item) for item in allowed_labels if item}
        self._label_class_map = dict(label_class_map)
        self._entity_prefix = str(entity_prefix or 'detected').strip() or 'detected'
        self._source = str(source or 'emorobcare_cv').strip() or 'emorobcare_cv'

    def parse_detections(self, msg, min_score: float = 0.0) -> list[ObjectObservation]:
        """Convert the emorobcare detector output into normalized observations."""
        observations: list[ObjectObservation] = []
        for detection in getattr(msg, 'detections', []):
            label = normalize_label(getattr(detection, 'label', ''))
            if not label:
                continue
            if self._allowed_labels and label not in self._allowed_labels:
                continue

            score = float(getattr(detection, 'confidence', 0.0))
            if score < float(min_score):
                continue

            x1 = float(getattr(detection, 'x1', 0.0))
            x2 = float(getattr(detection, 'x2', 0.0))
            y1 = float(getattr(detection, 'y1', 0.0))
            y2 = float(getattr(detection, 'y2', 0.0))
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            kb_class = self._label_class_map.get(label, sanitize_kb_class(label))
            observations.append(
                ObjectObservation(
                    entity_id=build_entity_id(
                        entity_prefix=self._entity_prefix,
                        label=label,
                        tracker_id='',
                        center_x=center_x,
                        center_y=center_y,
                    ),
                    label=label,
                    kb_class=kb_class,
                    score=score,
                    tracker_id='',
                    source=self._source,
                    center_x=center_x,
                    center_y=center_y,
                )
            )
        return observations
