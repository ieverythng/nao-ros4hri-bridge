"""Pure world-model state helpers for the WME node."""

from __future__ import annotations

from dataclasses import dataclass
import json
import sys
import time

from planner_common import EnrichedSnapshot
from planner_common import ExecutionFeedback
from planner_common import SceneSummary
from planner_common import build_world_model_text
from planner_common import coerce_str_list


_MUTABLE_DATACLASS_KWARGS = {}
if sys.version_info >= (3, 10):  # pragma: no branch - local macOS uses Python 3.9
    _MUTABLE_DATACLASS_KWARGS['slots'] = True


@dataclass(**_MUTABLE_DATACLASS_KWARGS)
class _TrackedEntity:
    entity_id: str
    label: str
    kb_class: str
    score: float
    source: str
    last_seen_sec: float
    tracker_id: str = ''
    center_x: float = 0.0
    center_y: float = 0.0
    currently_visible: bool = False

    @classmethod
    def from_scene_object(cls, scene_object, now_sec: float) -> '_TrackedEntity':
        return cls(
            entity_id=scene_object.entity_id,
            label=scene_object.label,
            kb_class=scene_object.kb_class,
            score=float(scene_object.score),
            source=scene_object.source,
            last_seen_sec=_resolve_last_seen(scene_object.last_seen_sec, now_sec),
            tracker_id=scene_object.tracker_id,
            center_x=float(scene_object.center_x),
            center_y=float(scene_object.center_y),
            currently_visible=True,
        )

    def refresh_from_scene_object(self, scene_object, now_sec: float) -> None:
        self.label = scene_object.label
        self.kb_class = scene_object.kb_class
        self.score = float(scene_object.score)
        self.source = scene_object.source
        self.last_seen_sec = _resolve_last_seen(scene_object.last_seen_sec, now_sec)
        self.tracker_id = scene_object.tracker_id
        self.center_x = float(scene_object.center_x)
        self.center_y = float(scene_object.center_y)
        self.currently_visible = True


def _resolve_last_seen(last_seen_sec: float, fallback_sec: float) -> float:
    try:
        resolved = float(last_seen_sec)
    except (TypeError, ValueError):
        resolved = 0.0
    if resolved > 0.0:
        return resolved
    return float(fallback_sec)


def _now_or_fallback(now_sec: float | None) -> float:
    if now_sec is None:
        return time.time()
    return float(now_sec)


def _stable_json(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), default=str)


class WorldModelState:
    """Maintain a short-horizon planner-facing world model."""

    def __init__(
        self,
        *,
        observer_name: str = 'myself',
        recent_after_sec: float = 4.0,
        occluded_after_sec: float = 10.0,
        stale_after_sec: float = 45.0,
        max_kb_rows: int = 12,
    ) -> None:
        self.observer_name = str(observer_name or 'myself').strip() or 'myself'
        self.recent_after_sec = max(0.1, float(recent_after_sec))
        self.occluded_after_sec = max(self.recent_after_sec, float(occluded_after_sec))
        self.stale_after_sec = max(self.occluded_after_sec, float(stale_after_sec))
        self.max_kb_rows = max(0, int(max_kb_rows))

        self.backend = ''
        self._tracked_entities: dict[str, _TrackedEntity] = {}
        self._kb_rows: list[dict] = []
        self._active_plan_id = ''
        self._execution_status = ''
        self._execution_reason = ''
        self._scene_targets: tuple[str, ...] = ()
    # ------------------------------------------------------------------
    # Ingestion
    # ------------------------------------------------------------------

    def apply_scene_summary(self, summary: SceneSummary, *, now_sec: float | None = None) -> None:
        now_value = _now_or_fallback(now_sec)
        if summary.observer:
            self.observer_name = summary.observer
        if summary.backend:
            self.backend = summary.backend

        for tracked in self._tracked_entities.values():
            tracked.currently_visible = False

        for scene_object in summary.objects:
            if not scene_object.entity_id:
                continue
            tracked = self._tracked_entities.get(scene_object.entity_id)
            if tracked is None:
                self._tracked_entities[scene_object.entity_id] = _TrackedEntity.from_scene_object(
                    scene_object,
                    now_value,
                )
                continue
            tracked.refresh_from_scene_object(scene_object, now_value)

        self._prune_expired_entities(now_value)

    def apply_execution_feedback(
        self,
        feedback: ExecutionFeedback,
        *,
        now_sec: float | None = None,
    ) -> None:
        now_value = _now_or_fallback(now_sec)
        if feedback.plan_id:
            self._active_plan_id = feedback.plan_id
        if feedback.status:
            self._execution_status = feedback.status
        self._execution_reason = feedback.reason or feedback.replan_hint
        if feedback.scene_targets:
            self._scene_targets = feedback.scene_targets
        elif feedback.status in ('completed', 'failed', 'invalid'):
            self._scene_targets = ()
        self._prune_expired_entities(now_value)

    def refresh_kb_rows(self, rows: list[dict], *, now_sec: float | None = None) -> None:
        now_value = _now_or_fallback(now_sec)
        deduped_rows: list[dict] = []
        seen: set[str] = set()
        for row in rows:
            if not isinstance(row, dict):
                continue
            row_key = _stable_json(row)
            if row_key in seen:
                continue
            seen.add(row_key)
            deduped_rows.append(dict(row))
        self._kb_rows = deduped_rows[: self.max_kb_rows]

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def build_snapshot_payload(self, *, now_sec: float | None = None) -> dict:
        now_value = _now_or_fallback(now_sec)
        self._prune_expired_entities(now_value)

        entities: list[dict] = []
        for tracked in self._sorted_entities():
            enriched = self._enriched_entity_dict(tracked, now_value)
            if enriched is None:
                continue
            entities.append(enriched)

        return {
            'observer': self.observer_name,
            'backend': self.backend,
            'active_plan_id': self._active_plan_id,
            'execution_status': self._execution_status,
            'execution_reason': self._execution_reason,
            'scene_targets': list(self._scene_targets),
            'entities': entities,
            'kb_rows': list(self._kb_rows),
            'timestamp_sec': round(now_value, 3),
        }

    def build_snapshot(self, *, now_sec: float | None = None) -> EnrichedSnapshot:
        return EnrichedSnapshot.from_payload(self.build_snapshot_payload(now_sec=now_sec))

    def build_text(self, *, now_sec: float | None = None, max_chars: int = 2400) -> str:
        return build_world_model_text(
            self.build_snapshot(now_sec=now_sec),
            max_chars=max_chars,
        )

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    @property
    def scene_targets(self) -> tuple[str, ...]:
        return self._scene_targets

    def build_kb_query_patterns(self, templates) -> list[str]:
        clean_templates = coerce_str_list(templates)
        if not clean_templates:
            return []

        observer = self.observer_name or 'myself'
        patterns: list[str] = []
        for template in clean_templates:
            pattern = str(template).replace('{observer}', observer)
            if pattern:
                patterns.append(pattern)
        return patterns

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sorted_entities(self) -> list[_TrackedEntity]:
        return sorted(
            self._tracked_entities.values(),
            key=lambda entity: (
                not self._is_plan_relevant(entity),
                0 if entity.currently_visible else 1,
                entity.label,
                entity.entity_id,
            ),
        )

    def _enriched_entity_dict(self, tracked: _TrackedEntity, now_sec: float) -> dict | None:
        age_sec = max(0.0, now_sec - tracked.last_seen_sec)
        if age_sec > self.stale_after_sec:
            return None

        state = self._entity_state(tracked, age_sec)
        risk_tags = self._risk_tags(tracked, state)
        return {
            'entity_id': tracked.entity_id,
            'label': tracked.label,
            'kb_class': tracked.kb_class,
            'state': state,
            'score': round(float(tracked.score), 3),
            'source': tracked.source,
            'last_seen_sec': round(float(tracked.last_seen_sec), 3),
            'age_sec': round(float(age_sec), 3),
            'is_plan_relevant': self._is_plan_relevant(tracked),
            'risk_tags': risk_tags,
        }

    def _entity_state(self, tracked: _TrackedEntity, age_sec: float) -> str:
        if tracked.currently_visible:
            return 'current'
        if age_sec <= self.recent_after_sec:
            return 'recent'
        if age_sec <= self.occluded_after_sec and self._is_plan_relevant(tracked):
            return 'occluded'
        return 'stale'

    def _is_plan_relevant(self, tracked: _TrackedEntity) -> bool:
        if not self._scene_targets:
            return False
        candidates = {
            tracked.entity_id.lower(),
            tracked.label.lower(),
            tracked.kb_class.lower(),
        }
        for target in self._scene_targets:
            clean_target = str(target or '').strip().lower()
            if clean_target and clean_target in candidates:
                return True
        return False

    def _risk_tags(self, tracked: _TrackedEntity, state: str) -> list[str]:
        tags: list[str] = []
        is_plan_relevant = self._is_plan_relevant(tracked)

        if state == 'current':
            tags.append('visible_now')
        elif state == 'recent':
            tags.append('recently_seen')
        elif state == 'occluded':
            tags.append('likely_occluded')
        else:
            tags.append('stale_memory')

        if is_plan_relevant:
            tags.append('plan_target')
            if self._execution_status in ('accepted', 'running'):
                tags.append('execution_active')
            if self._execution_status in ('failed', 'invalid'):
                tags.append('execution_blocked')

        if self._execution_reason and is_plan_relevant:
            tags.append('needs_attention')
        return tags

    def _prune_expired_entities(self, now_sec: float) -> None:
        expired_ids = [
            entity_id
            for entity_id, tracked in self._tracked_entities.items()
            if now_sec - tracked.last_seen_sec > self.stale_after_sec
        ]
        for entity_id in expired_ids:
            self._tracked_entities.pop(entity_id, None)
