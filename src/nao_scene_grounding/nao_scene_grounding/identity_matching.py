"""Identity-stabilization helpers for detector backends without tracker ids.

`emorobcare_cv` emits per-frame detections without a persistent tracker id. The
grounding node uses these helpers to reuse recently seen entity ids when a new
detection is close to an existing tracked object with the same semantic label.
"""

from __future__ import annotations

from dataclasses import replace
from math import hypot
from typing import Iterable
from typing import Protocol

from nao_scene_grounding.detector_adapters import ObjectObservation


class TrackedObservationLike(Protocol):
    """Structural type for tracked objects kept by the grounding node."""

    entity_id: str
    label: str
    kb_class: str
    source: str
    center_x: float
    center_y: float
    last_seen_sec: float


def reconcile_observation_entity_ids(
    observations: list[ObjectObservation],
    tracked_objects: Iterable[TrackedObservationLike],
    *,
    now_sec: float,
    max_match_distance_px: float,
    max_match_age_sec: float,
) -> list[ObjectObservation]:
    """Reuse nearby tracked entity ids for detections that lack tracker ids.

    The detector remains the source of raw boxes, while scene grounding owns the
    softer notion of identity persistence that is needed for KB facts and scene
    summaries.
    """

    if not observations:
        return []

    tracked_snapshot = tuple(tracked_objects)
    claimed_entity_ids: set[str] = set()
    reconciled: list[ObjectObservation] = []

    for observation in sorted(observations, key=_reconciliation_order):
        entity_id = observation.entity_id
        if not observation.tracker_id:
            matched = match_observation_to_tracked(
                observation,
                tracked_snapshot,
                claimed_entity_ids=claimed_entity_ids,
                now_sec=now_sec,
                max_match_distance_px=max_match_distance_px,
                max_match_age_sec=max_match_age_sec,
            )
            if matched is not None:
                entity_id = matched.entity_id

        claimed_entity_ids.add(entity_id)
        if entity_id != observation.entity_id:
            observation = replace(observation, entity_id=entity_id)
        reconciled.append(observation)

    return reconciled


def _reconciliation_order(observation: ObjectObservation) -> tuple[bool, float, str]:
    """Prefer tracker-backed and higher-confidence observations first."""
    return (
        not bool(observation.tracker_id),
        -float(observation.score),
        observation.entity_id,
    )


def match_observation_to_tracked(
    observation: ObjectObservation,
    tracked_objects: Iterable[TrackedObservationLike],
    *,
    claimed_entity_ids: set[str],
    now_sec: float,
    max_match_distance_px: float,
    max_match_age_sec: float,
) -> TrackedObservationLike | None:
    """Find the best recent tracked object match for a tracker-less detection."""

    if max_match_distance_px <= 0.0 or max_match_age_sec <= 0.0:
        return None

    best_match = None
    best_score = None
    for tracked in tracked_objects:
        candidate_score = _match_score(
            observation,
            tracked,
            claimed_entity_ids=claimed_entity_ids,
            now_sec=now_sec,
            max_match_distance_px=max_match_distance_px,
            max_match_age_sec=max_match_age_sec,
        )
        if candidate_score is None:
            continue

        if best_score is None or candidate_score < best_score:
            best_match = tracked
            best_score = candidate_score

    return best_match


def _match_score(
    observation: ObjectObservation,
    tracked: TrackedObservationLike,
    *,
    claimed_entity_ids: set[str],
    now_sec: float,
    max_match_distance_px: float,
    max_match_age_sec: float,
) -> tuple[float, float, str] | None:
    """Return the ranking tuple for one candidate match, or `None` if invalid."""
    if tracked.entity_id in claimed_entity_ids:
        return None
    if tracked.label != observation.label:
        return None
    if tracked.kb_class != observation.kb_class:
        return None
    if tracked.source != observation.source:
        return None

    age_sec = max(0.0, float(now_sec) - float(tracked.last_seen_sec))
    if age_sec > float(max_match_age_sec):
        return None

    distance_px = hypot(
        float(observation.center_x) - float(tracked.center_x),
        float(observation.center_y) - float(tracked.center_y),
    )
    if distance_px > float(max_match_distance_px):
        return None

    return (
        round(distance_px, 6),
        age_sec,
        tracked.entity_id,
    )
