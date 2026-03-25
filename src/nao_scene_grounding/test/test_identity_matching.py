from dataclasses import dataclass

from nao_scene_grounding.detector_adapters import ObjectObservation
from nao_scene_grounding.identity_matching import reconcile_observation_entity_ids


@dataclass(frozen=True)
class _TrackedObject:
    entity_id: str
    label: str
    kb_class: str
    source: str
    center_x: float
    center_y: float
    last_seen_sec: float


def _observation(
    *,
    entity_id: str,
    label: str,
    kb_class: str = 'Blueberry',
    score: float = 0.75,
    tracker_id: str = '',
    source: str = 'emorobcare_cv',
    center_x: float,
    center_y: float,
) -> ObjectObservation:
    return ObjectObservation(
        entity_id=entity_id,
        label=label,
        kb_class=kb_class,
        score=score,
        tracker_id=tracker_id,
        source=source,
        center_x=center_x,
        center_y=center_y,
    )


def test_reconcile_observation_entity_ids_reuses_recent_nearby_object_ids():
    tracked = [
        _TrackedObject(
            entity_id='detected_blueberry_512_377',
            label='blueberry',
            kb_class='Blueberry',
            source='emorobcare_cv',
            center_x=512.0,
            center_y=377.0,
            last_seen_sec=10.0,
        )
    ]
    observations = [
        _observation(
            entity_id='detected_blueberry_510_376',
            label='blueberry',
            center_x=510.0,
            center_y=376.0,
        )
    ]

    reconciled = reconcile_observation_entity_ids(
        observations,
        tracked,
        now_sec=10.3,
        max_match_distance_px=32.0,
        max_match_age_sec=2.0,
    )

    assert reconciled[0].entity_id == 'detected_blueberry_512_377'


def test_reconcile_observation_entity_ids_does_not_match_stale_or_far_objects():
    tracked = [
        _TrackedObject(
            entity_id='detected_blueberry_512_377',
            label='blueberry',
            kb_class='Blueberry',
            source='emorobcare_cv',
            center_x=512.0,
            center_y=377.0,
            last_seen_sec=1.0,
        )
    ]
    observations = [
        _observation(
            entity_id='detected_blueberry_600_400',
            label='blueberry',
            center_x=600.0,
            center_y=400.0,
        )
    ]

    reconciled = reconcile_observation_entity_ids(
        observations,
        tracked,
        now_sec=5.0,
        max_match_distance_px=24.0,
        max_match_age_sec=2.0,
    )

    assert reconciled[0].entity_id == 'detected_blueberry_600_400'


def test_reconcile_observation_entity_ids_keeps_multiple_nearby_objects_distinct():
    tracked = [
        _TrackedObject(
            entity_id='detected_blueberry_left',
            label='blueberry',
            kb_class='Blueberry',
            source='emorobcare_cv',
            center_x=120.0,
            center_y=220.0,
            last_seen_sec=8.0,
        ),
        _TrackedObject(
            entity_id='detected_blueberry_right',
            label='blueberry',
            kb_class='Blueberry',
            source='emorobcare_cv',
            center_x=240.0,
            center_y=220.0,
            last_seen_sec=8.0,
        ),
    ]
    observations = [
        _observation(
            entity_id='detected_blueberry_118_221',
            label='blueberry',
            score=0.92,
            center_x=118.0,
            center_y=221.0,
        ),
        _observation(
            entity_id='detected_blueberry_243_219',
            label='blueberry',
            score=0.89,
            center_x=243.0,
            center_y=219.0,
        ),
    ]

    reconciled = reconcile_observation_entity_ids(
        observations,
        tracked,
        now_sec=8.2,
        max_match_distance_px=32.0,
        max_match_age_sec=2.0,
    )

    assert {item.entity_id for item in reconciled} == {
        'detected_blueberry_left',
        'detected_blueberry_right',
    }


def test_reconcile_observation_entity_ids_preserves_tracker_backed_ids():
    tracked = [
        _TrackedObject(
            entity_id='detected_blueberry_left',
            label='blueberry',
            kb_class='Blueberry',
            source='emorobcare_cv',
            center_x=120.0,
            center_y=220.0,
            last_seen_sec=8.0,
        )
    ]
    observations = [
        _observation(
            entity_id='detected_blueberry_tracker_7',
            label='blueberry',
            tracker_id='7',
            center_x=118.0,
            center_y=221.0,
        )
    ]

    reconciled = reconcile_observation_entity_ids(
        observations,
        tracked,
        now_sec=8.2,
        max_match_distance_px=32.0,
        max_match_age_sec=2.0,
    )

    assert reconciled[0].entity_id == 'detected_blueberry_tracker_7'
