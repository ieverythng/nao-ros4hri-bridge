from nao_scene_grounding.scene_grounding_node import _TrackedObject
from nao_scene_grounding.scene_grounding_node import _kb_spatial_statements


def test_kb_spatial_statements_include_fov_and_geometry() -> None:
    tracked = _TrackedObject(
        entity_id='detected_cup_100_200',
        label='cup',
        kb_class='Cup',
        score=0.9132,
        tracker_id='',
        source='emorobcare_cv',
        center_x=320.5,
        center_y=240.25,
        last_seen_sec=1780275588.245,
    )

    statements = _kb_spatial_statements('myself', tracked)

    assert 'myself sees detected_cup_100_200' in statements
    assert 'detected_cup_100_200 rdf:type Cup' in statements
    assert 'detected_cup_100_200 inFieldOfViewOf myself' in statements
    assert 'detected_cup_100_200 hasVisualCenterX 320.5' in statements
    assert 'detected_cup_100_200 hasVisualCenterY 240.25' in statements
    assert 'detected_cup_100_200 hasDetectionScore 0.9132' in statements
    assert 'detected_cup_100_200 lastSeenSec 1780275588.245' in statements
    assert 'detected_cup_100_200 observedBy myself' in statements
    assert 'detected_cup_100_200 detectionSource emorobcare_cv' in statements
