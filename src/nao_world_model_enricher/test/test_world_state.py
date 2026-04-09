from planner_common import ExecutionFeedback
from planner_common import SceneSummary

from nao_world_model_enricher.world_state import WorldModelState


SCENE_CUP = SceneSummary.from_payload(
    {
        'observer': 'myself',
        'backend': 'emorobcare_cv',
        'objects': [
            {
                'entity_id': 'cup_1',
                'label': 'cup',
                'kb_class': 'Cup',
                'score': 0.91,
                'source': 'emorobcare_cv',
                'last_seen_sec': 10.0,
            }
        ],
    }
)

EMPTY_SCENE = SceneSummary.from_payload(
    {
        'observer': 'myself',
        'backend': 'emorobcare_cv',
        'objects': [],
    }
)


def _find_entity(snapshot, entity_id: str) -> dict:
    for entity in snapshot['entities']:
        if entity['entity_id'] == entity_id:
            return entity
    raise AssertionError('entity %s not found' % entity_id)


def test_world_state_transitions_from_current_to_recent_to_stale() -> None:
    state = WorldModelState(recent_after_sec=2.0, occluded_after_sec=5.0, stale_after_sec=12.0)
    state.apply_scene_summary(SCENE_CUP, now_sec=10.0)
    current = _find_entity(state.build_snapshot_payload(now_sec=10.0), 'cup_1')
    assert current['state'] == 'current'

    state.apply_scene_summary(EMPTY_SCENE, now_sec=11.0)
    recent = _find_entity(state.build_snapshot_payload(now_sec=12.0), 'cup_1')
    assert recent['state'] == 'recent'

    stale = _find_entity(state.build_snapshot_payload(now_sec=18.0), 'cup_1')
    assert stale['state'] == 'stale'
    assert 'stale_memory' in stale['risk_tags']


def test_world_state_marks_plan_relevant_objects_as_occluded() -> None:
    state = WorldModelState(recent_after_sec=2.0, occluded_after_sec=6.0, stale_after_sec=20.0)
    state.apply_scene_summary(SCENE_CUP, now_sec=10.0)
    state.apply_execution_feedback(
        ExecutionFeedback.from_payload(
            {
                'plan_id': 'plan_1',
                'status': 'running',
                'scene_targets': ['cup'],
            }
        ),
        now_sec=11.0,
    )
    state.apply_scene_summary(EMPTY_SCENE, now_sec=12.0)

    occluded = _find_entity(state.build_snapshot_payload(now_sec=15.5), 'cup_1')
    assert occluded['state'] == 'occluded'
    assert occluded['is_plan_relevant'] is True
    assert 'likely_occluded' in occluded['risk_tags']
    assert 'execution_active' in occluded['risk_tags']


def test_world_state_propagates_execution_blockers_into_risk_tags() -> None:
    state = WorldModelState(recent_after_sec=2.0, occluded_after_sec=6.0, stale_after_sec=20.0)
    state.apply_scene_summary(SCENE_CUP, now_sec=10.0)
    state.apply_execution_feedback(
        ExecutionFeedback.from_payload(
            {
                'plan_id': 'plan_2',
                'status': 'failed',
                'reason': 'path blocked',
                'scene_targets': ['cup'],
                'step': {'id': 'step_2', 'type': 'skill', 'name': 'approach'},
            }
        ),
        now_sec=11.0,
    )

    snapshot = state.build_snapshot_payload(now_sec=11.5)
    entity = _find_entity(snapshot, 'cup_1')
    assert snapshot['execution_reason'] == 'path blocked'
    assert 'execution_blocked' in entity['risk_tags']
    assert 'needs_attention' in entity['risk_tags']


def test_world_state_keeps_kb_rows_bounded_and_builds_text_summary() -> None:
    state = WorldModelState(max_kb_rows=1)
    state.apply_scene_summary(SCENE_CUP, now_sec=10.0)
    state.refresh_kb_rows(
        [
            {'entity': 'cup_1', 'type': 'Cup'},
            {'entity': 'bottle_1', 'type': 'Bottle'},
        ],
        now_sec=11.0,
    )

    snapshot = state.build_snapshot_payload(now_sec=11.0)
    assert snapshot['kb_rows'] == [{'entity': 'cup_1', 'type': 'Cup'}]

    text = state.build_text(now_sec=11.0, max_chars=220)
    assert 'Current world model context:' in text
    assert len(text) <= 220
