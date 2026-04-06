from planner_common.contracts import EnrichedSnapshot
from planner_common.contracts import ExecutionFeedback
from planner_common.contracts import PlannerRequest
from planner_common.contracts import SceneSummary
from planner_common.contracts import build_plan_payload
from planner_common.contracts import build_world_model_text
from planner_common.contracts import extract_json_object
from planner_common.contracts import normalize_plan_steps
from planner_common.contracts import truncate_text


def test_planner_request_defaults_missing_fields() -> None:
    request = PlannerRequest.from_payload('{"user_text":"find the cup"}')
    assert request.user_text == 'find the cup'
    assert request.request_id.startswith('request_')
    assert request.normalized_intents == ()
    assert request.scene_targets == ()


def test_scene_summary_accepts_grounding_payload() -> None:
    summary = SceneSummary.from_payload(
        '{"observer":"myself","backend":"emorobcare_cv","objects":[{"entity_id":"cup_1","label":"cup","kb_class":"Cup","score":0.91,"source":"emorobcare_cv"}]}'
    )
    assert summary.backend == 'emorobcare_cv'
    assert len(summary.objects) == 1
    assert summary.objects[0].entity_id == 'cup_1'


def test_execution_feedback_parses_nested_step() -> None:
    feedback = ExecutionFeedback.from_payload(
        '{"plan_id":"plan_1","status":"failed","scene_targets":["cup"],"step":{"id":"step_2","type":"skill","name":"perform_motion"}}'
    )
    assert feedback.plan_id == 'plan_1'
    assert feedback.status == 'failed'
    assert feedback.scene_targets == ('cup',)
    assert feedback.step_id == 'step_2'


def test_extract_json_object_accepts_fenced_json() -> None:
    payload = '```json\n{"plan":{"plan_id":"plan_7"}}\n```'
    assert extract_json_object(payload) == {'plan': {'plan_id': 'plan_7'}}


def test_normalize_plan_steps_filters_invalid_step_types() -> None:
    steps = normalize_plan_steps(
        [
            {'type': 'say', 'args': {'text': 'hello'}},
            {'type': 'mystery', 'args': {}},
        ]
    )
    assert steps == [
        {
            'id': 'step_1',
            'type': 'say',
            'name': '',
            'args': {'text': 'hello'},
            'requires': [],
            'on_failure': 'fail',
            'retry_budget': 0,
        }
    ]


def test_build_plan_payload_keeps_shared_envelope_shape() -> None:
    request = PlannerRequest.from_payload(
        '{"request_id":"r1","user_text":"look at the cup","scene_targets":["cup"]}'
    )
    payload = build_plan_payload(
        request=request,
        ack_text='I will look at the cup.',
        steps=[{'type': 'look_at', 'name': 'look_at', 'args': {'target_frame': 'cup_frame'}}],
        validation_status='draft',
        retry_budget=1,
    )
    assert payload['scene_targets'] == ['cup']
    assert payload['plan']['scene_targets'] == ['cup']
    assert payload['plan']['validation_status'] == 'draft'
    assert payload['plan']['steps'][0]['args']['target_frame'] == 'cup_frame'


def test_build_world_model_text_bounds_output() -> None:
    snapshot = EnrichedSnapshot.from_payload(
        '{"observer":"myself","backend":"emorobcare_cv","active_plan_id":"plan_1","execution_status":"running","scene_targets":["cup"],"entities":[{"entity_id":"cup_1","label":"cup","kb_class":"Cup","state":"current","score":0.9,"source":"emorobcare_cv","is_plan_relevant":true,"risk_tags":["visible_now"]}],"kb_rows":[{"entity":"cup_1","type":"Cup"}]}'
    )
    text = build_world_model_text(snapshot, max_chars=180)
    assert 'Current world model context:' in text
    assert len(text) <= 180


def test_truncate_text_adds_ellipsis_when_needed() -> None:
    assert truncate_text('hello world', 5) == 'hell…'
