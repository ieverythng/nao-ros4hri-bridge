from planner_common.contracts import EnrichedSnapshot
from planner_common.contracts import ExecutionFeedback
from planner_common.contracts import PlannerDialogueAct
from planner_common.contracts import PlannerRequest
from planner_common.contracts import SceneSummary
from planner_common.contracts import build_dialogue_act_payload
from planner_common.contracts import build_execution_feedback_payload
from planner_common.contracts import build_plan_payload
from planner_common.contracts import build_world_model_text
from planner_common.contracts import extract_json_object
from planner_common.contracts import normalize_grounded_context
from planner_common.contracts import normalize_plan_steps
from planner_common.contracts import truncate_text


def test_planner_request_defaults_missing_fields() -> None:
    request = PlannerRequest.from_payload('{"user_text":"find the cup"}')
    assert request.user_text == 'find the cup'
    assert request.goal_text == ''
    assert request.request_id.startswith('request_')
    assert request.goal_id.startswith('goal_')
    assert request.request_kind == 'new_goal'
    assert request.normalized_intents == ()
    assert request.requested_plan == ()
    assert request.scene_targets == ()
    assert request.grounded_context == {
        'knowledge_snapshot': {},
        'scene_summary': {},
        'world_model_snapshot': {},
        'world_model_text': '',
    }


def test_planner_request_parses_goal_text_aliases() -> None:
    request = PlannerRequest.from_payload(
        {
            'goal_text': 'inspect the cup and report completion',
            'user_text': 'can you inspect the cup and tell me when done',
        }
    )

    assert request.goal_text == 'inspect the cup and report completion'
    assert request.user_text == 'can you inspect the cup and tell me when done'

    alias_request = PlannerRequest.from_payload({'goal': 'look at the book'})
    assert alias_request.goal_text == 'look at the book'


def test_planner_request_keeps_supervisor_metadata() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'turn_7',
            'goal_id': 'goal_7',
            'parent_goal_id': 'goal_parent',
            'supersedes_goal_id': 'goal_old',
            'request_kind': 'clarification_answer',
            'interaction_mode': 'supervised',
            'dialogue_turn_id': 'dialogue_9',
            'grounded_context': {
                'knowledge_snapshot': {'cup': True},
                'scene_summary': {'objects': ['cup']},
                'world_model_text': 'cup visible',
            },
            'requested_plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'stand'},
                }
            ],
        }
    )
    assert request.goal_id == 'goal_7'
    assert request.parent_goal_id == 'goal_parent'
    assert request.supersedes_goal_id == 'goal_old'
    assert request.request_kind == 'clarification_answer'
    assert request.interaction_mode == 'supervised'
    assert request.dialogue_turn_id == 'dialogue_9'
    assert request.grounded_context['knowledge_snapshot'] == {'cup': True}
    assert request.grounded_context['scene_summary'] == {'objects': ['cup']}
    assert request.grounded_context['world_model_snapshot'] == {}
    assert request.grounded_context['world_model_text'] == 'cup visible'
    assert request.requested_plan == (
        {
            'id': 'step_1',
            'type': 'skill',
            'name': 'perform_motion',
            'args': {'object': 'stand'},
            'requires': [],
            'on_failure': 'fail',
            'retry_budget': 0,
        },
    )


def test_scene_summary_accepts_grounding_payload() -> None:
    summary = SceneSummary.from_payload(
        '{"observer":"myself","backend":"emorobcare_cv","objects":[{"entity_id":"cup_1","label":"cup","kb_class":"Cup","score":0.91,"source":"emorobcare_cv"}]}'
    )
    assert summary.backend == 'emorobcare_cv'
    assert len(summary.objects) == 1
    assert summary.objects[0].entity_id == 'cup_1'


def test_execution_feedback_parses_nested_step_and_supervisor_fields() -> None:
    feedback = ExecutionFeedback.from_payload(
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":2,"event_type":"step_failed","status":"failed","blocking":true,"needs_user_input":true,"unmet_preconditions":["cup_visible"],"scene_targets":["cup"],"step":{"id":"step_2","type":"skill","name":"perform_motion","retry_budget":1,"on_failure":"replan","requires":["cup_visible"]}}'
    )
    assert feedback.goal_id == 'goal_1'
    assert feedback.plan_id == 'plan_1'
    assert feedback.plan_version == 2
    assert feedback.event_type == 'step_failed'
    assert feedback.blocking is True
    assert feedback.needs_user_input is True
    assert feedback.unmet_preconditions == ('cup_visible',)
    assert feedback.scene_targets == ('cup',)
    assert feedback.step_id == 'step_2'
    assert feedback.step_retry_budget == 1
    assert feedback.step_on_failure == 'replan'
    assert feedback.step_requires == ('cup_visible',)
    assert feedback.result_summary == ''


def test_execution_feedback_result_summary_round_trips() -> None:
    payload = build_execution_feedback_payload(
        intent='raw_user_input',
        source='nao_orchestrator',
        plan_context={'goal_id': 'g1', 'plan_id': 'p1', 'plan_version': 1},
        status='completed',
        event_type='step_succeeded',
        reason='ok',
        result_summary='Scan summary: two faces.',
    )
    assert payload['result_summary'] == 'Scan summary: two faces.'
    feedback = ExecutionFeedback.from_payload(payload)
    assert feedback.result_summary == 'Scan summary: two faces.'


def test_execution_feedback_event_type_defaults_succeeded_to_step_succeeded() -> None:
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'g1',
            'plan_id': 'p1',
            'plan_version': 1,
            'status': 'succeeded',
        }
    )

    assert feedback.event_type == 'step_succeeded'


def test_planner_dialogue_act_payload_round_trips() -> None:
    payload = build_dialogue_act_payload(
        goal_id='goal_8',
        plan_id='plan_8',
        plan_version=3,
        act='ask_clarification',
        await_user_response=True,
        reason='missing target',
        text_hint='Which cup do you mean?',
        slots_needed=['target_object'],
        context={'scene_targets': ['cup']},
    )
    act = PlannerDialogueAct.from_payload(payload)
    assert act.goal_id == 'goal_8'
    assert act.act == 'ask_clarification'
    assert act.await_user_response is True
    assert act.slots_needed == ('target_object',)
    assert act.context == {'scene_targets': ['cup']}


def test_execution_feedback_builder_keeps_supervisor_shape() -> None:
    payload = build_execution_feedback_payload(
        intent='raw_user_input',
        source='nao_orchestrator',
        plan_context={
            'goal_id': 'goal_1',
            'plan_id': 'plan_1',
            'plan_version': 3,
            'validation_status': 'draft',
            'replan_hint': 'try_again',
            'retry_budget': 1,
            'scene_targets': ['cup'],
        },
        status='failed',
        event_type='step_failed',
        reason='cup left the scene',
        step={'id': 'step_1', 'type': 'look_at', 'name': 'look_at'},
        blocking=True,
        unmet_preconditions=['cup_visible'],
        needs_user_input=True,
        validation_errors=['step_1: missing target frame'],
    )
    feedback = ExecutionFeedback.from_payload(payload)
    assert feedback.goal_id == 'goal_1'
    assert feedback.plan_version == 3
    assert feedback.event_type == 'step_failed'
    assert feedback.blocking is True
    assert feedback.unmet_preconditions == ('cup_visible',)
    assert feedback.needs_user_input is True
    assert feedback.validation_errors == ('step_1: missing target frame',)
    assert feedback.step_retry_budget == 1
    assert feedback.step_on_failure == 'fail'
    assert feedback.step_requires == ()


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


def test_build_plan_payload_keeps_supervisor_envelope_shape() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r1',
            'goal_id': 'goal_1',
            'user_text': 'look at the cup',
            'scene_targets': ['cup'],
        }
    )
    payload = build_plan_payload(
        request=request,
        ack_text='I will look at the cup.',
        steps=[{'type': 'look_at', 'name': 'look_at', 'args': {'target_frame': 'cup_frame'}}],
        validation_status='draft',
        retry_budget=1,
        plan_version=4,
        status='executing',
        communication_policy={'emit_progress': True},
    )
    assert payload['goal_id'] == 'goal_1'
    assert payload['grounded_context']['knowledge_snapshot'] == {}
    assert payload['scene_targets'] == ['cup']
    assert payload['plan']['goal_id'] == 'goal_1'
    assert payload['plan']['plan_version'] == 4
    assert payload['plan']['status'] == 'executing'
    assert payload['plan']['scene_targets'] == ['cup']
    assert payload['plan']['communication_policy']['emit_progress'] is True
    assert payload['plan']['steps'][0]['args']['target_frame'] == 'cup_frame'


def test_normalize_grounded_context_stabilizes_missing_sections() -> None:
    grounded_context = normalize_grounded_context(
        {'knowledge_snapshot': {'cup': True}, 'world_model_text': 'cup visible'}
    )
    assert grounded_context == {
        'knowledge_snapshot': {'cup': True},
        'scene_summary': {},
        'world_model_snapshot': {},
        'world_model_text': 'cup visible',
    }


def test_build_world_model_text_bounds_output() -> None:
    snapshot = EnrichedSnapshot.from_payload(
        '{"observer":"myself","backend":"emorobcare_cv","active_plan_id":"plan_1","execution_status":"running","scene_targets":["cup"],"entities":[{"entity_id":"cup_1","label":"cup","kb_class":"Cup","state":"current","score":0.9,"source":"emorobcare_cv","is_plan_relevant":true,"risk_tags":["visible_now"]}],"kb_rows":[{"entity":"cup_1","type":"Cup"}]}'
    )
    text = build_world_model_text(snapshot, max_chars=180)
    assert 'Current world model context:' in text
    assert len(text) <= 180


def test_truncate_text_adds_ellipsis_when_needed() -> None:
    assert truncate_text('hello world', 5) == 'hell…'
