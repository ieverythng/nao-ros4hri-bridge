from planner_common.contracts import ExecutionFeedback
from planner_common.contracts import PlannerDialogueAct
from planner_common.contracts import PlannerRequest
from planner_common.contracts import SceneSummary
from planner_common.contracts import build_dialogue_act_payload
from planner_common.contracts import build_execution_feedback_payload
from planner_common.contracts import build_plan_payload
from planner_common.contracts import extract_json_object
from planner_common.contracts import normalize_grounded_context
from planner_common.contracts import normalize_plan_steps
from planner_common.contracts import project_llm_grounded_context
from planner_common.contracts import truncate_text


def test_planner_request_defaults_missing_fields() -> None:
    request = PlannerRequest.from_payload('{"user_text":"find the cup"}')
    assert request.user_text == 'find the cup'
    assert request.goal_text == ''
    assert request.request_id.startswith('request_')
    assert request.goal_id.startswith('goal_')
    assert request.request_kind == 'new_goal'
    assert request.normalized_intents == ()
    assert request.scene_targets == ()
    assert request.grounded_context == {
        'knowledge_snapshot': {},
        'scene_summary': {},
        'state_t0': {},
    }


def test_planner_request_keeps_supervisor_metadata() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'turn_7',
            'goal_id': 'goal_7',
            'parent_goal_id': 'goal_parent',
            'supersedes_goal_id': 'goal_old',
            'request_kind': 'clarification_answer',
            'dialogue_turn_id': 'dialogue_9',
            'grounded_context': {
                'knowledge_snapshot': {'cup': True},
                'scene_summary': {'objects': ['cup']},
                'state_t0': {'observer': 'myself'},
            },
        }
    )
    assert request.goal_id == 'goal_7'
    assert request.parent_goal_id == 'goal_parent'
    assert request.supersedes_goal_id == 'goal_old'
    assert request.request_kind == 'clarification_answer'
    assert request.dialogue_turn_id == 'dialogue_9'
    assert request.grounded_context['knowledge_snapshot'] == {'cup': True}
    assert request.grounded_context['scene_summary'] == {'objects': ['cup']}
    assert request.grounded_context['state_t0'] == {'observer': 'myself'}


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
    assert feedback.result_payload == {}


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
    assert feedback.result_payload == {}


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


def test_build_plan_payload_keeps_nested_canonical_shape() -> None:
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
        steps=[{'type': 'look_at', 'name': 'look_at', 'args': {'target_frame': 'cup_frame'}}],
        validation_status='draft',
        retry_budget=1,
        plan_version=4,
        status='executing',
        communication_policy={'emit_progress': True},
    )
    assert payload['plan']['goal_id'] == 'goal_1'
    assert payload['plan']['plan_version'] == 4
    assert payload['plan']['status'] == 'executing'
    assert payload['plan']['scene_targets'] == ['cup']
    assert 'context_ref' not in payload['plan']
    assert payload['plan']['communication_policy']['emit_progress'] is True
    assert payload['plan']['communication_policy_source'] == ''
    assert payload['plan']['steps'][0]['args']['target_frame'] == 'cup_frame'
    assert 'failure_reason' not in payload['plan']
    assert 'goal_id' not in payload
    assert 'scene_targets' not in payload
    assert 'grounded_context' not in payload


def test_build_plan_payload_forces_completion_off_for_report_result() -> None:
    request = PlannerRequest.from_payload({'goal_id': 'goal_1', 'request_id': 'r1'})

    payload = build_plan_payload(
        request=request,
        steps=[
            {'type': 'skill', 'name': 'scan', 'args': {}},
            {'type': 'skill', 'name': 'report_result', 'args': {}},
        ],
        communication_policy={'emit_completion': True},
    )

    assert payload['plan']['communication_policy']['emit_completion'] is False


def test_build_plan_payload_keeps_completion_for_non_speaking_plan() -> None:
    request = PlannerRequest.from_payload({'goal_id': 'goal_1', 'request_id': 'r1'})

    payload = build_plan_payload(
        request=request,
        steps=[{'type': 'skill', 'name': 'perform_motion', 'args': {'object': 'stand'}}],
        communication_policy={'emit_completion': True},
    )

    assert payload['plan']['communication_policy']['emit_completion'] is True


def test_normalize_grounded_context_stabilizes_missing_sections() -> None:
    grounded_context = normalize_grounded_context(
        {'knowledge_snapshot': {'cup': True}, 'state_t0': {'observer': 'myself'}}
    )
    assert grounded_context == {
        'knowledge_snapshot': {'cup': True},
        'scene_summary': {},
        'state_t0': {'observer': 'myself'},
    }


def test_project_llm_grounded_context_filters_backend_noise_and_relations() -> None:
    projected = project_llm_grounded_context(
        {
            'scene_summary': {
                'schema_version': 'scene_summary_v2',
                'observer': 'myself',
                'backend': 'emorobcare_cv',
                'objects': [
                    {
                        'entity_id': 'cup_jrjic',
                        'label': 'cup_jrjic',
                        'kb_class': 'Tableware',
                        'score': 0.8,
                        'source': 'knowledge_snapshot',
                        'center_x': 12.0,
                        'center_y': 44.0,
                        'last_seen_sec': 1777040000.0,
                    }
                ],
                'people': [
                    {
                        'id': 'anonymous_person_ehfbf',
                        'label': 'anonymous_person_ehfbf',
                        'type': 'Human',
                        'source': 'hri_tracked_persons',
                    }
                ],
            },
        },
        knowledge_rows=[
            {'entity': 'cup_jrjic', 'predicate': 'rdf:type', 'object': 'dbr:Cup'},
            {'entity': 'cup_jrjic', 'predicate': 'isOn', 'object': 'table_1'},
            {'entity': 'cup_jrjic', 'predicate': 'seenBy', 'object': 'myself'},
        ],
    )

    assert 'counts' not in projected
    cup = next(item for item in projected['entities'] if item['id'] == 'cup_jrjic')
    person = next(
        item for item in projected['entities'] if item['id'] == 'anonymous_person_ehfbf'
    )
    assert cup['label'] == 'cup'
    assert cup['kind'] == 'object'
    assert {'predicate': 'oro:isOn', 'object': 'table_1'} in cup['relations']
    assert {'predicate': 'seenBy', 'object': 'myself'} not in cup['relations']
    assert person['label'] is None
    forbidden = {'schema_version', 'source', 'backend', 'center_x', 'center_y', 'score', 'last_seen_sec'}
    assert forbidden.isdisjoint(cup.keys())


def test_project_llm_grounded_context_prioritizes_stable_entities_over_detector_churn() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {'objects': [], 'people': []}},
        knowledge_rows=[
            {'entity': 'detected_blueberry_pvrts', 'predicate': 'rdf:type', 'object': 'dbr:Blueberry'},
            {'entity': 'cup_qgqrd', 'predicate': 'rdf:type', 'object': 'dbr:Cup'},
            {'entity': 'cup_qgqrd', 'predicate': 'dbp:color', 'object': 'gold'},
        ],
    )

    assert [item['id'] for item in projected['entities']] == [
        'cup_qgqrd',
        'detected_blueberry_pvrts',
    ]
    assert projected['entities'][0]['relations'] == [
        {'predicate': 'dbp:color', 'object': 'gold'}
    ]


def test_project_llm_grounded_context_keeps_state_t0_only_when_enabled() -> None:
    raw_context = {
        'scene_summary': {},
        'state_t0': {'observer': 'myself', 'entities': [{'id': 'cup_1', 'type': 'Cup'}]},
    }

    assert 'state_t0' not in project_llm_grounded_context(raw_context)
    assert project_llm_grounded_context(
        raw_context,
        include_state_t0=True,
    )['state_t0'] == raw_context['state_t0']


def test_normalize_grounded_context_accepts_compact_shape() -> None:
    grounded_context = normalize_grounded_context(
        {
            'entities': [
                {
                    'id': 'cup_1',
                    'label': 'cup',
                    'kind': 'object',
                    'class': 'Cup',
                    'source': 'detector',
                    'relations': [
                        {'predicate': 'rdf:type', 'object': 'Cup'},
                        {'predicate': 'dbp:color', 'object': 'blue'},
                    ],
                }
            ],
        }
    )

    assert grounded_context == {
        'entities': [
            {
                'id': 'cup_1',
                'label': 'cup',
                'kind': 'object',
                'class': 'Cup',
                'visible': True,
                'relations': [{'predicate': 'dbp:color', 'object': 'blue'}],
            }
        ],
    }


def test_build_plan_payload_omits_context_ref_from_new_envelopes() -> None:
    request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_42',
            'grounded_context': {
                'scene_summary': {
                    'observer': 'myself',
                    'backend': 'emorobcare_cv',
                },
                'state_t0': {
                    'captured_at_sec': 1777040000.0,
                },
            },
        }
    )
    payload = build_plan_payload(
        request=request,
        steps=[],
    )
    assert 'context_ref' not in payload['plan']
    assert 'grounded_context' not in payload


def test_project_llm_grounded_context_prioritizes_and_bounds_relations() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'cup_1', 'predicate': 'rdf:type', 'object': 'dbr:Cup'},
            {'entity': 'cup_1', 'predicate': 'rdf:type', 'object': 'Tableware'},
            {'entity': 'cup_1', 'predicate': 'dbp:name', 'object': 'blue mug'},
            {'entity': 'cup_1', 'predicate': 'dbp:color', 'object': 'blue'},
            {'entity': 'cup_1', 'predicate': 'oro:isAt', 'object': 'table_1'},
            {'entity': 'cup_1', 'predicate': 'oro:isOn', 'object': 'coaster_1'},
            {'entity': 'cup_1', 'predicate': 'oro:contains', 'object': 'water'},
            {'entity': 'cup_1', 'predicate': 'foaf:knows', 'object': 'person_1'},
            {'entity': 'cup_1', 'predicate': 'irrelevantPredicate', 'object': 'noise'},
        ],
    )

    relations = projected['entities'][0]['relations']
    assert relations == [
        {'predicate': 'rdf:type', 'object': 'Tableware'},
        {'predicate': 'dbp:name', 'object': 'blue mug'},
        {'predicate': 'dbp:color', 'object': 'blue'},
        {'predicate': 'oro:isAt', 'object': 'table_1'},
        {'predicate': 'oro:isOn', 'object': 'coaster_1'},
        {'predicate': 'oro:contains', 'object': 'water'},
    ]


def test_project_llm_grounded_context_can_include_raw_relations_for_skill_payloads() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'cup_1', 'predicate': 'rdf:type', 'object': 'dbr:Cup'},
            {'entity': 'cup_1', 'predicate': 'custom:fragile', 'object': 'true'},
        ],
        include_raw_relations=True,
    )

    entity = projected['entities'][0]
    assert 'relations' not in entity
    assert entity['raw_relations'] == [
        {'predicate': 'rdf:type', 'object': 'dbr:Cup'},
        {'predicate': 'custom:fragile', 'object': 'true'},
    ]


def test_execution_feedback_builder_keeps_plan_retry_budget_independent_of_step() -> None:
    payload = build_execution_feedback_payload(
        intent='raw_user_input',
        source='nao_orchestrator',
        plan_context={
            'goal_id': 'goal_2',
            'plan_id': 'plan_2',
            'plan_version': 5,
            'retry_budget': 0,
        },
        status='failed',
        event_type='step_failed',
        step={
            'id': 'step_1',
            'type': 'skill',
            'name': 'find_object',
            'retry_budget': 3,
            'on_failure': 'replan',
        },
    )
    feedback = ExecutionFeedback.from_payload(payload)
    assert feedback.retry_budget == 0
    assert feedback.step_retry_budget == 3


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


def test_normalize_plan_steps_rejects_retry_failure_policy_alias() -> None:
    steps = normalize_plan_steps(
        [
            {
                'type': 'skill',
                'name': 'perform_motion',
                'args': {'object': 'stand'},
                'on_failure': 'retry',
            }
        ]
    )
    assert steps[0]['on_failure'] == 'fail'


def test_truncate_text_adds_ellipsis_when_needed() -> None:
    assert truncate_text('hello world', 5) == 'hell…'
