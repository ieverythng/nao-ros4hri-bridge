from planner_common.contracts import ExecutionFeedback
from planner_common.contracts import PlannerDialogueAct
from planner_common.contracts import PlannerRequest
from planner_common.contracts import SceneSummary
from planner_common.contracts import build_dialogue_act_payload
from planner_common.contracts import build_execution_feedback_payload
from planner_common.contracts import build_plan_payload
from planner_common.contracts import extract_json_object
from planner_common.contracts import is_explicit_knowledge_statement
from planner_common.contracts import normalize_grounded_context
from planner_common.contracts import normalize_plan_steps
from planner_common.contracts import optional_float_fields
from planner_common.contracts import project_llm_grounded_context
from planner_common.contracts import strip_live_result_report_summary_text
from planner_common.contracts import truncate_text
from planner_common.report_outcome import build_report_outcome
from planner_common.report_outcome import plan_semantic_errors


def test_explicit_knowledge_statement_rejects_function_style_syntax() -> None:
    assert is_explicit_knowledge_statement('red_cup rdf:type Cup')
    assert is_explicit_knowledge_statement('red_cup dbp:name "RED CUP"')
    assert not is_explicit_knowledge_statement('rdf:type(cup, dbp:RedCup)')
    assert not is_explicit_knowledge_statement('add one cup')


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


def test_planner_request_normalizes_authoritative_target_selection() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'turn_grouped',
            'goal_id': 'goal_grouped',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'work_table',
                'member_ids': ['cup_1', 'book_1', 'cup_1'],
                'recipient_id': 'person_1',
                'ordering': 'none',
                'report_policy': 'final',
                'ignored': 'value',
            },
        }
    )

    assert request.target_selection == {
        'selection_kind': 'location_members',
        'operation': 'deliver',
        'source_location_id': 'work_table',
        'member_ids': ['cup_1', 'book_1'],
        'recipient_id': 'person_1',
        'ordering': 'none',
        'report_policy': 'final',
    }


def test_planner_request_normalizes_explicit_visit_selection() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'turn_visit',
            'goal_id': 'goal_visit',
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['cup_1'],
                'ordering': 'sequential',
                'report_policy': 'per_target',
            },
        }
    )

    assert request.target_selection['selection_kind'] == 'explicit_members'
    assert request.target_selection['member_ids'] == ['cup_1']


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
    assert feedback.plan_outcome_summary == {}


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
    assert feedback.plan_outcome_summary == {}


def test_execution_feedback_plan_outcome_summary_round_trips() -> None:
    payload = build_execution_feedback_payload(
        intent='raw_user_input',
        source='nao_orchestrator',
        plan_context={'goal_id': 'g1', 'plan_id': 'p1', 'plan_version': 1},
        status='completed',
        plan_outcome_summary={
            'completed_targets': ['cup_1'],
            'failed_targets': [],
            'pending_targets': [],
            'last_successful_step_id': 'step_1',
            'terminal_step_id': '',
            'terminal_reason': 'completed',
            'all_required_steps_succeeded': True,
        },
    )

    feedback = ExecutionFeedback.from_payload(payload)

    assert feedback.plan_outcome_summary == {
        'completed_targets': ['cup_1'],
        'failed_targets': [],
        'pending_targets': [],
        'last_successful_step_id': 'step_1',
        'terminal_step_id': '',
        'terminal_reason': 'completed',
        'all_required_steps_succeeded': True,
    }


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


def test_build_plan_payload_preserves_explicit_empty_scene_target_scope() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_targetless_motion',
            'goal_id': 'goal_targetless_motion',
            'scene_targets': ['anonymous_person_visible'],
        }
    )

    payload = build_plan_payload(
        request=request,
        steps=[
            {
                'type': 'skill',
                'name': 'perform_motion',
                'args': {'object': 'sit'},
            }
        ],
        scene_targets=[],
    )

    assert payload['plan']['scene_targets'] == []


def test_build_plan_payload_preserves_authoritative_target_selection() -> None:
    request = PlannerRequest.from_payload(
        {
            'request_id': 'turn_delivery',
            'goal_id': 'goal_delivery',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'table_1',
                'member_ids': ['cup_1', 'book_1'],
                'recipient_id': 'person_1',
                'report_policy': 'final',
            },
        }
    )

    payload = build_plan_payload(
        request=request,
        steps=[
            {
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
            }
        ],
    )

    assert payload['plan']['target_selection'] == request.target_selection


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


def test_report_outcome_excludes_delivery_recipient_and_support_anchor() -> None:
    outcome = build_report_outcome(
        plan_steps=[
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {
                    'object_id': 'cup_1',
                    'recipient': 'person_1',
                    'source': 'work_table',
                },
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'navigate_to',
                'args': {'target': 'work_table'},
            },
        ],
        execution_results=[
            {
                'id': 'step_1',
                'name': 'bring_object',
                'status': 'succeeded',
                'result_payload': {'object_id': 'cup_1', 'recipient': 'person_1'},
            },
            {'id': 'step_2', 'name': 'navigate_to', 'status': 'succeeded'},
        ],
        plan_outcome_summary={'completed_targets': ['cup_1', 'person_1', 'work_table']},
        grounded_context={
            'entities': [
                {'id': 'cup_1', 'kind': 'object', 'class': 'Cup', 'label': 'cup'},
                {'id': 'person_1', 'kind': 'person', 'class': 'Human', 'label': 'ALEX'},
            ],
            'locations': [
                {'id': 'work_table', 'label': 'work table', 'role': 'support_group'},
            ],
        },
    )

    assert outcome['mode'] == 'delivery'
    assert [item['id'] for item in outcome['reportable_objects']] == ['cup_1']
    assert outcome['recipients'][0]['id'] == 'person_1'
    assert {'id': 'person_1', 'label': 'ALEX', 'reason': 'recipient'} in outcome[
        'excluded_targets'
    ]
    assert {'id': 'work_table', 'label': 'work table', 'reason': 'navigation_only'} in outcome[
        'excluded_targets'
    ]


def test_report_outcome_treats_location_destination_as_delivery_anchor() -> None:
    outcome = build_report_outcome(
        plan_steps=[
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {
                    'object_id': 'book_1',
                    'destination': 'kitchen',
                },
            },
        ],
        execution_results=[
            {
                'id': 'step_1',
                'name': 'bring_object',
                'status': 'succeeded',
                'result_payload': {'object_id': 'book_1', 'destination': 'kitchen'},
            },
        ],
        plan_outcome_summary={'completed_targets': ['book_1', 'kitchen']},
        grounded_context={
            'entities': [
                {'id': 'book_1', 'kind': 'object', 'class': 'Book', 'label': 'book'},
            ],
            'locations': [
                {'id': 'kitchen', 'label': 'kitchen', 'class': 'Room', 'role': 'location_group'},
            ],
        },
        scene_targets=['book_1', 'kitchen'],
    )

    assert outcome['mode'] == 'delivery'
    assert [item['id'] for item in outcome['reportable_objects']] == ['book_1']
    assert outcome['recipients'] == []
    assert {'id': 'kitchen', 'label': 'kitchen', 'role': 'location'} in outcome['anchors']
    assert {'id': 'kitchen', 'label': 'kitchen', 'reason': 'room'} in outcome[
        'excluded_targets'
    ]


def test_report_outcome_rejects_person_as_place_object_support() -> None:
    outcome = build_report_outcome(
        plan_steps=[
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'place_object',
                'args': {'object_id': 'book_1', 'destination': 'person_1'},
            }
        ],
        execution_results=[
            {
                'id': 'step_1',
                'name': 'place_object',
                'status': 'succeeded',
                'result_payload': {'object_id': 'book_1', 'destination': 'person_1'},
            }
        ],
        grounded_context={
            'entities': [
                {'id': 'book_1', 'kind': 'object', 'class': 'Book', 'label': 'book'},
                {'id': 'person_1', 'kind': 'person', 'class': 'Human', 'label': 'ALEX'},
            ]
        },
    )

    assert outcome['mode'] == 'failure'
    assert outcome['reportable_objects'] == []
    assert outcome['recipients'] == []
    assert outcome['failures'] == [
        {
            'step_id': 'step_1',
            'skill': 'place_object',
            'target': 'book_1',
            'reason': 'a person cannot be a placement support',
        }
    ]


def test_plan_semantics_require_selected_objects_to_reach_delivery_recipient() -> None:
    errors = plan_semantic_errors(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'place_object',
                'args': {'object_id': 'book_1', 'destination': 'work_table'},
            }
        ],
        grounded_context={
            'entities': [
                {'id': 'book_1', 'kind': 'object', 'class': 'Book'},
                {'id': 'person_1', 'kind': 'person', 'class': 'Human'},
            ]
        },
        target_selection={
            'selection_kind': 'explicit_members',
            'operation': 'deliver',
            'member_ids': ['book_1'],
            'recipient_id': 'person_1',
        },
    )

    assert errors == [
        "delivery plan does not hand off book_1 to recipient 'person_1'"
    ]


def test_plan_semantics_reject_arbitrary_object_as_delivery_destination() -> None:
    errors = plan_semantic_errors(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'book_1', 'recipient': 'table_1'},
            }
        ],
        grounded_context={
            'entities': [
                {'id': 'book_1', 'kind': 'object', 'class': 'Book'},
                {'id': 'book_2', 'kind': 'object', 'class': 'Book'},
            ],
        },
        target_selection={
            'selection_kind': 'explicit_members',
            'operation': 'deliver',
            'member_ids': ['book_1'],
            'recipient_id': 'book_2',
        },
    )

    assert (
        'delivery target_selection.recipient_id must identify a grounded person: book_2'
        in errors
    )


def test_plan_semantics_reject_grounded_container_as_delivery_recipient() -> None:
    errors = plan_semantic_errors(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'apple_1', 'recipient': 'house_1'},
            }
        ],
        grounded_context={
            'entities': [
                {'id': 'apple_1', 'kind': 'object', 'class': 'Apple'},
                {
                    'id': 'house_1',
                    'kind': 'object',
                    'class': 'Container',
                    'relations': [{'predicate': 'oro:contains', 'object': 'phone_1'}],
                },
            ],
        },
        target_selection={
            'selection_kind': 'explicit_members',
            'operation': 'deliver',
            'member_ids': ['apple_1'],
            'recipient_id': 'house_1',
        },
    )

    assert errors == [
        'delivery target_selection.recipient_id must identify a grounded person: house_1'
    ]


def test_plan_semantics_allow_navigation_to_any_grounded_entity() -> None:
    errors = plan_semantic_errors(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'navigate_to',
                'args': {'target': 'cup_1'},
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'navigate_to',
                'args': {'target': 'person_1'},
            },
            {
                'id': 'step_3',
                'type': 'skill',
                'name': 'navigate_to',
                'args': {'target': 'kitchen'},
            },
        ],
        grounded_context={
            'entities': [
                {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                {'id': 'person_1', 'kind': 'person', 'class': 'Human'},
            ],
            'locations': [
                {
                    'id': 'kitchen',
                    'label': 'kitchen',
                    'contains': [
                        {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                    ],
                },
            ],
        },
        target_selection={
            'selection_kind': 'explicit_members',
            'operation': 'visit',
            'member_ids': ['cup_1', 'person_1', 'kitchen'],
            'report_policy': 'none',
        },
    )

    assert errors == []


def test_report_outcome_does_not_promote_raw_completed_target_aliases() -> None:
    outcome = build_report_outcome(
        plan_steps=[
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
            },
        ],
        execution_results=[
            {
                'id': 'step_1',
                'name': 'bring_object',
                'status': 'succeeded',
                'result_payload': {'object_id': 'cup_1', 'recipient': 'person_1'},
            },
        ],
        plan_outcome_summary={'completed_targets': ['cup_1', 'ALEX']},
        grounded_context={
            'entities': [
                {'id': 'cup_1', 'kind': 'object', 'class': 'Cup', 'label': 'cup'},
                {'id': 'person_1', 'kind': 'person', 'class': 'Human', 'label': 'ALEX'},
            ],
        },
    )

    assert outcome['mode'] == 'delivery'
    assert [item['id'] for item in outcome['reportable_objects']] == ['cup_1']
    assert [item['id'] for item in outcome['recipients']] == ['person_1']
    assert all(item['id'] != 'ALEX' for item in outcome['reportable_objects'])


def test_final_report_outcome_marks_uncompleted_selected_members() -> None:
    outcome = build_report_outcome(
        plan_steps=[
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'bring_object',
                'args': {'object_id': 'book_1', 'recipient': 'person_1'},
            },
        ],
        execution_results=[
            {
                'id': 'step_1',
                'name': 'bring_object',
                'status': 'succeeded',
                'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
            }
        ],
        grounded_context={
            'entities': [
                {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                {'id': 'book_1', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                {'id': 'person_1', 'label': 'ALEX', 'kind': 'person', 'class': 'Human'},
            ]
        },
        target_selection={
            'operation': 'deliver',
            'member_ids': ['cup_1', 'book_1'],
            'recipient_id': 'person_1',
        },
        report_role='final',
    )

    assert outcome['mode'] == 'mixed'
    assert [item['id'] for item in outcome['reportable_objects']] == ['cup_1']
    assert outcome['failures'] == [
        {
            'step_id': '',
            'skill': 'bring_object',
            'target': 'book_1',
            'reason': 'selected target has no successful execution evidence',
        }
    ]


def test_strip_live_result_report_summary_text_covers_manipulation_skills() -> None:
    steps = [
        {
            'type': 'skill',
            'name': 'bring_object',
            'args': {'object_id': 'codex_probe_cup'},
        },
        {
            'type': 'skill',
            'name': 'report_result',
            'args': {'summary_text': 'I will report a generic completion.'},
        },
        {
            'type': 'skill',
            'name': 'say',
            'args': {'text': 'done'},
        },
        {
            'type': 'skill',
            'name': 'report_result',
            'args': {'summary_text': 'Keep this standalone report.'},
        },
    ]

    repaired = strip_live_result_report_summary_text(steps)

    assert repaired[1]['args'] == {}
    assert repaired[3]['args'] == {'summary_text': 'Keep this standalone report.'}


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

    assert projected['counts'] == {
        'entities': 3,
        'people': 1,
        'objects': 1,
        'locations': 1,
    }
    cup = next(item for item in projected['entities'] if item['id'] == 'cup_jrjic')
    person = next(
        item for item in projected['entities'] if item['id'] == 'anonymous_person_ehfbf'
    )
    assert cup['label'] == 'cup'
    assert cup['kind'] == 'object'
    assert {'predicate': 'oro:isOn', 'object': 'table_1'} in cup['relations']
    assert {'predicate': 'seenBy', 'object': 'myself'} not in cup['relations']
    assert projected['locations'] == [
        {
            'id': 'table_1',
            'label': 'table_1',
            'role': 'support_group',
            'member_count': 1,
            'object_count': 1,
            'person_count': 0,
            'contains': [
                {
                    'id': 'cup_jrjic',
                    'label': 'cup',
                    'kind': 'object',
                    'class': 'Tableware',
                    'relation': 'oro:isOn',
                }
            ],
        }
    ]
    assert person['label'] == 'anonymous_person_ehfbf'
    forbidden = {'schema_version', 'source', 'backend', 'center_x', 'center_y', 'score', 'last_seen_sec'}
    assert forbidden.isdisjoint(cup.keys())


def test_project_llm_grounded_context_prefers_semantic_names_without_collapsing_ids() -> None:
    projected = project_llm_grounded_context(
        {
            'scene_summary': {
                'objects': [
                    {
                        'entity_id': 'codex_gold_apple',
                        'label': 'codex_gold_apple',
                        'kb_class': 'Apple',
                    },
                    {
                        'entity_id': 'codex_gold_table',
                        'label': 'codex_gold_table',
                        'kb_class': 'Table',
                    },
                ],
                'people': [
                    {
                        'id': 'codex_gold_recipient',
                        'label': 'codex_gold_recipient',
                        'type': 'Human',
                    },
                ],
            },
        },
        knowledge_rows=[
            {'entity': 'codex_gold_apple', 'predicate': 'dbp:name', 'object': 'KAREN'},
            {'entity': 'codex_gold_table', 'predicate': 'dbp:name', 'object': 'table'},
            {'entity': 'codex_gold_recipient', 'predicate': 'dbp:name', 'object': 'ALEX'},
            {
                'entity': 'codex_gold_recipient',
                'predicate': 'oro:isIn',
                'object': 'codex_gold_handoff_area',
            },
            {
                'entity': 'codex_gold_handoff_area',
                'predicate': 'rdf:type',
                'object': 'Place',
            },
        ],
    )

    by_id = {item['id']: item for item in projected['entities']}
    assert by_id['codex_gold_apple']['label'] == 'KAREN'
    assert by_id['codex_gold_table']['label'] == 'table'
    assert by_id['codex_gold_recipient']['label'] == 'ALEX'
    assert len({by_id[item]['label'] for item in by_id if item.startswith('codex_gold_')}) == 4
    assert all(location['id'] != 'codex_gold_recipient' for location in projected.get('locations', []))


def test_project_llm_grounded_context_keeps_anonymous_person_tracker_id() -> None:
    projected = project_llm_grounded_context(
        {
            'scene_summary': {
                'people': [
                    {
                        'id': 'anonymous_person_fcdai',
                        'label': 'anonymous_person',
                        'type': 'Human',
                    }
                ]
            }
        }
    )

    assert projected['entities'][0]['label'] == 'anonymous_person_fcdai'


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


def test_project_llm_grounded_context_keeps_only_frame_qualified_metric_position() -> None:
    projected = project_llm_grounded_context(
        {
            'scene_summary': {
                'objects': [
                    {
                        'entity_id': 'cup_1',
                        'label': 'cup',
                        'kb_class': 'Cup',
                        'center_x': 320.0,
                        'center_y': 240.0,
                        'frame_id': 'base_link',
                        'position': {'x': 1.0, 'y': 0.25, 'z': 0.6},
                        'distance_m': 1.03,
                    },
                    {
                        'entity_id': 'book_1',
                        'label': 'book',
                        'kb_class': 'Book',
                        'position': {'x': 0.2, 'y': 0.1, 'z': 0.4},
                    },
                ]
            }
        }
    )

    cup = next(item for item in projected['entities'] if item['id'] == 'cup_1')
    book = next(item for item in projected['entities'] if item['id'] == 'book_1')
    assert cup['frame_id'] == 'base_link'
    assert cup['position'] == {'x': 1.0, 'y': 0.25, 'z': 0.6}
    assert cup['distance_m'] == 1.03
    assert 'center_x' not in cup
    assert 'position' not in book


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
        'counts': {
            'entities': 1,
            'people': 0,
            'objects': 1,
            'locations': 0,
        },
    }


def test_normalize_grounded_context_preserves_generated_person_handles() -> None:
    grounded_context = normalize_grounded_context(
        {
            'entities': [
                {
                    'id': 'anonymous_person_fcdai',
                    'label': 'anonymous_person',
                    'kind': 'person',
                    'class': 'Human',
                },
                {
                    'id': 'sim_person_pqbcq',
                    'label': 'sim_person',
                    'kind': 'person',
                    'class': 'Human',
                },
                {
                    'id': 'codex_kitchen',
                    'label': 'Kitchen',
                    'kind': 'object',
                    'class': 'Room',
                },
                {
                    'id': 'apple_rvath',
                    'label': 'apple',
                    'kind': 'object',
                    'class': 'Apple',
                    'relations': [{'predicate': 'oro:isAt', 'object': 'codex_kitchen'}],
                },
            ],
            'locations': [
                {
                    'id': 'codex_kitchen',
                    'label': 'Kitchen',
                    'class': 'Room',
                    'contains': [{'id': 'apple_rvath', 'relation': 'oro:isAt'}],
                }
            ],
        }
    )

    people = {
        item['id']: item['label']
        for item in grounded_context['entities']
        if item.get('kind') == 'person'
    }
    assert people == {
        'anonymous_person_fcdai': 'anonymous_person_fcdai',
        'sim_person_pqbcq': 'sim_person_pqbcq',
    }
    assert grounded_context['counts'] == {
        'entities': 4,
        'people': 2,
        'objects': 1,
        'locations': 1,
    }


def test_normalize_grounded_context_keeps_anonymous_person_kind_after_type_loss() -> None:
    grounded_context = normalize_grounded_context(
        {
            'entities': [
                {
                    'id': 'anonymous_person_edcca',
                    'label': 'anonymous_person',
                    'kind': 'object',
                    'class': 'cyc:SpatialThing-Localized',
                }
            ]
        }
    )

    assert grounded_context['entities'][0]['kind'] == 'person'
    assert grounded_context['entities'][0]['label'] == 'anonymous_person_edcca'


def test_normalize_grounded_context_keeps_locations_out_of_object_kind() -> None:
    grounded_context = normalize_grounded_context(
        {
            'entities': [
                {
                    'id': 'Kitchen',
                    'label': 'Kitchen',
                    'kind': 'object',
                    'class': 'cyc:SpatialThing-Localized',
                    'relations': [{'predicate': 'oro:contains', 'object': 'cup_1'}],
                },
                {
                    'id': 'cup_1',
                    'label': 'cup',
                    'kind': 'object',
                    'class': 'Cup',
                    'relations': [{'predicate': 'oro:isIn', 'object': 'Kitchen'}],
                },
                {
                    'id': 'Lab',
                    'label': 'Lab',
                    'kind': 'object',
                    'class': 'Room',
                },
                {
                    'id': 'work_table',
                    'label': 'work table',
                    'kind': 'object',
                    'class': 'Table',
                    'relations': [{'predicate': 'oro:contains', 'object': 'cup_1'}],
                },
                {
                    'id': 'lab_bench',
                    'label': 'bench',
                    'kind': 'object',
                    'class': 'Bench',
                },
            ]
        }
    )

    entities = {item['id']: item for item in grounded_context['entities']}
    assert entities['Kitchen']['kind'] == 'location'
    assert entities['cup_1']['kind'] == 'object'
    assert entities['Lab']['kind'] == 'location'
    assert entities['work_table']['kind'] == 'object'
    assert entities['lab_bench']['kind'] == 'object'
    assert grounded_context['counts'] == {
        'entities': 5,
        'people': 0,
        'objects': 3,
        'locations': 3,
    }
    work_table = next(
        item for item in grounded_context['locations'] if item['id'] == 'work_table'
    )
    assert work_table['role'] == 'support_group'
    assert [item['id'] for item in work_table['contains']] == ['cup_1']


def test_project_llm_grounded_context_derives_location_groups_from_kb_predicates() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'codex_kitchen', 'predicate': 'rdf:type', 'object': 'Room'},
            {'entity': 'codex_kitchen', 'predicate': 'dbp:name', 'object': 'kitchen'},
            {'entity': 'codex_kitchen_cup', 'predicate': 'rdf:type', 'object': 'Cup'},
            {'entity': 'codex_kitchen_cup', 'predicate': 'dbp:name', 'object': 'TITAS'},
            {'entity': 'codex_kitchen_cup', 'predicate': 'oro:isIn', 'object': 'codex_kitchen'},
            {'entity': 'codex_table', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'codex_table', 'predicate': 'oro:contains', 'object': 'codex_phone'},
            {'entity': 'codex_phone', 'predicate': 'rdf:type', 'object': 'Phone'},
        ],
    )

    cup = next(item for item in projected['entities'] if item['id'] == 'codex_kitchen_cup')
    assert {'predicate': 'oro:isIn', 'object': 'codex_kitchen'} in cup['relations']
    assert projected['locations'] == [
        {
            'id': 'codex_kitchen',
            'label': 'kitchen',
            'class': 'Room',
            'role': 'navigation_target',
            'member_count': 1,
            'object_count': 1,
            'person_count': 0,
            'contains': [
                {
                    'id': 'codex_kitchen_cup',
                    'label': 'TITAS',
                    'kind': 'object',
                    'class': 'Cup',
                    'relation': 'oro:isIn',
                }
            ],
        },
        {
            'id': 'codex_table',
            'label': 'codex_table',
            'class': 'Table',
            'role': 'support_group',
            'member_count': 1,
            'object_count': 1,
            'person_count': 0,
            'contains': [
                {
                    'id': 'codex_phone',
                    'label': 'codex_phone',
                    'kind': 'object',
                    'class': 'Phone',
                    'relation': 'oro:contains',
                }
            ],
        },
    ]


def test_project_llm_grounded_context_counts_container_location_from_spatial_relation() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'sos_house', 'predicate': 'rdf:type', 'object': 'Container'},
            {'entity': 'sos_house', 'predicate': 'dbp:name', 'object': 'house'},
            {'entity': 'sos_apple', 'predicate': 'rdf:type', 'object': 'Apple'},
            {'entity': 'sos_apple', 'predicate': 'dbp:name', 'object': 'apple'},
            {'entity': 'sos_apple', 'predicate': 'oro:isAt', 'object': 'sos_house'},
        ],
    )

    assert projected['counts'] == {
        'entities': 2,
        'people': 0,
        'objects': 1,
        'locations': 1,
    }
    assert projected['locations'] == [
        {
            'id': 'sos_house',
            'label': 'house',
            'class': 'Container',
            'role': 'navigation_target',
            'member_count': 1,
            'object_count': 1,
            'person_count': 0,
            'contains': [
                {
                    'id': 'sos_apple',
                    'label': 'apple',
                    'kind': 'object',
                    'class': 'Apple',
                    'relation': 'oro:isAt',
                }
            ],
        }
    ]


def test_project_llm_grounded_context_derives_location_groups_from_kb_alias_predicates() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'codex_lab_table_section', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'codex_lab_table_section', 'predicate': 'dbp:name', 'object': 'work_table'},
            {'entity': 'codex_lab_table_section', 'predicate': 'placeOf', 'object': 'codex_lab_book'},
            {'entity': 'codex_lab_cup', 'predicate': 'rdf:type', 'object': 'Cup'},
            {'entity': 'codex_lab_cup', 'predicate': 'dbp:name', 'object': 'red cup'},
            {'entity': 'codex_lab_cup', 'predicate': 'isContainedIn', 'object': 'codex_lab_table_section'},
            {'entity': 'codex_lab_phone', 'predicate': 'rdf:type', 'object': 'Phone'},
            {'entity': 'codex_lab_phone', 'predicate': 'dbp:name', 'object': 'phone'},
            {'entity': 'codex_lab_phone', 'predicate': 'isAt', 'object': 'codex_lab_table_section'},
            {'entity': 'codex_lab_book', 'predicate': 'rdf:type', 'object': 'Book'},
            {'entity': 'codex_lab_book', 'predicate': 'dbp:name', 'object': 'blue book'},
        ],
    )

    assert projected['locations'] == [
        {
            'id': 'codex_lab_table_section',
            'label': 'work_table',
            'class': 'Table',
            'role': 'support_group',
            'member_count': 3,
            'object_count': 3,
            'person_count': 0,
            'contains': [
                {
                    'id': 'codex_lab_book',
                    'label': 'blue book',
                    'kind': 'object',
                    'class': 'Book',
                    'relation': 'oro:contains',
                },
                {
                    'id': 'codex_lab_phone',
                    'label': 'phone',
                    'kind': 'object',
                    'class': 'Phone',
                    'relation': 'oro:isAt',
                },
                {
                    'id': 'codex_lab_cup',
                    'label': 'red cup',
                    'kind': 'object',
                    'class': 'Cup',
                    'relation': 'oro:isIn',
                },
            ],
        },
    ]


def test_project_llm_grounded_context_preserves_location_aliases_from_named_entities() -> None:
    projected = project_llm_grounded_context(
        {
            'entities': [
                {
                    'id': 'codex_lab_table_section',
                    'label': 'codex_lab_table_section',
                    'kind': 'object',
                    'class': 'Table',
                    'relations': [{'predicate': 'dbp:name', 'object': 'work_table'}],
                },
                {
                    'id': 'codex_lab_cup',
                    'label': 'cup',
                    'kind': 'object',
                    'class': 'Cup',
                    'relations': [
                        {'predicate': 'oro:isOn', 'object': 'codex_lab_table_section'},
                    ],
                },
            ],
            'locations': [
                {
                    'id': 'codex_lab_table_section',
                    'label': 'codex_lab_table_section',
                    'class': 'Table',
                    'contains': [{'id': 'codex_lab_cup', 'relation': 'oro:isOn'}],
                }
            ],
        }
    )

    assert projected['locations'][0]['aliases'] == ['work_table']


def test_location_groups_keep_user_objects_with_spatial_materialization_class() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'codex_lab_table_section', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'codex_lab_table_section', 'predicate': 'dbp:name', 'object': 'work_table'},
            {
                'entity': 'codex_lab_manual',
                'predicate': 'rdf:type',
                'object': 'cyc:SpatialThing-Localized',
            },
            {'entity': 'codex_lab_manual', 'predicate': 'rdf:type', 'object': 'Book'},
            {'entity': 'codex_lab_manual', 'predicate': 'dbp:name', 'object': 'LAB_MANUAL'},
            {'entity': 'codex_lab_manual', 'predicate': 'oro:isOn', 'object': 'codex_lab_table_section'},
            {
                'entity': 'codex_lab_phone',
                'predicate': 'rdf:type',
                'object': 'cyc:SpatialThing-Localized',
            },
            {'entity': 'codex_lab_phone', 'predicate': 'rdf:type', 'object': 'Phone'},
            {'entity': 'codex_lab_phone', 'predicate': 'dbp:name', 'object': 'LAB_PHONE'},
            {'entity': 'codex_lab_phone', 'predicate': 'oro:isOn', 'object': 'codex_lab_table_section'},
        ],
    )

    assert projected['locations'][0]['id'] == 'codex_lab_table_section'
    assert [item['id'] for item in projected['locations'][0]['contains']] == [
        'codex_lab_manual',
        'codex_lab_phone',
    ]
    assert projected['locations'][0]['object_count'] == 2


def test_project_llm_grounded_context_filters_meta_support_and_people_from_location_members() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'codex_lab_table', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'codex_lab_table', 'predicate': 'dbp:name', 'object': 'work table'},
            {'entity': 'codex_lab_cup', 'predicate': 'rdf:type', 'object': 'Cup'},
            {'entity': 'codex_lab_cup', 'predicate': 'rdf:type', 'object': 'cyc:SpatialThing-Localized'},
            {'entity': 'codex_lab_cup', 'predicate': 'dbp:name', 'object': 'red cup'},
            {'entity': 'codex_lab_cup', 'predicate': 'oro:isOn', 'object': 'codex_lab_table'},
            {'entity': 'codex_lab_surface', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'codex_lab_surface', 'predicate': 'oro:isOn', 'object': 'codex_lab_table'},
            {'entity': 'codex_lab_room', 'predicate': 'rdf:type', 'object': 'Room'},
            {'entity': 'codex_lab_room', 'predicate': 'oro:isAt', 'object': 'codex_lab_table'},
            {'entity': 'codex_spatial_marker', 'predicate': 'rdf:type', 'object': 'cyc:SpatialThing-Localized'},
            {'entity': 'codex_spatial_marker', 'predicate': 'oro:isAt', 'object': 'codex_lab_table'},
            {'entity': 'codex_alex', 'predicate': 'rdf:type', 'object': 'Human'},
            {'entity': 'codex_alex', 'predicate': 'oro:isAt', 'object': 'codex_lab_table'},
        ],
    )

    assert projected['locations'][0]['contains'] == [
        {
            'id': 'codex_lab_cup',
            'label': 'red cup',
            'kind': 'object',
            'class': 'Cup',
            'relation': 'oro:isOn',
        }
    ]
    assert projected['locations'][0]['member_count'] == 1
    assert projected['locations'][0]['role'] == 'support_group'


def test_project_llm_grounded_context_filters_inactive_detector_people() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {
                'entity': 'anonymous_person_old',
                'predicate': 'rdf:type',
                'object': 'Human',
            },
            {
                'entity': 'anonymous_person_old',
                'predicate': 'myself sees',
                'object': 'true',
            },
            {
                'entity': 'anonymous_person_current',
                'predicate': 'rdf:type',
                'object': 'Human',
            },
            {
                'entity': 'sim_person_stale',
                'predicate': 'rdf:type',
                'object': 'Human',
            },
            {
                'entity': 'sim_person_current',
                'predicate': 'rdf:type',
                'object': 'Human',
            },
            {
                'entity': 'fixture_person',
                'predicate': 'rdf:type',
                'object': 'Human',
            },
        ],
        active_person_ids={'anonymous_person_current', 'sim_person_current'},
    )

    assert {item['id'] for item in projected['entities']} == {
        'anonymous_person_current',
        'fixture_person',
        'sim_person_current',
    }
    assert projected['counts']['people'] == 3


def test_project_llm_grounded_context_keeps_anonymous_people_without_tracker_state() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {
                'entity': 'anonymous_person_fixture',
                'predicate': 'rdf:type',
                'object': 'Human',
            }
        ],
    )

    assert [item['id'] for item in projected['entities']] == [
        'anonymous_person_fixture'
    ]


def test_normalized_location_groups_never_promote_people_to_locations() -> None:
    normalized = normalize_grounded_context(
        {
            'entities': [
                {
                    'id': 'codex_recipient_person',
                    'label': 'ALEX',
                    'kind': 'person',
                    'class': 'Human',
                    'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                },
                {
                    'id': 'codex_kitchen_cup',
                    'label': 'cup',
                    'kind': 'object',
                    'class': 'Cup',
                    'relations': [{'predicate': 'oro:isIn', 'object': 'codex_recipient_person'}],
                },
            ],
            'locations': [
                {
                    'id': 'codex_recipient_person',
                    'label': 'ALEX',
                    'class': 'Human',
                    'contains': [
                        {
                            'id': 'codex_kitchen_cup',
                            'label': 'cup',
                            'kind': 'object',
                            'class': 'Cup',
                        }
                    ],
                }
            ],
        }
    )

    assert 'locations' not in normalized


def test_location_groups_never_promote_objects_or_people_from_contains_relations() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'anonymous_person_dcdbc', 'predicate': 'rdf:type', 'object': 'Human'},
            {'entity': 'book_nwred', 'predicate': 'rdf:type', 'object': 'Book'},
            {'entity': 'book_nwred', 'predicate': 'oro:contains', 'object': 'anonymous_person_dcdbc'},
            {'entity': 'cup_ohjps', 'predicate': 'rdf:type', 'object': 'Tableware'},
            {'entity': 'cup_ohjps', 'predicate': 'oro:contains', 'object': 'anonymous_person_dcdbc'},
            {'entity': 'cup_table', 'predicate': 'rdf:type', 'object': 'Table'},
            {'entity': 'cup_table', 'predicate': 'oro:contains', 'object': 'cup_ohjps'},
            {'entity': 'book_nwred', 'predicate': 'oro:isAt', 'object': 'anonymous_person_dcdbc'},
        ],
    )

    assert [item['id'] for item in projected['locations']] == ['cup_table']
    assert projected['locations'][0]['class'] == 'Table'
    assert projected['locations'][0]['role'] == 'support_group'
    assert projected['locations'][0]['member_count'] == 1
    assert projected['locations'][0]['contains'][0]['id'] == 'cup_ohjps'
    assert projected['locations'][0]['contains'][0]['relation'] == 'oro:contains'


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
            {'entity': 'cup_1', 'predicate': 'oro:isIn', 'object': 'kitchen_1'},
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
        {'predicate': 'oro:isIn', 'object': 'kitchen_1'},
    ]


def test_project_llm_grounded_context_bounds_location_relations_after_membership() -> None:
    projected = project_llm_grounded_context(
        {'scene_summary': {}},
        knowledge_rows=[
            {'entity': 'cup_1', 'predicate': 'rdf:type', 'object': 'dbr:Cup'},
            {'entity': 'cup_1', 'predicate': 'rdf:type', 'object': 'Tableware'},
            {'entity': 'cup_1', 'predicate': 'dbp:name', 'object': 'blue mug'},
            {'entity': 'cup_1', 'predicate': 'dbp:color', 'object': 'blue'},
            {'entity': 'cup_1', 'predicate': 'oro:isAt', 'object': 'table_1'},
            {'entity': 'cup_1', 'predicate': 'oro:isOn', 'object': 'coaster_1'},
            {'entity': 'cup_1', 'predicate': 'oro:isIn', 'object': 'kitchen_1'},
            {'entity': 'cup_1', 'predicate': 'oro:contains', 'object': 'water'},
        ],
    )

    relations = projected['entities'][0]['relations']
    assert {'predicate': 'oro:isIn', 'object': 'kitchen_1'} in relations
    assert {'predicate': 'oro:contains', 'object': 'water'} not in relations


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


def test_normalize_plan_steps_canonicalizes_place_object_aliases() -> None:
    steps = normalize_plan_steps(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'place_object',
                'args': {'object': 'apple_1', 'target': 'table_1'},
            }
        ]
    )

    assert steps[0]['args'] == {
        'target': 'apple_1',
        'destination': 'table_1',
    }


def test_normalize_plan_steps_resolves_place_destination_from_prior_pick() -> None:
    steps = normalize_plan_steps(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'pick_object',
                'args': {'target': 'apple_1'},
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'place_object',
                'args': {'target': 'table_1'},
                'requires': ['step_1'],
            },
        ]
    )

    assert steps[1]['args'] == {
        'target': 'apple_1',
        'destination': 'table_1',
    }


def test_normalize_plan_steps_does_not_invent_place_object_without_prior_pick() -> None:
    steps = normalize_plan_steps(
        [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'place_object',
                'args': {'target': 'table_1'},
            }
        ]
    )

    assert steps[0]['args'] == {'target': 'table_1'}


def test_truncate_text_adds_ellipsis_when_needed() -> None:
    assert truncate_text('hello world', 5) == 'hell…'


def test_optional_float_fields_keeps_only_numeric_values() -> None:
    assert optional_float_fields(
        {
            'center_x': '12.5',
            'center_y': None,
            'distance_m': 'not-a-number',
            'confidence': 0.9,
        },
        ('center_x', 'center_y', 'distance_m', 'confidence'),
    ) == {
        'center_x': 12.5,
        'confidence': 0.9,
    }
