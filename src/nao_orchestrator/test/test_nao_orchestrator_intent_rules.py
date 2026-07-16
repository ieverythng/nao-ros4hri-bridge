import json

from kb_skills.intent_labels import KB_QUERY_VISIBLE_PEOPLE

from nao_orchestrator.intent_rules import classify_motion_target
from nao_orchestrator.intent_rules import build_scan_result_payload
from nao_orchestrator.intent_rules import Intent
from nao_orchestrator.intent_rules import make_intent_signature
from nao_orchestrator.intent_rules import normalize_incoming_intent
from nao_orchestrator.intent_rules import normalize_legacy_intent
from nao_orchestrator.intent_rules import parse_plan_envelope
from nao_orchestrator.intent_rules import parse_execution_plan
from nao_orchestrator.intent_rules import parse_intent_data
from nao_orchestrator.intent_rules import posture_topic_fallback_for_motion
from nao_orchestrator.intent_rules import resolve_ack_text
from nao_orchestrator.intent_rules import resolve_say_text
from nao_orchestrator.intent_rules import resolve_scan_result
from nao_orchestrator.intent_rules import scan_step_should_auto_report
from nao_orchestrator.intent_rules import is_people_scan_target
from nao_orchestrator.intent_rules import is_unresolved_report_template
from nao_orchestrator.intent_rules import summarize_people_detection
from nao_orchestrator.intent_rules import validate_execution_plan
from nao_orchestrator.orchestrator import _ExecutionReportResult
from nao_orchestrator.orchestrator import _binding_value
from nao_orchestrator.orchestrator import _dedupe_statements
from nao_orchestrator.orchestrator import _execution_report_dialogue_context
from nao_orchestrator.orchestrator import _motion_result_payload
from nao_orchestrator.orchestrator import _normalize_execution_mode
from nao_orchestrator.orchestrator import _plan_outcome_summary
from nao_orchestrator.orchestrator import _report_text_from_result_payload
from nao_orchestrator.orchestrator import _single_entity_statement
from nao_orchestrator.orchestrator import _statement_from_binding
from nao_orchestrator.orchestrator import _statement_parts
from nao_orchestrator.orchestrator import NaoOrchestrator


def test_parse_intent_data_returns_dict_for_valid_json() -> None:
    payload = parse_intent_data('{"object":"stand"}')
    assert payload == {'object': 'stand'}


def test_normalize_legacy_intent_maps_posture_to_perform_motion() -> None:
    intent_name, data = normalize_legacy_intent('posture_stand', 'Hello there!')
    assert intent_name == Intent.PERFORM_MOTION
    assert data['object'] == 'stand'


def test_kb_statement_helpers_parse_concrete_statements() -> None:
    assert _statement_parts('codex_marker dbp:color blue') == (
        'codex_marker',
        'dbp:color',
        'blue',
    )
    assert _statement_parts('codex_marker') == ('', '', '')
    assert _single_entity_statement('codex_marker') == 'codex_marker'
    assert _single_entity_statement('red cup') == ''


def test_kb_statement_helpers_build_removal_statements_from_bindings() -> None:
    row = {'?predicate': 'dbp:color', '?object': 'green'}

    assert _binding_value(row, 'predicate') == 'dbp:color'
    assert _binding_value(row, 'object') == 'green'
    assert (
        _statement_from_binding('codex_marker', _binding_value(row, 'predicate'), row)
        == 'codex_marker dbp:color green'
    )
    assert _dedupe_statements(['a b c', '', 'a b c', 'a b d']) == [
        'a b c',
        'a b d',
    ]


def test_normalize_legacy_intent_accepts_json_payload() -> None:
    raw = json.dumps(
        {
            'intent': '__intent_say__',
            'object': 'Testing migrated say dispatch.',
            'recipient': 'person_1',
        }
    )
    intent_name, data = normalize_legacy_intent(raw, 'Hello there!')
    assert intent_name == Intent.SAY
    assert data == {
        'object': 'Testing migrated say dispatch.',
        'recipient': 'person_1',
    }


def test_normalize_incoming_intent_maps_help_to_say() -> None:
    intent_name, data = normalize_incoming_intent('help', {}, 'Hello there!')
    assert intent_name == Intent.SAY
    assert 'stand, sit, kneel' in data['object']


def test_normalize_incoming_intent_preserves_custom_kb_query_labels() -> None:
    intent_name, data = normalize_incoming_intent(
        KB_QUERY_VISIBLE_PEOPLE,
        {'goal': 'visible_people'},
        'Hello there!',
    )
    assert intent_name == KB_QUERY_VISIBLE_PEOPLE
    assert data == {'goal': 'visible_people'}


def test_resolve_say_text_prefers_suggested_response_for_greet() -> None:
    text = resolve_say_text(
        Intent.GREET,
        {'suggested_response': 'Hello from the migrated orchestrator!'},
        'Default hello',
    )
    assert text == 'Hello from the migrated orchestrator!'


def test_say_step_text_helpers_accept_common_llm_fields() -> None:
    text = resolve_say_text(
        Intent.SAY,
        {'object': '', 'suggested_response': 'I finished that sequence.'},
        '',
    )
    assert text == 'I finished that sequence.'


def test_classify_motion_target_maps_head_motion() -> None:
    route, payload = classify_motion_target(
        Intent.PERFORM_MOTION,
        {'object': 'head_look_left'},
    )
    assert route == 'head_motion'
    assert payload['yaw'] == 0.45


def test_normalize_execution_mode_defaults_to_fake() -> None:
    assert _normalize_execution_mode('fake') == 'fake'
    assert _normalize_execution_mode('real') == 'real'
    assert _normalize_execution_mode('unexpected') == 'fake'


def test_classify_motion_target_maps_look_at_reset_alias() -> None:
    route, payload = classify_motion_target(
        Intent.PERFORM_MOTION,
        {'object': 'look_at_reset'},
    )
    assert route == 'look_at_reset'
    assert payload['policy'] == 'reset'


def test_resolve_ack_text_prefers_explicit_ack_text() -> None:
    text = resolve_ack_text(
        Intent.PERFORM_MOTION,
        {'ack_text': 'Sure, I am standing up now.'},
        'Default hello',
    )
    assert text == 'Sure, I am standing up now.'


def test_parse_execution_plan_filters_unknown_steps() -> None:
    plan = parse_execution_plan(
        {
            'plan': [
                {'type': 'skill', 'name': 'perform_motion', 'args': {'object': 'stand'}},
                {'type': 'mystery', 'name': 'ignored', 'args': {}},
            ]
        }
    )
    assert plan == [
        {
            'id': 'step_1',
            'type': 'skill',
            'name': 'perform_motion',
            'args': {'object': 'stand'},
            'requires': [],
            'on_failure': 'fail',
            'retry_budget': 0,
        }
    ]


def test_parse_plan_envelope_accepts_dict_style_plan_metadata() -> None:
    envelope = parse_plan_envelope(
        {
            'plan': {
                'goal_id': 'goal-7',
                'plan_id': 'plan-42',
                'plan_version': 3,
                'context_ref': {'observer': 'myself'},
                'status': 'executing',
                'validation_status': 'draft',
                'scene_targets': ['person'],
                'communication_policy': {'emit_acknowledge': False},
                'steps': [
                    {'type': 'say', 'args': {'text': 'hello'}},
                ],
            },
            'scene_targets': ['cup'],
        }
    )
    assert envelope['goal_id'] == 'goal-7'
    assert envelope['plan_id'] == 'plan-42'
    assert envelope['plan_version'] == 3
    assert envelope['context_ref'] == {'observer': 'myself'}
    assert envelope['status'] == 'executing'
    assert envelope['validation_status'] == 'draft'
    assert envelope['scene_targets'] == ['person']
    assert envelope['communication_policy']['emit_acknowledge'] is False
    assert envelope['steps'][0]['id'] == 'step_1'


def test_scan_auto_report_only_when_no_later_speech_step() -> None:
    plan = [
        {'type': 'skill', 'name': 'scan', 'args': {}},
        {'type': 'skill', 'name': 'report_result', 'args': {'summary_text': 'done'}},
    ]

    assert scan_step_should_auto_report(plan=plan, step_index=0) is False
    assert scan_step_should_auto_report(plan=plan, step_index=1) is True


def test_validate_execution_plan_marks_explicit_empty_plan_invalid() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {'plan': []},
    )
    assert envelope['steps'] == []
    assert envelope['errors'] == ['plan contains no valid executable steps']


def test_validate_execution_plan_rejects_wrong_kind_and_extra_visit_targets() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'person_1', 'kind': 'person', 'class': 'Human'},
                ],
                'locations': [{'id': 'kitchen', 'kind': 'location', 'class': 'Room'}],
            },
            'plan': {
                'target_selection': {
                    'selection_kind': 'explicit_members',
                    'operation': 'visit',
                    'member_ids': ['cup_1'],
                    'ordering': 'sequential',
                    'report_policy': 'per_target',
                },
                'steps': [
                    {'id': 'step_1', 'type': 'skill', 'name': 'navigate_to', 'args': {'target': 'cup_1'}},
                    {'id': 'step_2', 'type': 'skill', 'name': 'report_result', 'args': {}, 'requires': ['step_1']},
                    {'id': 'step_3', 'type': 'skill', 'name': 'navigate_to', 'args': {'target': 'person_1'}, 'requires': ['step_2']},
                    {'id': 'step_4', 'type': 'skill', 'name': 'navigate_to', 'args': {'target': 'kitchen'}, 'requires': ['step_3']},
                ],
            },
        },
    )

    assert envelope['steps'] == []
    assert any('visit targets' in error for error in envelope['errors'])


def test_validate_execution_plan_rejects_delivery_missing_selected_member() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'book_1', 'kind': 'object', 'class': 'Book'},
                    {'id': 'person_1', 'kind': 'person', 'class': 'Human'},
                ]
            },
            'plan': {
                'target_selection': {
                    'selection_kind': 'location_members',
                    'operation': 'deliver',
                    'source_location_id': 'table_1',
                    'member_ids': ['cup_1', 'book_1'],
                    'recipient_id': 'person_1',
                    'report_policy': 'final',
                },
                'steps': [
                    {'id': 'step_1', 'type': 'skill', 'name': 'bring_object', 'args': {'object_id': 'cup_1', 'recipient': 'person_1'}},
                    {'id': 'step_2', 'type': 'skill', 'name': 'report_result', 'args': {}, 'requires': ['step_1']},
                ],
            },
        },
    )

    assert envelope['steps'] == []
    assert any('book_1' in error for error in envelope['errors'])


def test_validate_execution_plan_rejects_invalid_look_at_step() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {'type': 'look_at', 'args': {}},
            ]
        },
    )
    assert envelope['steps'] == []
    assert envelope['errors'] == [
        'step_1: look_at step is missing target_frame or supported policy'
    ]


def test_validate_execution_plan_accepts_targetless_look_at_policy() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {'type': 'look_at', 'args': {'policy': 'social'}},
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['args'] == {'policy': 'social'}


def test_validate_execution_plan_accepts_look_at_target_alias() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {'type': 'look_at', 'args': {'target': 'anonymous person bcbhb'}},
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['args']['target_frame'] == 'anonymous person bcbhb'


def test_validate_execution_plan_maps_look_at_head_center_to_reset() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {'type': 'look_at', 'args': {'target': 'head_center'}},
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['args']['policy'] == 'reset'


def test_validate_execution_plan_accepts_clarify_failure_policy() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'stand'},
                    'on_failure': 'clarify',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['on_failure'] == 'clarify'


def test_validate_execution_plan_accepts_continue_failure_policy() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'sit'},
                    'on_failure': 'continue',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['on_failure'] == 'continue'


def test_validate_execution_plan_rejects_retry_failure_policy_alias() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'stand'},
                    'on_failure': 'retry',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['on_failure'] == 'fail'


def test_validate_execution_plan_accepts_scan_skill() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'scan',
                    'args': {'target': 'people', 'max_sweeps': 2},
                    'on_failure': 'replan',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['name'] == 'scan'


def test_validate_execution_plan_accepts_fake_navigation_skill() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'navigate_to',
                    'args': {'target': 'kitchen', 'result_mode': 'path_blocked'},
                    'on_failure': 'replan',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['name'] == 'navigate_to'


def test_validate_execution_plan_accepts_report_result_skill() -> None:
    envelope = validate_execution_plan(
        Intent.SAY,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'report_result',
                    'args': {'summary_text': 'I found one person in front of me.'},
                    'on_failure': 'fail',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['name'] == 'report_result'


def test_validate_execution_plan_accepts_report_result_reusing_prior_context() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'scan',
                    'args': {},
                },
                {
                    'type': 'skill',
                    'name': 'report_result',
                    'args': {},
                    'requires': ['step_1'],
                },
            ]
        },
    )

    assert envelope['errors'] == []
    assert envelope['steps'][1]['name'] == 'report_result'
    assert envelope['steps'][1]['args'] == {}


def test_unresolved_report_template_detects_evidence_placeholders() -> None:
    assert is_unresolved_report_template(
        'I found the following: [evidence.objects], [evidence.people].'
    )
    assert is_unresolved_report_template('Report {result.summary_text}.')
    assert not is_unresolved_report_template('I found one person near the table.')


def test_validate_execution_plan_accepts_ask_user_skill_and_defaults_failure_policy() -> None:
    envelope = validate_execution_plan(
        Intent.SAY,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'ask_user',
                    'args': {'question': 'Should I scan again now?'},
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['name'] == 'ask_user'
    assert envelope['steps'][0]['on_failure'] == 'ask_user'


def test_validate_execution_plan_rejects_ask_user_without_prompt_or_slots() -> None:
    envelope = validate_execution_plan(
        Intent.SAY,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'ask_user',
                    'args': {},
                }
            ]
        },
    )
    assert envelope['steps'] == []
    assert envelope['errors'] == [
        'step_1: ask_user step is missing prompt text or slots_needed'
    ]


def test_validate_execution_plan_accepts_wave_greet_fake_skill() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'type': 'skill',
                    'name': 'wave',
                    'args': {'style': 'friendly'},
                    'on_failure': 'continue',
                }
            ]
        },
    )
    assert envelope['errors'] == []
    assert envelope['steps'][0]['name'] == 'wave'


def test_validate_execution_plan_accepts_manipulation_fake_skills() -> None:
    envelope = validate_execution_plan(
        Intent.PRESENT_CONTENT,
        {
            'plan': [
                {'type': 'skill', 'name': 'pick', 'args': {'target': 'cup_1'}},
                {
                    'type': 'skill',
                    'name': 'place',
                    'args': {'target': 'cup_1', 'destination': 'shelf_1'},
                },
                {
                    'type': 'skill',
                    'name': 'bring',
                    'args': {'target': 'book_1', 'recipient': 'person_1'},
                },
            ]
        },
    )

    assert envelope['errors'] == []
    assert [step['name'] for step in envelope['steps']] == ['pick', 'place', 'bring']


def test_scan_step_is_available_without_demo_gate() -> None:
    success, reason, metadata = resolve_scan_result(
        {'target_kind': 'scene'},
        default_result_mode='success',
        default_summary='I scanned the scene.',
    )

    assert success
    assert reason == 'I scanned the scene.'
    assert metadata == {
        'target': '',
        'target_kind': 'scene',
        'result_mode': 'success',
    }


def test_targeted_scan_without_explicit_summary_reports_missing_detection() -> None:
    success, reason, metadata = resolve_scan_result(
        {'target': 'people', 'target_kind': 'people'},
        default_result_mode='success',
        default_summary='I scanned the scene.',
    )

    assert success
    assert (
        reason
        == 'I completed the scan for people, but no confirmed detection result was reported.'
    )
    assert metadata['target_kind'] == 'people'


def test_targeted_scan_preserves_explicit_summary() -> None:
    success, reason, metadata = resolve_scan_result(
        {
            'target': 'people',
            'target_kind': 'people',
            'summary': 'I found one person near the table.',
        },
        default_result_mode='success',
        default_summary='I scanned the scene.',
    )

    assert success
    assert reason == 'I found one person near the table.'
    assert metadata['target'] == 'people'


def test_scan_step_can_report_configured_failure() -> None:
    success, reason, metadata = resolve_scan_result(
        {'target_kind': 'people'},
        default_result_mode='failure',
    )

    assert success is False
    assert reason == 'scan requested failure for people'
    assert metadata['result_mode'] == 'failure'


def test_people_scan_payload_marks_objects_as_non_target_when_no_people() -> None:
    payload = build_scan_result_payload(
        {
            'target': 'people',
            'target_kind': 'people',
            'objects': [
                {'id': 'obj_1', 'label': 'bottle', 'source': 'scene_summary'},
            ],
        }
    )

    assert payload['skill'] == 'scan'
    assert payload['target_found'] is False
    assert payload['people'] == []
    assert payload['objects'][0]['label'] == 'bottle'
    assert 'none were confirmed as people' in payload['summary_text'].lower()


def test_scene_scan_payload_summarizes_objects() -> None:
    payload = build_scan_result_payload(
        {
            'target_kind': 'scene',
            'objects': [
                {'id': 'cup_1', 'label': 'cup', 'source': 'scene_summary'},
                {'id': 'chair_1', 'label': 'chair', 'source': 'scene_summary'},
            ],
        }
    )

    assert payload['target_found'] is True
    assert payload['summary_text'].startswith('I completed the scene scan')


def test_report_text_from_summaryless_scan_payload_summarizes_objects() -> None:
    report_text = _report_text_from_result_payload(
        {
            'skill': 'scan',
            'target_kind': 'scene',
            'summary_text': '',
            'objects': [
                {'id': 'blueberry_1', 'label': 'blueberry', 'source': 'scene_summary'},
                {'id': 'blueberry_2', 'label': 'blueberry', 'source': 'scene_summary'},
            ],
        }
    )

    assert report_text.startswith('I completed the scene scan')
    assert 'blueberry' in report_text


def test_report_text_from_completed_target_payload_uses_conservative_completion() -> None:
    report_text = _report_text_from_result_payload(
        {
            'skill': 'navigate_to',
            'status': 'succeeded',
            'target': 'cup',
            'summary_text': '',
        }
    )

    assert report_text == 'I completed the task for cup.'


def test_report_result_text_uses_chatbot_context_for_step_chain() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    captured = {}

    def request_report(context):
        captured.update(context)
        return _ExecutionReportResult(
            text='I navigated to the cup and found two blueberries.',
            source='chatbot',
        )

    orchestrator._request_execution_report_text = request_report
    orchestrator.get_logger = lambda: type(
        'Logger',
        (),
        {'info': lambda *_args, **_kwargs: None},
    )()

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'goal_text': 'navigate to the cup and report other objects',
            'normalized_intents': ['navigate_to', 'inspect_scene', 'report_result'],
            'dialogue_context': [
                'user:Navigate to the cup and tell me what else you see.',
                'assistant:Sure, I will navigate to the cup and look around.',
            ],
            'plan_context': {
                'plan_id': 'plan_1',
                'plan_version': 2,
                'scene_targets': ['cup'],
            },
            'execution_results': [
                {
                    'id': 'step_1',
                    'name': 'navigate_to',
                    'type': 'skill',
                    'status': 'succeeded',
                    'result_summary': 'I navigated to the cup.',
                    'result_payload': {
                        'skill': 'navigate_to',
                        'status': 'succeeded',
                        'target': 'cup',
                    },
                },
                {
                    'id': 'step_2',
                    'name': 'scan',
                    'type': 'skill',
                    'status': 'succeeded',
                    'result_summary': 'I found two blueberries.',
                    'result_payload': {
                        'skill': 'scan',
                        'objects': [{'label': 'blueberry'}, {'label': 'blueberry'}],
                    },
                },
            ],
        },
    )

    assert report_text == 'I navigated to the cup and found two blueberries.'
    assert captured['goal_text'] == 'navigate to the cup and report other objects'
    assert captured['scene_targets'] == ['cup']
    assert captured['dialogue_context'] == [
        'user:Navigate to the cup and tell me what else you see.'
    ]
    assert [step['name'] for step in captured['steps']] == ['navigate_to', 'scan']


def test_execution_report_context_marks_intermediate_report_result() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)

    context = orchestrator._execution_report_context(
        {
            'current_step_index': 1,
            'plan_steps': [
                {'id': 'step_1', 'name': 'navigate_to', 'args': {'target': 'apple'}},
                {'id': 'step_2', 'name': 'report_result', 'args': {}},
                {'id': 'step_3', 'name': 'navigate_to', 'args': {'target': 'book'}},
                {'id': 'step_4', 'name': 'report_result', 'args': {}},
            ],
            'execution_results': [
                {
                    'name': 'navigate_to',
                    'status': 'succeeded',
                    'result_summary': 'I navigated to the apple.',
                }
            ],
            'last_result_summary': 'I navigated to the apple.',
        }
    )

    assert context['report_role'] == 'intermediate'
    assert context['latest_result_summary'] == 'I navigated to the apple.'
    assert [step['name'] for step in context['future_steps']] == [
        'navigate_to',
        'report_result',
    ]


def test_execution_report_context_marks_terminal_report_result() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)

    context = orchestrator._execution_report_context(
        {
            'current_step_index': 3,
            'plan_steps': [
                {'id': 'step_1', 'name': 'navigate_to', 'args': {'target': 'apple'}},
                {'id': 'step_2', 'name': 'report_result', 'args': {}},
                {'id': 'step_3', 'name': 'navigate_to', 'args': {'target': 'book'}},
                {'id': 'step_4', 'name': 'report_result', 'args': {}},
            ],
            'execution_results': [
                {
                    'name': 'navigate_to',
                    'status': 'succeeded',
                    'result_summary': 'I navigated to the book.',
                }
            ],
        }
    )

    assert context['report_role'] == 'final'
    assert context['future_steps'] == []


def test_execution_report_context_includes_plan_outcome_summary() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)

    context = orchestrator._execution_report_context(
        {
            'plan_outcome_summary': {
                'completed_targets': ['cup', 'book'],
                'pending_targets': ['phone'],
                'terminal_reason': 'phone was unavailable',
                'all_required_steps_succeeded': False,
            },
        }
    )

    assert context['plan_outcome_summary'] == {
        'terminal_reason': 'phone was unavailable',
        'all_required_steps_succeeded': False,
    }
    assert 'completed_targets' not in context['plan_outcome_summary']
    assert 'pending_targets' not in context['plan_outcome_summary']


def test_execution_report_context_keeps_report_outcome_as_target_evidence() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)

    context = orchestrator._execution_report_context(
        {
            'plan_steps': [
                {
                    'id': 'step_1',
                    'type': 'skill',
                    'name': 'navigate_to',
                    'args': {'target': 'person_1'},
                },
                {
                    'id': 'step_2',
                    'type': 'skill',
                    'name': 'bring_object',
                    'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
                },
            ],
            'execution_results': [
                {
                    'id': 'step_1',
                    'name': 'navigate_to',
                    'status': 'succeeded',
                    'args': {'target': 'person_1'},
                },
                {
                    'id': 'step_2',
                    'name': 'bring_object',
                    'status': 'succeeded',
                    'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
                },
            ],
            'plan_outcome_summary': {
                'completed_targets': ['person_1', 'cup_1'],
                'all_required_steps_succeeded': True,
            },
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'person_1', 'label': 'ALEX', 'kind': 'person', 'class': 'Human'},
                ]
            },
        }
    )

    assert context['plan_outcome_summary'] == {'all_required_steps_succeeded': True}
    assert context['report_outcome']['reportable_objects'] == [
        {
            'id': 'cup_1',
            'label': 'cup',
            'class': 'Cup',
            'kind': 'object',
            'status': 'completed',
        }
    ]
    assert context['report_outcome']['recipients'] == [
        {'id': 'person_1', 'label': 'ALEX', 'class': 'Human', 'kind': 'person'}
    ]


def test_execution_report_context_keeps_only_user_dialogue_lines() -> None:
    context = _execution_report_dialogue_context(
        [
            'user:Move your head in all directions again.',
            'assistant:I will move my head in all directions. I am ready for your next request!',
            'user:Which directions?',
            'assistant:I moved my head left and right.',
        ]
    )

    assert context == [
        'user:Move your head in all directions again.',
        'user:Which directions?',
    ]


def test_plan_outcome_summary_tracks_completed_failed_and_pending_targets() -> None:
    summary = _plan_outcome_summary(
        [
            {'id': 'step_1', 'type': 'skill', 'name': 'bring_object', 'args': {'target': 'cup'}},
            {'id': 'step_2', 'type': 'skill', 'name': 'bring_object', 'args': {'target': 'book'}},
            {'id': 'step_3', 'type': 'skill', 'name': 'bring_object', 'args': {'target': 'phone'}},
            {'id': 'step_4', 'type': 'skill', 'name': 'report_result', 'args': {}},
        ],
        [
            {
                'id': 'step_1',
                'name': 'bring_object',
                'status': 'succeeded',
                'args': {'target': 'cup'},
                'result_payload': {'target': 'cup'},
            },
            {
                'id': 'step_2',
                'name': 'bring_object',
                'status': 'failed',
                'args': {'target': 'book'},
                'result_payload': {'target': 'book'},
            },
        ],
        terminal_reason='blocked',
        terminal_step={'id': 'step_2'},
    )

    assert summary == {
        'completed_targets': ['cup'],
        'failed_targets': ['book'],
        'pending_targets': ['phone'],
        'last_successful_step_id': 'step_1',
        'terminal_step_id': 'step_2',
        'terminal_reason': 'blocked',
        'all_required_steps_succeeded': False,
    }


def test_execution_context_retains_admitted_request_for_report_result() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._planner_request_context_by_goal = {}
    orchestrator._planner_request_context_order = []
    orchestrator._remember_planner_request_context(
        'goal_1',
        {
            'goal_id': 'goal_1',
            'goal_text': 'move your head in all directions and wave',
            'dialogue_context': ['user:move your head in all directions and wave'],
            'grounded_context': {'entities': [{'id': 'person_1'}]},
        },
    )

    context = orchestrator._execution_context_for_goal(
        'goal_1',
        {'plan': {'goal_id': 'goal_1', 'steps': []}},
    )

    assert context['goal_text'] == 'move your head in all directions and wave'
    assert context['dialogue_context'][0].startswith('user:')
    assert context['grounded_context']['entities'][0]['id'] == 'person_1'
    assert context['plan']['goal_id'] == 'goal_1'


def test_plan_validation_uses_admitted_grounding_for_delivery_recipient() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._planner_request_context_by_goal = {
        'goal_delivery': {
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'kind': 'object'},
                    {'id': 'person_1', 'kind': 'person'},
                ]
            }
        }
    }
    plan_data = {
        'plan': {
            'goal_id': 'goal_delivery',
            'plan_id': 'plan_delivery',
            'plan_version': 1,
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'deliver',
                'member_ids': ['cup_1'],
                'recipient_id': 'person_1',
            },
            'steps': [
                {
                    'id': 'step_1',
                    'type': 'skill',
                    'name': 'bring_object',
                    'args': {'object_id': 'cup_1', 'recipient_id': 'person_1'},
                }
            ],
        }
    }

    context = orchestrator._validated_plan_context('raw_user_input', plan_data)

    assert context is not None
    assert context['errors'] == []
    assert [step['name'] for step in context['steps']] == ['bring_object']


def test_report_result_text_falls_back_to_successful_step_chain() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'execution_results': [
                {
                    'name': 'navigate_to',
                    'status': 'succeeded',
                    'result_summary': 'I navigated to the cup.',
                },
                {
                    'name': 'scan',
                    'status': 'succeeded',
                    'result_summary': 'I found two blueberries.',
                },
            ],
        },
    )

    assert report_text == 'I navigated to the cup. I found two blueberries.'


def test_report_result_text_preserves_delivery_step_evidence_before_generic_fallback() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'execution_results': [
                {
                    'name': 'bring_object',
                    'status': 'succeeded',
                    'args': {'object_id': 'cup_1', 'recipient': 'person_1'},
                    'result_summary': 'I brought cup_1 to person_1.',
                },
                {
                    'name': 'bring_object',
                    'status': 'succeeded',
                    'args': {'object_id': 'phone_1', 'recipient': 'person_1'},
                    'result_summary': 'I brought phone_1 to person_1.',
                },
            ],
        },
    )

    assert report_text == 'I brought cup_1 to person_1. I brought phone_1 to person_1.'


def test_report_result_text_does_not_fabricate_delivery_without_evidence() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'plan_context': {
                'target_selection': {
                    'selection_kind': 'explicit_members',
                    'operation': 'deliver',
                    'member_ids': ['cup_1'],
                    'recipient_id': 'person_1',
                }
            },
            'plan_steps': [
                {
                    'id': 'step_1',
                    'type': 'skill',
                    'name': 'bring_object',
                    'args': {'target': 'cup_1', 'recipient': 'person_1'},
                }
            ],
            'execution_results': [],
        },
    )

    assert report_text == ''


def test_motion_result_payload_supplies_reportable_internal_summary() -> None:
    payload = _motion_result_payload(
        'head_motion',
        {'motion': 'head_look_right'},
        {'motion_name': 'head_look_right'},
    )

    assert payload['skill'] == 'perform_motion'
    assert payload['status'] == 'succeeded'
    assert payload['summary_text'] == 'I moved my head right.'
    assert payload['metadata']['speech_produced'] is False


def test_report_result_text_can_reuse_motion_chain_summaries() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'execution_results': [
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head left.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head right.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I centered my head.',
                },
            ],
        },
    )

    assert report_text == (
        'I moved my head left. I moved my head right. I centered my head.'
    )


def test_report_result_routes_explicit_summary_through_chatbot_first() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    captured = {}

    def _chatbot_report(context):
        captured.update(context)
        return _ExecutionReportResult(
            text='I completed the requested motion and waved at you.',
            source='chatbot',
        )

    orchestrator._request_execution_report_text = _chatbot_report
    orchestrator.get_logger = lambda: type(
        'Logger',
        (),
        {'info': lambda *_args, **_kwargs: None},
    )()

    report_text = orchestrator._resolve_report_result_text(
        {'summary_text': 'I moved left. I moved right. I waved.'},
        {
            'goal_text': 'move your head in all directions and wave',
            'grounded_context': {'entities': [{'id': 'person_1', 'label': 'person'}]},
        },
    )

    assert report_text == 'I completed the requested motion and waved at you.'
    assert captured['requested_summary'] == 'I moved left. I moved right. I waved.'
    assert captured['grounded_context']['entities'][0]['id'] == 'person_1'


def test_report_result_text_preserves_motion_chain_before_terminal_result() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'execution_results': [
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I centered my head.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head left.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head right.',
                },
                {
                    'name': 'wave_greet',
                    'status': 'succeeded',
                    'result_summary': 'I performed a friendly wave.',
                },
            ],
        },
    )

    assert report_text == (
        'I centered my head. I moved my head left. I moved my head right. '
        'I performed a friendly wave.'
    )


def test_report_result_text_preserves_bounded_motion_chain_fallback() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator._request_execution_report_text = (
        lambda _context: _ExecutionReportResult(source='unavailable')
    )

    report_text = orchestrator._resolve_report_result_text(
        {},
        {
            'execution_results': [
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head left.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head right.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head up.',
                },
                {
                    'name': 'perform_motion',
                    'status': 'succeeded',
                    'result_summary': 'I moved my head down.',
                },
                {
                    'name': 'wave_greet',
                    'status': 'succeeded',
                    'result_summary': 'I performed a friendly wave.',
                },
            ],
        },
    )

    assert report_text == (
        'I moved my head left. I moved my head right. I moved my head up. '
        'I moved my head down. I performed a friendly wave.'
    )


def test_scene_scan_payload_preserves_positional_evidence() -> None:
    payload = build_scan_result_payload(
        {
            'target_kind': 'scene',
            'objects': [
                {
                    'id': 'cup_1',
                    'label': 'cup',
                    'source': 'scene_summary',
                    'center_x': 0.22,
                    'center_y': 0.61,
                    'confidence': 0.94,
                }
            ],
        }
    )

    assert payload['objects'][0]['id'] == 'cup_1'
    assert payload['objects'][0]['center_x'] == 0.22
    assert payload['objects'][0]['center_y'] == 0.61
    assert payload['objects'][0]['confidence'] == 0.94


def test_scene_scan_payload_reports_people_and_objects_separately() -> None:
    payload = build_scan_result_payload(
        {
            'target_kind': 'scene',
            'objects': [{'id': 'cup_1', 'label': 'cup', 'source': 'scene_summary'}],
            'people': [{'id': 'anonymous_person_abc', 'source': 'hri_persons'}],
        }
    )

    assert payload['target_found'] is True
    assert payload['objects'][0]['label'] == 'cup'
    assert payload['people'][0]['id'] == 'anonymous_person_abc'
    assert 'detected cup' in payload['summary_text']
    assert 'one person (id: anonymous_person_abc)' in payload['summary_text']


def test_people_scan_target_detection_supports_common_aliases() -> None:
    assert is_people_scan_target('people') is True
    assert is_people_scan_target('human') is True
    assert is_people_scan_target('scene', target='person') is True
    assert is_people_scan_target('object', target='bottle') is False


def test_people_scan_summary_formats_single_and_plural() -> None:
    assert summarize_people_detection([]) == ''
    assert summarize_people_detection(['anonymous_person_abc']) == (
        'I found one person (id: anonymous_person_abc).'
    )
    assert summarize_people_detection(
        ['anonymous_person_abc', 'anonymous_person_xyz']
    ) == 'I found 2 people (for example: anonymous_person_abc).'


def test_validate_execution_plan_rejects_duplicate_plan_step_ids() -> None:
    envelope = validate_execution_plan(
        Intent.PERFORM_MOTION,
        {
            'plan': [
                {
                    'id': 'wave',
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'stand'},
                },
                {
                    'id': 'wave',
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'sit'},
                },
            ]
        },
    )
    assert [step['id'] for step in envelope['steps']] == ['wave']
    assert envelope['errors'] == ['duplicate plan step id: wave']


def test_posture_topic_fallback_for_motion_preserves_legacy_bridge_names() -> None:
    assert posture_topic_fallback_for_motion('standinit') == 'stand'
    assert posture_topic_fallback_for_motion('crouch') == 'kneel'


def test_make_intent_signature_is_stable_for_same_payload() -> None:
    left = make_intent_signature(Intent.SAY, {'recipient': 'p1', 'object': 'hello'})
    right = make_intent_signature(Intent.SAY, {'object': 'hello', 'recipient': 'p1'})
    assert left == right


def test_make_intent_signature_ignores_ack_text_only_differences() -> None:
    left = make_intent_signature(
        Intent.PERFORM_MOTION,
        {'object': 'stand', 'ack_text': 'Sure.'},
    )
    right = make_intent_signature(
        Intent.PERFORM_MOTION,
        {'object': 'stand', 'ack_text': 'Okay.'},
    )
    assert left == right


def test_orchestrator_fake_perform_motion_mode_routes_to_fake_skill() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator.perform_motion_execution_mode = 'fake'
    calls = []

    def fake_execute(skill_name, step_args, *, on_started=None):
        calls.append((skill_name, step_args))
        return True, '', {'skill': skill_name, 'status': 'succeeded'}

    orchestrator._execute_fake_skill_step = fake_execute

    success, reason, payload = NaoOrchestrator._execute_motion_plan_step(
        orchestrator,
        {'object': 'head_look_left'},
    )

    assert success is True
    assert reason == ''
    assert payload == {'skill': 'perform_motion', 'status': 'succeeded'}
    assert calls == [('perform_motion', {'object': 'head_look_left'})]


def test_orchestrator_fake_look_at_mode_routes_to_fake_skill() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator.look_at_execution_mode = 'fake'
    orchestrator._stats = type('Stats', (), {'dispatched_look_at': 0})()
    calls = []

    def fake_execute(skill_name, step_args, *, on_started=None):
        calls.append((skill_name, step_args))
        return True, '', {'skill': skill_name, 'status': 'succeeded'}

    orchestrator._execute_fake_skill_step = fake_execute

    success, reason = NaoOrchestrator._dispatch_planned_look_at(
        orchestrator,
        'look_at',
        {'target_frame': 'person_1'},
    )

    assert success is True
    assert reason == ''
    assert orchestrator._stats.dispatched_look_at == 1
    assert calls == [('look_at', {'target_frame': 'person_1'})]


def test_orchestrator_real_perform_motion_mode_keeps_head_action_route() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator.perform_motion_execution_mode = 'real'
    orchestrator._stats = type('Stats', (), {'dispatched_head_motion': 0})()
    calls = []

    def fake_head(payload, *, on_started=None):
        calls.append(payload)
        return False, 'head motion dispatch failed'

    orchestrator._execute_head_motion_step = fake_head

    success, reason, payload = NaoOrchestrator._execute_motion_plan_step(
        orchestrator,
        {'object': 'head_look_left'},
    )

    assert success is False
    assert reason == 'head motion dispatch failed'
    assert payload == {}
    assert calls and calls[0]['yaw'] == 0.45
