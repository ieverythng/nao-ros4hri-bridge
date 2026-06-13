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
from nao_orchestrator.orchestrator import _normalize_execution_mode
from nao_orchestrator.orchestrator import _report_text_from_result_payload
from nao_orchestrator.orchestrator import NaoOrchestrator


def test_parse_intent_data_returns_dict_for_valid_json() -> None:
    payload = parse_intent_data('{"object":"stand"}')
    assert payload == {'object': 'stand'}


def test_normalize_legacy_intent_maps_posture_to_perform_motion() -> None:
    intent_name, data = normalize_legacy_intent('posture_stand', 'Hello there!')
    assert intent_name == Intent.PERFORM_MOTION
    assert data['object'] == 'stand'


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


def test_normalize_execution_mode_defaults_to_real() -> None:
    assert _normalize_execution_mode('fake') == 'fake'
    assert _normalize_execution_mode('real') == 'real'
    assert _normalize_execution_mode('unexpected') == 'real'


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
    assert reason == 'I completed the scan for people, but no confirmed detection result was reported.'
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
            'plan_context': {'plan_id': 'plan_1', 'plan_version': 2},
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
    assert [step['name'] for step in captured['steps']] == ['navigate_to', 'scan']


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

    success, reason = NaoOrchestrator._execute_motion_plan_step(
        orchestrator,
        {'object': 'head_look_left'},
    )

    assert success is True
    assert reason == ''
    assert calls == [('perform_motion', {'object': 'head_look_left'})]


def test_orchestrator_real_perform_motion_mode_keeps_head_action_route() -> None:
    orchestrator = NaoOrchestrator.__new__(NaoOrchestrator)
    orchestrator.perform_motion_execution_mode = 'real'
    orchestrator._stats = type('Stats', (), {'dispatched_head_motion': 0})()
    calls = []

    def fake_head(payload, *, on_started=None):
        calls.append(payload)
        return False, 'head motion dispatch failed'

    orchestrator._execute_head_motion_step = fake_head

    success, reason = NaoOrchestrator._execute_motion_plan_step(
        orchestrator,
        {'object': 'head_look_left'},
    )

    assert success is False
    assert reason == 'head motion dispatch failed'
    assert calls and calls[0]['yaw'] == 0.45
