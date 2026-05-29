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
from nao_orchestrator.intent_rules import is_people_scan_target
from nao_orchestrator.intent_rules import is_unresolved_report_template
from nao_orchestrator.intent_rules import summarize_people_detection
from nao_orchestrator.intent_rules import validate_execution_plan


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
                'goal_token': 'goal-7:turn-5',
                'plan_id': 'plan-42',
                'plan_version': 3,
                'status': 'executing',
                'validation_status': 'draft',
                'communication_policy': {'emit_acknowledge': False},
                'steps': [
                    {'type': 'say', 'args': {'text': 'hello'}},
                ],
            },
            'scene_targets': ['cup'],
        }
    )
    assert envelope['goal_id'] == 'goal-7'
    assert envelope['goal_token'] == 'goal-7:turn-5'
    assert envelope['plan_id'] == 'plan-42'
    assert envelope['plan_version'] == 3
    assert envelope['status'] == 'executing'
    assert envelope['validation_status'] == 'draft'
    assert envelope['scene_targets'] == ['cup']
    assert envelope['communication_policy']['emit_acknowledge'] is False
    assert envelope['steps'][0]['id'] == 'step_1'


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
        'step_1: look_at step is missing target_frame or reset policy'
    ]


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
