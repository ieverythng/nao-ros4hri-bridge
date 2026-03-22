import json

from hri_actions_msgs.msg import Intent
from kb_skills.intent_labels import KB_QUERY_VISIBLE_PEOPLE

from nao_orchestrator.intent_rules import classify_motion_target
from nao_orchestrator.intent_rules import make_intent_signature
from nao_orchestrator.intent_rules import normalize_incoming_intent
from nao_orchestrator.intent_rules import normalize_legacy_intent
from nao_orchestrator.intent_rules import parse_execution_plan
from nao_orchestrator.intent_rules import parse_intent_data
from nao_orchestrator.intent_rules import posture_topic_fallback_for_motion
from nao_orchestrator.intent_rules import resolve_ack_text
from nao_orchestrator.intent_rules import resolve_say_text


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
            'type': 'skill',
            'name': 'perform_motion',
            'args': {'object': 'stand'},
        }
    ]


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
