from types import SimpleNamespace

from interaction_trace_viewer.payload_normalizer import classify_speech_topic
from interaction_trace_viewer.payload_normalizer import normalize_intent_message
from interaction_trace_viewer.payload_normalizer import normalize_rosout_message
from interaction_trace_viewer.payload_normalizer import normalize_string_message


def test_normalize_string_message_parses_json_payload() -> None:
    msg = SimpleNamespace(data='{"goal_text":"scan room","scene_targets":["room"]}')
    event = normalize_string_message(
        channel='/planner/request',
        msg=msg,
        max_payload_chars=4000,
    )

    assert event.event_type == 'planner_request'
    assert event.payload['goal_text'] == 'scan room'
    assert 'goal=' in event.summary


def test_classify_speech_topic_distinguishes_user_from_robot() -> None:
    assert classify_speech_topic('/humans/voices/alice/speech') == 'user_utterance'
    assert classify_speech_topic('/robot/speech') == 'robot_speech'


def test_normalize_intent_message_extracts_nested_trace_id() -> None:
    msg = SimpleNamespace(
        intent='planner_request',
        data='{"goal_id":"goal_42","goal_text":"scan room"}',
        person_id='',
        intent_type='',
        priority=10,
        confidence=0.9,
    )
    event = normalize_intent_message(
        channel='/planner/request',
        msg=msg,
        max_payload_chars=4000,
    )

    assert event.trace_id == 'goal_42'
    assert event.event_type == 'planner_request'


def test_normalize_rosout_message_classifies_errors_over_warnings() -> None:
    warning = SimpleNamespace(
        stamp=SimpleNamespace(sec=1, nanosec=0),
        name='node_a',
        msg='warning text',
        level=30,
        file='a.py',
        function='fn',
        line=1,
    )
    error = SimpleNamespace(
        stamp=SimpleNamespace(sec=2, nanosec=0),
        name='node_b',
        msg='error text',
        level=40,
        file='b.py',
        function='fn',
        line=2,
    )

    warning_event = normalize_rosout_message(
        channel='/rosout',
        msg=warning,
        max_payload_chars=4000,
    )
    error_event = normalize_rosout_message(
        channel='/rosout',
        msg=error,
        max_payload_chars=4000,
    )

    assert warning_event.event_type == 'warning'
    assert error_event.event_type == 'error'
