from types import SimpleNamespace

from interaction_trace_viewer.payload_normalizer import classify_speech_topic
from interaction_trace_viewer.payload_normalizer import normalize_include_event_types
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


def test_normalize_turn_trace_string_message() -> None:
    msg = SimpleNamespace(
        data='{"event_type":"chatbot_turn_result","route":"dialogue","intent":"greet","intent_source":"llm_response_route"}'
    )
    event = normalize_string_message(
        channel='/chatbot_llm/turn_trace',
        msg=msg,
        max_payload_chars=4000,
    )

    assert event.event_type == 'chatbot_turn_trace'
    assert 'route=dialogue' in event.summary


def test_normalize_fake_skill_event_maps_to_skill_result() -> None:
    msg = SimpleNamespace(
        data='{"event_type":"fake_skill_completed","skill":"navigate_to","payload":{"status":"failed","summary_text":"path blocked","failure":{"code":"path_blocked"}}}'
    )
    event = normalize_string_message(
        channel='/fake_skills/events',
        msg=msg,
        max_payload_chars=4000,
    )

    assert event.event_type == 'skill_result'
    assert event.payload['skill'] == 'navigate_to'
    assert event.payload['status'] == 'failed'
    assert event.payload['failure']['code'] == 'path_blocked'


def test_normalize_world_model_snapshot_maps_to_kb_snapshot() -> None:
    msg = SimpleNamespace(
        data='{"entities":[{"entity_id":"anonymous_person_1","kb_class":"Human"}]}'
    )
    event = normalize_string_message(
        channel='/world_model/enriched_snapshot',
        msg=msg,
        max_payload_chars=4000,
    )

    assert event.event_type == 'kb_snapshot'
    assert 'entities=1' in event.summary


def test_normalize_include_event_types_adds_kb_snapshot_when_world_model_channel_selected() -> None:
    include_event_types = normalize_include_event_types(
        include_channels={'planner/request', 'world_model/enriched_snapshot'},
        include_event_types={'planner_request', 'execution_feedback'},
        exclude_event_types=set(),
    )

    assert 'kb_snapshot' in include_event_types
