from rcl_interfaces.msg import Log

from nao_chatbot.demo_rosout_filter import _LEVELS
from nao_chatbot.demo_rosout_filter import _format_intent_event
from nao_chatbot.demo_rosout_filter import _format_json_event
from nao_chatbot.demo_rosout_filter import _normalize_node_name
from nao_chatbot.demo_rosout_filter import _parse_nodes


class _IntentMsg:
    intent = 'planner_request'
    source = 'chatbot_llm'
    modality = 'speech'
    data = '{"plan":{"goal_id":"goal-1","plan_id":"plan-1","validation_status":"valid","steps":[{},{}]}}'


def test_parse_nodes_normalizes_slashes_and_empty_values():
    assert _parse_nodes('/chatbot_llm, planner_llm,,/nao_orchestrator ') == (
        'chatbot_llm',
        'planner_llm',
        'nao_orchestrator',
    )


def test_parse_nodes_falls_back_to_default_allowlist():
    assert 'planner_llm' in _parse_nodes('')


def test_log_levels_match_rosgraph_constants():
    assert _LEVELS['info'] == Log.INFO
    assert _LEVELS['warn'] == Log.WARN
    assert _LEVELS['error'] == Log.ERROR


def test_normalize_node_name_strips_leading_slash():
    assert _normalize_node_name('/dialogue_manager') == 'dialogue_manager'


def test_format_intent_event_summarizes_planner_plan():
    line = _format_intent_event('PLANNER_REQUEST', _IntentMsg())

    assert line.startswith('PLANNER_REQUEST')
    assert 'intent=planner_request' in line
    assert 'goal_id=goal-1' in line
    assert 'plan_id=plan-1' in line
    assert 'steps=2' in line


def test_format_json_event_summarizes_planner_dialogue_act():
    line = _format_json_event(
        'PLANNER_ACT',
        '{"act":"acknowledge","goal_id":"goal-1","text_hint":"I will move my head."}',
    )

    assert line.startswith('PLANNER_ACT')
    assert 'act=acknowledge' in line
    assert 'goal_id=goal-1' in line
    assert 'hint=I will move my head.' in line
