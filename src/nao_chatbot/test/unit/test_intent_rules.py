from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import head_motion_goal_for_intent
from nao_chatbot.intent_rules import normalize_intent
from nao_chatbot.intent_rules import posture_command_for_intent


def test_detect_intent_posture_keywords() -> None:
    assert detect_intent("Can you stand up please?") == "posture_stand"
    assert detect_intent("Please sit down now.") == "posture_sit"
    assert detect_intent("Could you kneel for me?") == "posture_kneel"


def test_detect_intent_head_keywords() -> None:
    assert detect_intent("Please look left") == "head_look_left"
    assert detect_intent("Turn your head right") == "head_look_right"
    assert detect_intent("Can you look up?") == "head_look_up"
    assert detect_intent("Please look down.") == "head_look_down"
    assert detect_intent("Look center now") == "head_center"


def test_detect_intent_non_posture_keywords() -> None:
    assert detect_intent("Hello there") == "greet"
    assert detect_intent("How are you?") == "wellbeing"
    assert detect_intent("What is your name?") == "identity"
    assert detect_intent("help me") == "help"


def test_detect_intent_fallback() -> None:
    assert detect_intent("this sentence should not match") == "fallback"


def test_rule_responses_exist() -> None:
    assert "standing posture" in build_rule_response("posture_stand")
    assert "sitting posture" in build_rule_response("posture_sit")
    assert "kneeling posture" in build_rule_response("posture_kneel")
    assert "left" in build_rule_response("head_look_left")
    assert "centering" in build_rule_response("head_center")
    assert "Nice to meet you" in build_rule_response("greet")
    assert "testing the chat to speech pipeline" in build_rule_response("fallback")


def test_posture_command_mapping() -> None:
    assert posture_command_for_intent("posture_stand") == "stand"
    assert posture_command_for_intent("posture_sit") == "sit"
    assert posture_command_for_intent("posture_kneel") == "kneel"
    assert posture_command_for_intent("fallback") == ""


def test_head_motion_goal_mapping() -> None:
    assert head_motion_goal_for_intent("head_look_left") == {
        "yaw": 0.45,
        "pitch": 0.0,
        "relative": False,
    }
    assert head_motion_goal_for_intent("head_center") == {
        "yaw": 0.0,
        "pitch": 0.0,
        "relative": False,
    }
    assert head_motion_goal_for_intent("fallback") == {}


def test_normalize_intent_aliases_and_keywords() -> None:
    assert normalize_intent("__intent_greet__") == "greet"
    assert normalize_intent("PLEASE_STAND") == "posture_stand"
    assert normalize_intent("__intent_look_left__") == "head_look_left"
    assert normalize_intent("who_are_you") == "identity"
    assert (
        normalize_intent("__intent_start_activity__", hint_text="please stand up now")
        == "posture_stand"
    )
    assert normalize_intent("turn_right_now") == "head_look_right"
    assert normalize_intent("something_unknown") == "fallback"
