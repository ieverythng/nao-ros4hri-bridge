from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import posture_command_for_intent


def test_detect_intent_posture_keywords() -> None:
    assert detect_intent("Can you stand up please?") == "posture_stand"
    assert detect_intent("Please sit down now.") == "posture_sit"
    assert detect_intent("Could you kneel for me?") == "posture_kneel"


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
    assert "Nice to meet you" in build_rule_response("greet")
    assert "testing the chat to speech pipeline" in build_rule_response("fallback")


def test_posture_command_mapping() -> None:
    assert posture_command_for_intent("posture_stand") == "stand"
    assert posture_command_for_intent("posture_sit") == "sit"
    assert posture_command_for_intent("posture_kneel") == "kneel"
    assert posture_command_for_intent("fallback") == ""
