from types import SimpleNamespace

from nao_chatbot.chat_goal_codec import coerce_user_intent
from nao_chatbot.chat_goal_codec import extract_canonical_goal
from nao_chatbot.chat_goal_codec import extract_json_object


def _goal(
    configuration: str,
    initial_input: str = "",
    person_id: str = "",
    group_id: str = "",
):
    return SimpleNamespace(
        role=SimpleNamespace(configuration=configuration),
        initial_input=initial_input,
        person_id=person_id,
        group_id=group_id,
    )


def test_extract_canonical_goal_prefers_user_message() -> None:
    goal = _goal(
        '{"user_message":"hello","conversation_history":["user:hi"],"user_id":"u1"}'
    )

    user_text, history, user_id, turn_id = extract_canonical_goal(goal)

    assert user_text == "hello"
    assert history == ["user:hi"]
    assert user_id == "u1"
    assert turn_id == "unknown"


def test_extract_canonical_goal_reads_turn_id() -> None:
    goal = _goal(
        '{"user_message":"hello","conversation_history":[],"user_id":"u1","turn_id":"t00042"}'
    )

    user_text, history, user_id, turn_id = extract_canonical_goal(goal)

    assert user_text == "hello"
    assert history == []
    assert user_id == "u1"
    assert turn_id == "t00042"


def test_extract_json_object_from_mixed_payload() -> None:
    payload = "noise before {\"user_intent\":{\"type\":\"help\"}} trailing"
    parsed = extract_json_object(payload)

    assert parsed["user_intent"]["type"] == "help"


def test_coerce_user_intent_string() -> None:
    assert coerce_user_intent("identity") == {"type": "identity"}
