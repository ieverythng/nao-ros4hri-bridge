from nao_chatbot.chat_history import history_to_messages
from nao_chatbot.chat_history import messages_to_history
from nao_chatbot.chat_history import trim_messages


def test_history_round_trip_preserves_roles() -> None:
    history = ["user:hello", "assistant:hi there"]
    messages = history_to_messages(history, max_history_messages=10)

    assert messages == [
        {"role": "user", "content": "hello"},
        {"role": "assistant", "content": "hi there"},
    ]

    recovered = messages_to_history(messages, max_history_messages=10)
    assert recovered == history


def test_trim_messages_preserves_leading_system() -> None:
    messages = [
        {"role": "system", "content": "s"},
        {"role": "user", "content": "u1"},
        {"role": "assistant", "content": "a1"},
        {"role": "user", "content": "u2"},
    ]
    trimmed = trim_messages(messages, max_history_messages=2)

    assert trimmed == [
        {"role": "system", "content": "s"},
        {"role": "assistant", "content": "a1"},
        {"role": "user", "content": "u2"},
    ]
