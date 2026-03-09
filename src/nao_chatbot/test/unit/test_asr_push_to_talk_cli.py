from nao_chatbot.asr_push_to_talk_cli import KeyAction
from nao_chatbot.asr_push_to_talk_cli import build_parser
from nao_chatbot.asr_push_to_talk_cli import interpret_key


def test_interpret_key_toggles_on_space() -> None:
    assert interpret_key(False, " ") == KeyAction(True, False, False)
    assert interpret_key(True, " ") == KeyAction(False, False, False)


def test_interpret_key_opens_and_closes() -> None:
    assert interpret_key(False, "o") == KeyAction(True, False, False)
    assert interpret_key(True, "c") == KeyAction(False, False, False)


def test_interpret_key_quit_and_help() -> None:
    assert interpret_key(False, "q") == KeyAction(False, True, False)
    assert interpret_key(True, "?") == KeyAction(True, False, True)


def test_build_parser_defaults_to_interactive_mode() -> None:
    args = build_parser().parse_args([])

    assert args.topic == "/asr_vosk/push_to_talk"
    assert args.open is False
    assert args.close is False
    assert args.pulse is None
    assert args.close_on_exit is True
