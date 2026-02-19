from nao_chatbot.asr_utils import extract_text
from nao_chatbot.asr_utils import normalize_text
from nao_chatbot.asr_utils import parse_vosk_result
from nao_chatbot.asr_utils import set_text


class FakeSpeech:
    def __init__(self) -> None:
        self.final = ""
        self.incremental = ""


def test_normalize_text() -> None:
    assert normalize_text("  hello   world ") == "hello world"
    assert normalize_text("") == ""
    assert normalize_text(None) == ""


def test_parse_vosk_result_prefers_text() -> None:
    assert parse_vosk_result('{"text":"hello world","partial":"hello"}') == "hello world"
    assert parse_vosk_result('{"partial":"hello"}') == "hello"
    assert parse_vosk_result("{}") == ""
    assert parse_vosk_result("not-json") == ""


def test_set_text_final_and_extract() -> None:
    msg = FakeSpeech()
    assert set_text(msg, "hello robot", partial=False)
    assert msg.final == "hello robot"
    assert extract_text(msg) == "hello robot"


def test_set_text_partial_uses_incremental() -> None:
    msg = FakeSpeech()
    assert set_text(msg, "hello", partial=True)
    assert msg.incremental == "hello"
    assert extract_text(msg) == "hello"
