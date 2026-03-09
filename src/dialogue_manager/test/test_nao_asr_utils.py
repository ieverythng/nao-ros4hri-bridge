from dialogue_manager.nao_asr_utils import extract_text
from dialogue_manager.nao_asr_utils import merge_text_chunks


class _Msg:
    def __init__(self, final="", incremental="", text="", transcript="", utterance=""):
        self.final = final
        self.incremental = incremental
        self.text = text
        self.transcript = transcript
        self.utterance = utterance


def test_extract_text_prefers_final_only_by_default():
    msg = _Msg(final="", incremental="hello there")

    assert extract_text(msg) == ""


def test_extract_text_uses_incremental_when_enabled():
    msg = _Msg(final="", incremental="hello there")

    assert extract_text(msg, allow_incremental=True) == "hello there"


def test_extract_text_uses_final_when_available():
    msg = _Msg(final="hello world", incremental="hello")

    assert extract_text(msg) == "hello world"


def test_merge_text_chunks_replaces_prefix_with_longer_hypothesis():
    assert merge_text_chunks("hello", "hello robot") == "hello robot"


def test_merge_text_chunks_merges_non_overlapping_segments():
    assert merge_text_chunks("what is", "your name") == "what is your name"


def test_merge_text_chunks_preserves_existing_when_new_is_shorter_prefix():
    assert merge_text_chunks("hello robot", "hello") == "hello robot"
