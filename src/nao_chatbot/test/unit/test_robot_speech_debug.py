from nao_chatbot.speech_debug_labels import classify_closed_caption_speaker


def test_classify_closed_caption_speaker_marks_system_as_robot_output() -> None:
    assert classify_closed_caption_speaker("__system__", system_speaker_id="__system__") == (
        "ROBOT OUTPUT",
        "closed_caption",
    )


def test_classify_closed_caption_speaker_marks_voice_as_user_input() -> None:
    assert classify_closed_caption_speaker(
        "anonymous_speaker",
        system_speaker_id="__system__",
    ) == (
        "USER INPUT",
        "closed_caption:anonymous_speaker",
    )


def test_classify_closed_caption_speaker_handles_missing_voice_id() -> None:
    assert classify_closed_caption_speaker("", system_speaker_id="__system__") == (
        "USER INPUT",
        "closed_caption:unknown",
    )
