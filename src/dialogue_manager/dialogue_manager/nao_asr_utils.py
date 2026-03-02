"""Minimal LiveSpeech text extraction helpers for NAO bridge flows."""


def extract_text(msg) -> str:
    """Extract best-effort final text from a LiveSpeech-like message."""
    for field_name in ("final", "incremental", "text", "transcript", "utterance"):
        value = getattr(msg, field_name, "")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""
