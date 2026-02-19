import json


def extract_text(msg) -> str:
    for field_name in ("final", "incremental", "text", "transcript", "utterance"):
        value = getattr(msg, field_name, "")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def set_text(msg, text: str, partial: bool = False) -> bool:
    clean = normalize_text(text)
    if not clean:
        return False

    candidates = (
        ("incremental", "final", "text", "transcript", "utterance")
        if partial
        else ("final", "text", "transcript", "utterance", "incremental")
    )
    for field_name in candidates:
        if hasattr(msg, field_name):
            setattr(msg, field_name, clean)
            return True
    return False


def parse_vosk_result(payload: str) -> str:
    if not payload:
        return ""
    try:
        parsed = json.loads(payload)
    except json.JSONDecodeError:
        return ""
    for key in ("text", "partial", "transcript", "utterance"):
        value = parsed.get(key, "")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def normalize_text(text: str) -> str:
    if not isinstance(text, str):
        return ""
    return " ".join(text.strip().split())
