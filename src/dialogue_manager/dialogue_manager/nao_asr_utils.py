"""Minimal LiveSpeech text extraction helpers for NAO bridge flows."""


def extract_text(msg, allow_incremental: bool = False) -> str:
    """Extract text from a LiveSpeech-like message.

    By default this only accepts final hypotheses. Incremental text can be
    enabled explicitly for debug or UI-only flows.
    """
    final_value = getattr(msg, "final", "")
    if isinstance(final_value, str) and final_value.strip():
        return final_value.strip()

    fallback_fields = ("text", "transcript", "utterance")
    if allow_incremental:
        fallback_fields = ("incremental",) + fallback_fields

    for field_name in fallback_fields:
        value = getattr(msg, field_name, "")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def merge_text_chunks(existing: str, incoming: str) -> str:
    """Merge overlapping ASR text chunks into one utterance candidate."""
    left = " ".join(str(existing).split())
    right = " ".join(str(incoming).split())
    if not left:
        return right
    if not right:
        return left

    left_lower = left.lower()
    right_lower = right.lower()
    if left_lower == right_lower:
        return left
    if right_lower.startswith(left_lower):
        return right
    if left_lower.startswith(right_lower):
        return left

    left_tokens = left.split()
    right_tokens = right.split()
    max_overlap = min(len(left_tokens), len(right_tokens))
    for overlap in range(max_overlap, 0, -1):
        left_slice = [token.lower() for token in left_tokens[-overlap:]]
        right_slice = [token.lower() for token in right_tokens[:overlap]]
        if left_slice == right_slice:
            return " ".join(left_tokens + right_tokens[overlap:])

    return " ".join(left_tokens + right_tokens)
