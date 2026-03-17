from __future__ import annotations


def classify_closed_caption_speaker(
    speaker_id: str,
    *,
    system_speaker_id: str,
) -> tuple[str, str]:
    """Classify caption speaker IDs for human-readable operator logging."""
    clean_speaker_id = str(speaker_id).strip()
    if clean_speaker_id == str(system_speaker_id).strip():
        return ("ROBOT OUTPUT", "closed_caption")
    if clean_speaker_id:
        return ("USER INPUT", f"closed_caption:{clean_speaker_id}")
    return ("USER INPUT", "closed_caption:unknown")
