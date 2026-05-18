"""Shared result payload builders for fake skills."""

from __future__ import annotations

try:  # pragma: no cover - runtime dependency
    from skill_common import SkillResultPayload
except ImportError:  # pragma: no cover - unit-test fallback
    class SkillResultPayload:  # type: ignore[override]
        """Fallback payload shim when skill_common is not importable."""

        def __init__(self, payload: dict) -> None:
            self._payload = dict(payload or {})

        @classmethod
        def from_dict(cls, payload: dict) -> 'SkillResultPayload':
            return cls(payload)

        def to_dict(self) -> dict:
            return dict(self._payload)


def build_skill_result(
    *,
    skill: str,
    status: str,
    target: str = '',
    target_kind: str = '',
    target_found: bool | None = None,
    summary_text: str = '',
    evidence: dict | None = None,
    failure: dict | None = None,
    metadata: dict | None = None,
) -> SkillResultPayload:
    return SkillResultPayload.from_dict(
        {
            'skill': str(skill or '').strip(),
            'status': str(status or '').strip() or 'unknown',
            'target': str(target or '').strip(),
            'target_kind': str(target_kind or '').strip(),
            'target_found': target_found,
            'summary_text': str(summary_text or '').strip(),
            'evidence': dict(evidence or {}),
            'failure': dict(failure or {}),
            'metadata': dict(metadata or {}),
        }
    )


def failure_block(
    *,
    code: str,
    message: str,
    recoverable: bool,
    suggested_recovery: str,
) -> dict:
    return {
        'code': str(code or '').strip(),
        'message': str(message or '').strip(),
        'recoverable': bool(recoverable),
        'suggested_recovery': str(suggested_recovery or '').strip(),
    }
