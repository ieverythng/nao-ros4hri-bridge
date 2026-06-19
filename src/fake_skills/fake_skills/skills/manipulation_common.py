"""Shared helpers for fake manipulation skills."""

from __future__ import annotations

from fake_skills.result_builders import failure_block


def clean_arg(args: dict, *keys: str, default: str = '') -> str:
    for key in keys:
        value = str(args.get(key, '')).strip()
        if value:
            return value
    return default


def kb_effect(action: str, statement: str) -> dict:
    return {
        'action': str(action or '').strip(),
        'statement': str(statement or '').strip(),
        'simulated': True,
    }


def fail_once_mode(mode: str, *, fail_once_active: bool, failure_mode: str) -> str:
    if mode != 'fail_once':
        return mode
    return 'success' if fail_once_active else failure_mode


def manipulation_failure(*, code: str, message: str, recoverable: bool, recovery: str) -> dict:
    return failure_block(
        code=code,
        message=message,
        recoverable=recoverable,
        suggested_recovery=recovery,
    )
