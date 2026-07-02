"""Motion execution evidence helpers for nao_orchestrator.

`classify_motion_target` remains canonical in `intent_rules.py`; this module
focuses on the execution-side evidence and dispatch wrapper built on top of that
classification.
"""

from __future__ import annotations

from typing import Any, Callable

from hri_actions_msgs.msg import Intent as IntentMsg

from nao_orchestrator.intent_rules import classify_motion_target


def _first_non_empty_text(*values) -> str:
    for value in values:
        text = str(value or '').strip()
        if text:
            return text
    return ''


def _first_non_empty_value(data: dict, *keys: str) -> str:
    if not isinstance(data, dict):
        return ''
    return _first_non_empty_text(*(data.get(key, '') for key in keys))


def build_motion_summary(route: str, motion_label: str) -> str:
    clean_label = str(motion_label or '').strip().lower().replace('_', ' ')
    if clean_label.startswith('head look '):
        direction = clean_label.removeprefix('head look ').strip()
        return 'I moved my head %s.' % direction
    if clean_label == 'head center':
        return 'I centered my head.'
    if str(route or '').strip().lower() == 'look_at_reset':
        return 'I reset my gaze.'
    if clean_label:
        return 'I performed %s.' % clean_label
    return 'I performed the requested motion.'


def collect_motion_evidence(route: str, step_args: dict, resolved_payload: dict) -> dict:
    """Build non-spoken execution evidence for successful motion steps."""
    motion_label = _first_non_empty_value(
        resolved_payload,
        'motion_name',
        'motion',
        'name',
        'target',
        'policy',
    )
    if not motion_label:
        motion_label = _first_non_empty_value(
            step_args,
            'motion',
            'name',
            'target',
            'object',
            'policy',
        )
    clean_label = str(motion_label or route or 'motion').strip()
    summary_text = build_motion_summary(route, clean_label)
    return {
        'skill': 'perform_motion',
        'route': str(route or '').strip(),
        'motion': clean_label,
        'status': 'succeeded',
        'summary_text': summary_text,
        'metadata': {
            'speech_produced': False,
        },
    }


def execute_motion_plan_step(
    step_args: dict,
    *,
    perform_motion_execution_mode: str,
    execute_fake_skill_step: Callable[..., tuple[bool, str, dict]],
    execute_replay_motion_step: Callable[..., tuple[bool, str]],
    execute_head_motion_step: Callable[..., tuple[bool, str]],
    execute_look_at_reset_step: Callable[..., tuple[bool, str]],
    stats: Any,
    logger,
    on_started=None,
) -> tuple[bool, str, dict]:
    if perform_motion_execution_mode == 'fake':
        success, reason, payload = execute_fake_skill_step(
            'perform_motion',
            dict(step_args or {}),
            on_started=on_started,
        )
        if success:
            return True, reason, payload
        return False, reason or 'fake perform_motion dispatch failed', payload

    route, resolved_payload = classify_motion_target(IntentMsg.PERFORM_MOTION, step_args)
    if route == 'replay_motion':
        motion_name = resolved_payload['motion_name']
        success, reason = execute_replay_motion_step(
            motion_name,
            on_started=on_started,
        )
        if success:
            stats.dispatched_replay_motion += 1
            payload = collect_motion_evidence(route, step_args, resolved_payload)
            return True, payload['summary_text'], payload
        return False, reason or 'motion dispatch failed', {}

    if route == 'head_motion':
        success, reason = execute_head_motion_step(
            resolved_payload,
            on_started=on_started,
        )
        if success:
            stats.dispatched_head_motion += 1
            payload = collect_motion_evidence(route, step_args, resolved_payload)
            return True, payload['summary_text'], payload
        return False, reason or 'motion dispatch failed', {}

    if route == 'look_at_reset':
        success, reason = execute_look_at_reset_step(on_started=on_started)
        if success:
            stats.dispatched_look_at += 1
            payload = collect_motion_evidence(route, step_args, resolved_payload)
            return True, payload['summary_text'], payload
        return False, reason or 'look_at reset dispatch failed', {}

    stats.dispatch_failures += 1
    logger.warn('Unsupported motion payload: %s' % step_args)
    return False, 'unsupported motion payload', {}
