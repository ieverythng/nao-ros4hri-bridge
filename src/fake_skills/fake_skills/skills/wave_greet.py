"""Fake wave_greet skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    style = str(args.get('style', 'friendly')).strip() or 'friendly'
    hand = str(args.get('hand', 'right')).strip() or 'right'
    dry_run = bool(args.get('dry_run', True))

    if mode == 'fail_once' and not fail_once_active:
        mode = 'motion_unavailable'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'success'

    if mode == 'success':
        return build_skill_result(
            skill='wave_greet',
            status='succeeded',
            target='',
            target_kind='social_gesture',
            target_found=None,
            summary_text='I performed a %s wave.' % style,
            evidence={
                'gesture': 'wave',
                'style': style,
                'hand': hand,
                'dry_run': dry_run,
            },
            metadata=metadata,
        ).to_dict()

    if mode == 'motion_unavailable':
        return build_skill_result(
            skill='wave_greet',
            status='failed',
            target='',
            target_kind='social_gesture',
            target_found=None,
            summary_text='I could not perform the wave gesture.',
            evidence={'gesture': 'wave', 'style': style, 'dry_run': dry_run},
            failure=failure_block(
                code='motion_unavailable',
                message='The wave motion backend is unavailable.',
                recoverable=True,
                suggested_recovery='retry_wave',
            ),
            metadata=metadata,
        ).to_dict()

    if mode == 'safety_disabled':
        return build_skill_result(
            skill='wave_greet',
            status='failed',
            target='',
            target_kind='social_gesture',
            target_found=None,
            summary_text='I cannot wave because the safety gate is disabled.',
            evidence={'gesture': 'wave', 'style': style, 'dry_run': dry_run},
            failure=failure_block(
                code='safety_disabled',
                message='Gesture safety gate is disabled.',
                recoverable=False,
                suggested_recovery='require_operator_reset',
            ),
            metadata=metadata,
        ).to_dict()

    return build_skill_result(
        skill='wave_greet',
        status='failed',
        target='',
        target_kind='social_gesture',
        target_found=None,
        summary_text='I could not complete the wave gesture.',
        evidence={'gesture': 'wave', 'style': style, 'dry_run': dry_run},
        failure=failure_block(
            code=mode or 'unexpected_payload',
            message='Unexpected fake wave response mode.',
            recoverable=False,
            suggested_recovery='replan',
        ),
        metadata=metadata,
    ).to_dict()
