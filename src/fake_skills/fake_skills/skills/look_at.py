"""Fake look_at skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = str(args.get('target_frame', args.get('target', args.get('policy', '')))).strip()
    if mode == 'fail_once':
        mode = 'success' if fail_once_active else 'target_unavailable'

    if mode in {'success', 'dry_run'}:
        return build_skill_result(
            skill='look_at',
            status='succeeded',
            target=target,
            target_kind='frame',
            target_found=bool(target),
            summary_text='I looked at %s.' % (target or 'the requested target'),
            evidence={'target_frame': target, 'simulated': True, 'dry_run': mode == 'dry_run'},
            metadata=metadata,
        ).to_dict()

    return build_skill_result(
        skill='look_at',
        status='failed',
        target=target,
        target_kind='frame',
        target_found=False,
        summary_text='I could not look at the requested target.',
        evidence={'target_frame': target, 'simulated': True},
        failure=failure_block(
            code=mode or 'target_unavailable',
            message='The fake look-at target is unavailable.',
            recoverable=True,
            suggested_recovery='replan',
        ),
        metadata=metadata,
    ).to_dict()
