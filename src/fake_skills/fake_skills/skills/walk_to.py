"""Fake walk_to skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    distance_m = float(args.get('distance_m', 0.5) or 0.5)
    direction = str(args.get('direction', 'forward')).strip() or 'forward'
    dry_run = bool(args.get('dry_run', True))

    if mode == 'fail_once' and not fail_once_active:
        mode = 'path_blocked'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'success'

    if mode in ('success', 'dry_run'):
        summary = (
            'Dry run: I would walk %.2f meters %s.' % (distance_m, direction)
            if dry_run
            else 'I walked %.2f meters %s.' % (distance_m, direction)
        )
        return build_skill_result(
            skill='walk_to',
            status='succeeded',
            target='%.2f_m_%s' % (distance_m, direction),
            target_kind='distance_direction',
            target_found=True,
            summary_text=summary,
            evidence={
                'distance_m': distance_m,
                'direction': direction,
                'dry_run': dry_run,
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    if mode == 'path_blocked':
        return build_skill_result(
            skill='walk_to',
            status='failed',
            target='%.2f_m_%s' % (distance_m, direction),
            target_kind='distance_direction',
            target_found=False,
            summary_text='I could not walk %.2f meters %s because the path is blocked.' % (distance_m, direction),
            evidence={'distance_m': distance_m, 'direction': direction, 'dry_run': dry_run, 'simulated': True},
            failure=failure_block(
                code='path_blocked',
                message='Simulated local path is blocked.',
                recoverable=True,
                suggested_recovery='ask_user_for_alternative_route',
            ),
            metadata=metadata,
        ).to_dict()

    return build_skill_result(
        skill='walk_to',
        status='failed',
        target='%.2f_m_%s' % (distance_m, direction),
        target_kind='distance_direction',
        target_found=False,
        summary_text='I could not execute walk_to.',
        evidence={'distance_m': distance_m, 'direction': direction, 'dry_run': dry_run, 'simulated': True},
        failure=failure_block(
            code=mode or 'safety_disabled',
            message='Walk skill is unavailable in this mode.',
            recoverable=False,
            suggested_recovery='replan',
        ),
        metadata=metadata,
    ).to_dict()
