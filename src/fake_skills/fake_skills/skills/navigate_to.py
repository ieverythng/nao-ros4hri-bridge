"""Fake navigate_to skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


SUCCESS_MODES = {'success'}
FAILURE_RECOVERABLE = {
    'path_blocked': ('The simulated path to the target is blocked.', 'ask_user_for_alternative_route'),
    'unknown_location': ('The target location is unknown in the simulated map.', 'ask_user_to_specify_location'),
    'timeout': ('The simulated navigation timed out before completion.', 'retry_navigation'),
}
FAILURE_FATAL = {
    'safety_disabled': 'Navigation safety gate is disabled.',
    'always_fail': 'This scenario is configured to always fail.',
    'unexpected_payload': 'Navigation backend returned an unexpected payload.',
}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = str(args.get('target', args.get('location', ''))).strip()
    target_kind = 'location'

    if mode == 'fail_once' and not fail_once_active:
        mode = 'path_blocked'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'success'

    if mode in SUCCESS_MODES:
        return build_skill_result(
            skill='navigate_to',
            status='succeeded',
            target=target,
            target_kind=target_kind,
            target_found=True,
            summary_text='I navigated to %s.' % (target or 'the requested location'),
            evidence={'location': target or '', 'simulated': True},
            metadata=metadata,
        ).to_dict()

    if mode in FAILURE_RECOVERABLE:
        message, recovery = FAILURE_RECOVERABLE[mode]
        return build_skill_result(
            skill='navigate_to',
            status='failed',
            target=target,
            target_kind=target_kind,
            target_found=False,
            summary_text='I could not navigate to %s.' % (target or 'that location'),
            evidence={'location': target or '', 'simulated': True},
            failure=failure_block(
                code=mode,
                message=message,
                recoverable=True,
                suggested_recovery=recovery,
            ),
            metadata=metadata,
        ).to_dict()

    message = FAILURE_FATAL.get(mode, 'Unknown navigation failure mode.')
    return build_skill_result(
        skill='navigate_to',
        status='failed',
        target=target,
        target_kind=target_kind,
        target_found=False,
        summary_text='I could not navigate to %s.' % (target or 'that location'),
        evidence={'location': target or '', 'simulated': True},
        failure=failure_block(
            code=mode or 'unknown_failure',
            message=message,
            recoverable=False,
            suggested_recovery='replan',
        ),
        metadata=metadata,
    ).to_dict()
