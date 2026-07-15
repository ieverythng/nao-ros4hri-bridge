"""Fake pick/grab skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.skills.manipulation_common import clean_arg
from fake_skills.skills.manipulation_common import fail_once_mode
from fake_skills.skills.manipulation_common import kb_effect
from fake_skills.skills.manipulation_common import manipulation_failure


FAILURES = {
    'object_unavailable': (
        'The requested object is not available in the simulated scene.',
        True,
        'scan_then_retry',
    ),
    'unreachable': (
        'The requested object is outside the simulated reachable workspace.',
        True,
        'ask_user_to_move_object',
    ),
    'already_held': (
        'The robot is already holding an object in this simulated state.',
        True,
        'place_current_object_first',
    ),
    'grasp_failed': (
        'The simulated grasp did not stabilize.',
        True,
        'retry_pick',
    ),
    'always_fail': (
        'This scenario is configured to always fail.',
        False,
        'replan',
    ),
}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = clean_arg(args, 'object_id', 'target', 'object', default='object')
    support = clean_arg(args, 'support', 'from', 'source', default='unknown_support')
    gripper = clean_arg(args, 'gripper', 'hand', default='right_hand')
    mode = fail_once_mode(
        mode,
        fail_once_active=fail_once_active,
        failure_mode='object_unavailable',
    )

    if mode in {'success', 'picked', 'grabbed'}:
        return build_skill_result(
            skill='pick_object',
            status='succeeded',
            target=target,
            target_kind='object',
            target_found=True,
            summary_text='I picked up %s.' % target,
            evidence={
                'object_id': target,
                'gripper': gripper,
                'previous_support': support,
                'held': True,
                'kb_effects': [
                    kb_effect('remove', '%s oro:isOn %s' % (target, support)),
                    kb_effect('add', 'robot oro:holds %s' % target),
                ],
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    message, recoverable, recovery = FAILURES.get(
        mode,
        ('Unexpected fake pick-object response mode.', False, 'replan'),
    )
    return build_skill_result(
        skill='pick_object',
        status='failed',
        target=target,
        target_kind='object',
        target_found=mode not in {'object_unavailable', 'not_found'},
        summary_text='I could not pick up %s.' % target,
        evidence={
            'object_id': target,
            'gripper': gripper,
            'previous_support': support,
            'held': False,
            'simulated': True,
        },
        failure=manipulation_failure(
            code=mode or 'unexpected_payload',
            message=message,
            recoverable=recoverable,
            recovery=recovery,
        ),
        metadata=metadata,
    ).to_dict()
