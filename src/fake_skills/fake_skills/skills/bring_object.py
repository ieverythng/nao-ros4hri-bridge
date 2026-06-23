"""Fake bring skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.skills.manipulation_common import clean_arg
from fake_skills.skills.manipulation_common import fail_once_mode
from fake_skills.skills.manipulation_common import kb_effect
from fake_skills.skills.manipulation_common import manipulation_failure


FAILURES = {
    'acquisition_failure': (
        'The robot could not acquire the requested object.',
        True,
        'retry_pick_or_ask_user',
    ),
    'navigation_failure': (
        'The robot could not navigate to the delivery target.',
        True,
        'ask_user_for_alternative_route',
    ),
    'delivery_blocked': (
        'The delivery destination is blocked.',
        True,
        'ask_user_for_delivery_alternative',
    ),
    'recipient_unavailable': (
        'The requested recipient is unavailable.',
        True,
        'ask_user_to_identify_recipient',
    ),
    'always_fail': (
        'This scenario is configured to always fail.',
        False,
        'replan',
    ),
}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = clean_arg(args, 'object_id', 'target', 'object', default='object')
    recipient = clean_arg(
        args,
        'recipient_id',
        'recipient',
        'person',
        'destination',
        default='recipient',
    )
    source = clean_arg(args, 'source', 'support', 'from', default='unknown_support')
    mode = fail_once_mode(
        mode,
        fail_once_active=fail_once_active,
        failure_mode='acquisition_failure',
    )

    if mode in {'success', 'delivered'}:
        return build_skill_result(
            skill='bring_object',
            status='succeeded',
            target=target,
            target_kind='object',
            target_found=True,
            summary_text='I brought %s to %s.' % (target, recipient),
            evidence={
                'object_id': target,
                'recipient_id': recipient,
                'source': source,
                'chain': [
                    'find_object',
                    'pick_object',
                    'navigate_to',
                    'place_object',
                ],
                'kb_effects': [
                    kb_effect('remove', '%s oro:isOn %s' % (target, source)),
                    kb_effect('add', '%s oro:isAt %s' % (target, recipient)),
                    kb_effect('remove', 'robot oro:holds %s' % target),
                ],
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    message, recoverable, recovery = FAILURES.get(
        mode,
        ('Unexpected fake bring-object response mode.', False, 'replan'),
    )
    return build_skill_result(
        skill='bring_object',
        status='failed',
        target=target,
        target_kind='object',
        target_found=mode not in {'acquisition_failure', 'recipient_unavailable'},
        summary_text='I could not bring %s to %s.' % (target, recipient),
        evidence={
            'object_id': target,
            'recipient_id': recipient,
            'source': source,
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
