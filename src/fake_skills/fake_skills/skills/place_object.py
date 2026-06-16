"""Fake place skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.skills.manipulation_common import clean_arg
from fake_skills.skills.manipulation_common import fail_once_mode
from fake_skills.skills.manipulation_common import kb_effect
from fake_skills.skills.manipulation_common import manipulation_failure


FAILURES = {
    'destination_unavailable': (
        'The requested placement destination is not available.',
        True,
        'ask_user_to_specify_destination',
    ),
    'invalid_support': (
        'The requested destination cannot support the object.',
        True,
        'ask_user_for_valid_support',
    ),
    'no_held_object': (
        'The robot is not holding the requested object.',
        True,
        'pick_object_first',
    ),
    'placement_failed': (
        'The simulated placement did not settle safely.',
        True,
        'retry_place',
    ),
    'always_fail': (
        'This scenario is configured to always fail.',
        False,
        'replan',
    ),
}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = clean_arg(args, 'object_id', 'target', 'object', default='object')
    destination = clean_arg(
        args,
        'destination_id',
        'destination',
        'support',
        'target_location',
        default='destination',
    )
    relation = clean_arg(args, 'relation', 'predicate', default='oro:isOn')
    mode = fail_once_mode(mode, fail_once_active=fail_once_active, failure_mode='no_held_object')

    if mode in {'success', 'placed'}:
        return build_skill_result(
            skill='place_object',
            status='succeeded',
            target=target,
            target_kind='object',
            target_found=True,
            summary_text='I placed %s on %s.' % (target, destination),
            evidence={
                'object_id': target,
                'destination_id': destination,
                'relation': relation,
                'held': False,
                'kb_effects': [
                    kb_effect('remove', 'robot oro:holds %s' % target),
                    kb_effect('add', '%s %s %s' % (target, relation, destination)),
                ],
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    message, recoverable, recovery = FAILURES.get(
        mode,
        ('Unexpected fake place-object response mode.', False, 'replan'),
    )
    return build_skill_result(
        skill='place_object',
        status='failed',
        target=target,
        target_kind='object',
        target_found=mode != 'destination_unavailable',
        summary_text='I could not place %s on %s.' % (target, destination),
        evidence={
            'object_id': target,
            'destination_id': destination,
            'relation': relation,
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
