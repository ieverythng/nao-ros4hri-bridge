"""Fake perform_motion skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


_SUPPORTED_MOTIONS = {
    'stand',
    'standinit',
    'sit',
    'kneel',
    'crouch',
    'head_center',
    'head_look_left',
    'head_look_right',
    'head_look_up',
    'head_look_down',
    'look_at_reset',
}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    motion = str(args.get('object', args.get('motion', args.get('target', '')))).strip().lower()
    if mode == 'fail_once' and not fail_once_active:
        mode = 'convergence_timeout'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'success'

    if motion not in _SUPPORTED_MOTIONS:
        return build_skill_result(
            skill='perform_motion',
            status='failed',
            target=motion,
            target_kind='motion',
            target_found=False,
            summary_text='I could not perform the requested motion.',
            evidence={'motion': motion, 'simulated': True},
            failure=failure_block(
                code='unsupported_motion',
                message='The requested fake motion is not supported.',
                recoverable=False,
                suggested_recovery='replan',
            ),
            metadata=metadata,
        ).to_dict()

    if mode in {'success', 'dry_run'}:
        evidence = {
            'motion': motion,
            'yaw': args.get('yaw', ''),
            'pitch': args.get('pitch', ''),
            'relative': bool(args.get('relative', False)),
            'simulated': True,
            'dry_run': mode == 'dry_run',
        }
        if 'previous_posture_state' in args:
            previous_state = str(args.get('previous_posture_state', 'unknown')).strip() or 'unknown'
            evidence.update(
                {
                    'previous_posture_state': previous_state,
                    'current_posture_state': motion,
                    'state_changed': previous_state != motion,
                }
            )
        return build_skill_result(
            skill='perform_motion',
            status='succeeded',
            target=motion,
            target_kind='motion',
            target_found=True,
            summary_text=_success_summary(motion, dry_run=(mode == 'dry_run')),
            evidence=evidence,
            metadata=metadata,
        ).to_dict()

    if mode in {'motion_unavailable', 'convergence_timeout'}:
        return build_skill_result(
            skill='perform_motion',
            status='failed',
            target=motion,
            target_kind='motion',
            target_found=False,
            summary_text='I could not complete the %s motion.' % motion,
            evidence={'motion': motion, 'simulated': True},
            failure=failure_block(
                code=mode,
                message=_failure_message(mode),
                recoverable=True,
                suggested_recovery='retry_motion',
            ),
            metadata=metadata,
        ).to_dict()

    return build_skill_result(
        skill='perform_motion',
        status='failed',
        target=motion,
        target_kind='motion',
        target_found=False,
        summary_text='I could not complete the requested motion.',
        evidence={'motion': motion, 'simulated': True},
        failure=failure_block(
            code=mode or 'unexpected_payload',
            message='Unexpected fake perform_motion response mode.',
            recoverable=False,
            suggested_recovery='replan',
        ),
        metadata=metadata,
    ).to_dict()


def _success_summary(motion: str, *, dry_run: bool) -> str:
    prefix = 'Dry run: I would perform' if dry_run else 'I performed'
    labels = {
        'head_center': 'a head-centering motion',
        'head_look_left': 'a head look-left motion',
        'head_look_right': 'a head look-right motion',
        'head_look_up': 'a head look-up motion',
        'head_look_down': 'a head look-down motion',
        'look_at_reset': 'a gaze reset motion',
        'standinit': 'a stand-init posture motion',
    }
    return '%s %s.' % (prefix, labels.get(motion, 'the %s motion' % motion))


def _failure_message(mode: str) -> str:
    if mode == 'convergence_timeout':
        return 'The fake motion did not reach the target before timeout.'
    return 'The fake motion backend is unavailable.'
