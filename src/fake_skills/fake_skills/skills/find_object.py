"""Fake find_object skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block

SUCCESS_MODES = {'found', 'success'}


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    target = str(args.get('target', '')).strip() or 'object'
    target_kind = str(args.get('target_kind', 'object')).strip() or 'object'
    evidence_policy = str(args.get('evidence_policy', 'fresh_required')).strip() or 'fresh_required'
    metadata = dict(metadata or {})
    metadata.setdefault('evidence_policy', evidence_policy)
    metadata.setdefault('requires_fresh_scan', evidence_policy in {'fresh_required', 'force_fresh'})

    if mode == 'fail_once' and not fail_once_active:
        mode = 'not_found'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'found'

    if mode in SUCCESS_MODES:
        return build_skill_result(
            skill='find_object',
            status='succeeded',
            target=target,
            target_kind=target_kind,
            target_found=True,
            summary_text='I found one %s.' % target,
            evidence={
                'objects': [
                    {
                        'id': 'fake_%s_1' % target.replace(' ', '_'),
                        'label': target,
                        'confidence': 0.91,
                        'source': 'fake_find_object',
                        'evidence_policy': evidence_policy,
                    }
                ]
            },
            metadata=metadata,
        ).to_dict()

    if mode == 'ambiguous':
        return build_skill_result(
            skill='find_object',
            status='failed',
            target=target,
            target_kind=target_kind,
            target_found=False,
            summary_text='I found multiple candidates for %s.' % target,
            evidence={
                'objects': [
                    {'id': 'fake_%s_1' % target.replace(' ', '_'), 'label': target, 'confidence': 0.72},
                    {'id': 'fake_%s_2' % target.replace(' ', '_'), 'label': target, 'confidence': 0.69},
                ]
            },
            failure=failure_block(
                code='ambiguous',
                message='Multiple candidates matched the target.',
                recoverable=True,
                suggested_recovery='ask_user_to_disambiguate',
            ),
            metadata=metadata,
        ).to_dict()

    if mode == 'backend_unavailable':
        return build_skill_result(
            skill='find_object',
            status='failed',
            target=target,
            target_kind=target_kind,
            target_found=False,
            summary_text='I could not check %s right now.' % target,
            evidence={'objects': []},
            failure=failure_block(
                code='backend_unavailable',
                message='Fake find-object backend is unavailable.',
                recoverable=True,
                suggested_recovery='retry_find_object',
            ),
            metadata=metadata,
        ).to_dict()

    if mode == 'always_fail':
        return build_skill_result(
            skill='find_object',
            status='failed',
            target=target,
            target_kind=target_kind,
            target_found=False,
            summary_text='I could not find %s.' % target,
            evidence={'objects': []},
            failure=failure_block(
                code='always_fail',
                message='This scenario is configured to always fail.',
                recoverable=False,
                suggested_recovery='replan',
            ),
            metadata=metadata,
        ).to_dict()

    if mode == 'unexpected_payload':
        return {
            'skill': 'find_object',
            'status': 'failed',
            'target': target,
            'target_kind': target_kind,
            'target_found': False,
            'summary_text': 'Unexpected payload from fake find-object backend.',
            'evidence': {'raw': ['unexpected', 'payload']},
            'failure': {'code': 'unexpected_payload', 'recoverable': False},
            'metadata': metadata,
        }

    return build_skill_result(
        skill='find_object',
        status='failed',
        target=target,
        target_kind=target_kind,
        target_found=False,
        summary_text='I could not find %s.' % target,
        evidence={'objects': []},
        failure=failure_block(
            code='not_found',
            message='No matching object was detected in the current evidence snapshot.',
            recoverable=True,
            suggested_recovery='scan_then_retry',
        ),
        metadata=metadata,
    ).to_dict()
