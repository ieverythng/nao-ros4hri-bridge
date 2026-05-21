"""Fake inspect_area skill behavior."""

from __future__ import annotations

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


def execute(*, args: dict, mode: str, metadata: dict, fail_once_active: bool) -> dict:
    area = str(args.get('target', args.get('target_area', 'area'))).strip() or 'area'

    if mode == 'fail_once' and not fail_once_active:
        mode = 'backend_unavailable'
    elif mode == 'fail_once' and fail_once_active:
        mode = 'clear'

    if mode == 'clear':
        return build_skill_result(
            skill='inspect_area',
            status='succeeded',
            target=area,
            target_kind='area',
            target_found=True,
            summary_text='The %s area appears clear.' % area,
            evidence={'area': area, 'objects': [], 'people': [], 'simulated': True},
            metadata=metadata,
        ).to_dict()

    if mode == 'object_found':
        return build_skill_result(
            skill='inspect_area',
            status='succeeded',
            target=area,
            target_kind='area',
            target_found=True,
            summary_text='I detected an object near %s.' % area,
            evidence={
                'area': area,
                'objects': [{'id': 'fake_object_1', 'label': 'object', 'confidence': 0.79}],
                'people': [],
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    if mode == 'person_found':
        return build_skill_result(
            skill='inspect_area',
            status='succeeded',
            target=area,
            target_kind='area',
            target_found=True,
            summary_text='I detected a person near %s.' % area,
            evidence={
                'area': area,
                'objects': [],
                'people': [{'id': 'fake_person_1', 'label': 'person', 'confidence': 0.88}],
                'simulated': True,
            },
            metadata=metadata,
        ).to_dict()

    if mode == 'ambiguous':
        return build_skill_result(
            skill='inspect_area',
            status='failed',
            target=area,
            target_kind='area',
            target_found=False,
            summary_text='I found ambiguous evidence in %s.' % area,
            evidence={
                'area': area,
                'objects': [{'id': 'fake_candidate_1', 'label': 'unknown', 'confidence': 0.55}],
                'people': [],
                'simulated': True,
            },
            failure=failure_block(
                code='ambiguous',
                message='The inspected area produced ambiguous evidence.',
                recoverable=True,
                suggested_recovery='ask_user_to_disambiguate',
            ),
            metadata=metadata,
        ).to_dict()

    if mode == 'area_unknown':
        return build_skill_result(
            skill='inspect_area',
            status='failed',
            target=area,
            target_kind='area',
            target_found=False,
            summary_text='I do not know where %s is.' % area,
            evidence={'area': area, 'objects': [], 'people': [], 'simulated': True},
            failure=failure_block(
                code='area_unknown',
                message='The requested area is unknown.',
                recoverable=True,
                suggested_recovery='ask_user_to_specify_location',
            ),
            metadata=metadata,
        ).to_dict()

    return build_skill_result(
        skill='inspect_area',
        status='failed',
        target=area,
        target_kind='area',
        target_found=False,
        summary_text='I could not inspect %s.' % area,
        evidence={'area': area, 'objects': [], 'people': [], 'simulated': True},
        failure=failure_block(
            code=mode or 'backend_unavailable',
            message='Inspect-area backend is unavailable.',
            recoverable=True,
            suggested_recovery='retry_inspect_area',
        ),
        metadata=metadata,
    ).to_dict()
