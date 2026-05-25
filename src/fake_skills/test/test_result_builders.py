from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block


def test_build_skill_result_preserves_contract_shape() -> None:
    payload = build_skill_result(
        skill='navigate_to',
        status='failed',
        target='kitchen',
        target_kind='location',
        target_found=False,
        summary_text='I could not navigate to the kitchen.',
        evidence={'location': 'kitchen'},
        failure=failure_block(
            code='path_blocked',
            message='The path is blocked.',
            recoverable=True,
            suggested_recovery='ask_user_for_alternative_route',
        ),
        metadata={'fake': True},
    ).to_dict()

    assert payload['skill'] == 'navigate_to'
    assert payload['status'] == 'failed'
    assert payload['target_kind'] == 'location'
    assert payload['failure']['code'] == 'path_blocked'
    assert payload['metadata']['fake'] is True
