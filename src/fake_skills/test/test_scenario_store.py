from fake_skills.scenario_store import ScenarioStore


def test_scenario_store_merges_default_and_named_scenario() -> None:
    store = ScenarioStore(
        {
            'default': {'navigate_to': {'result_mode': 'success', 'delay_sec': 1.0}},
            'scenarios': {'path_blocked': {'navigate_to': {'result_mode': 'path_blocked'}}},
        }
    )

    config = store.resolve_skill_config(skill='navigate_to', scenario_id='path_blocked')

    assert config['result_mode'] == 'path_blocked'
    assert config['delay_sec'] == 1.0


def test_scenario_store_applies_request_override() -> None:
    store = ScenarioStore({'default': {'find_object': {'result_mode': 'found'}}})

    config = store.resolve_skill_config(
        skill='find_object',
        scenario_override={'result_mode': 'ambiguous'},
    )

    assert config['result_mode'] == 'ambiguous'
