from fake_skills.engine import FakeSkillEngine
from fake_skills.scenario_store import ScenarioStore


def _engine() -> FakeSkillEngine:
    return FakeSkillEngine(
        scenario_store=ScenarioStore(
            {
                'default': {
                    'navigate_to': {'result_mode': 'success', 'delay_sec': 0.0},
                    'find_object': {'result_mode': 'found', 'delay_sec': 0.0},
                },
                'scenarios': {
                    'blocked': {'navigate_to': {'result_mode': 'path_blocked'}},
                },
            }
        ),
        default_delay_sec=0.0,
        deterministic_seed=7,
    )


def test_engine_navigate_success() -> None:
    payload, delay_sec = _engine().execute(skill='navigate_to', args={'target': 'kitchen'})

    assert delay_sec == 0.0
    assert payload['status'] == 'succeeded'
    assert payload['skill'] == 'navigate_to'


def test_engine_named_scenario_failure() -> None:
    payload, _delay = _engine().execute(
        skill='navigate_to',
        args={'target': 'kitchen'},
        scenario_id='blocked',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'path_blocked'


def test_engine_fail_once_switches_to_success_on_second_call() -> None:
    engine = _engine()

    first, _ = engine.execute(
        skill='find_object',
        args={'target': 'cup', 'result_mode': 'fail_once'},
    )
    second, _ = engine.execute(
        skill='find_object',
        args={'target': 'cup', 'result_mode': 'fail_once'},
    )

    assert first['status'] == 'failed'
    assert second['status'] == 'succeeded'


def test_engine_find_object_treats_success_mode_as_found() -> None:
    payload, _delay = _engine().execute(
        skill='find_object',
        args={'target': 'cup', 'result_mode': 'success'},
    )

    assert payload['status'] == 'succeeded'
    assert payload['target_found'] is True


def test_engine_unknown_skill_returns_structured_failure() -> None:
    payload, delay_sec = _engine().execute(skill='dance', args={})

    assert delay_sec == 0.0
    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'unsupported_skill'
    assert payload['metadata']['result_mode'] == 'unsupported_skill'
