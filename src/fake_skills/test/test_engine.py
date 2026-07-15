from fake_skills.engine import FakeSkillEngine
from fake_skills.scenario_store import ScenarioStore


def _engine() -> FakeSkillEngine:
    return FakeSkillEngine(
        scenario_store=ScenarioStore(
            {
                'default': {
                    'navigate_to': {'result_mode': 'success', 'delay_sec': 0.0},
                    'find_object': {'result_mode': 'found', 'delay_sec': 0.0},
                    'perform_motion': {'result_mode': 'success', 'delay_sec': 0.0},
                    'pick_object': {'result_mode': 'success', 'delay_sec': 0.0},
                    'place_object': {'result_mode': 'success', 'delay_sec': 0.0},
                    'bring_object': {'result_mode': 'success', 'delay_sec': 0.0},
                },
                'scenarios': {
                    'blocked': {'navigate_to': {'result_mode': 'path_blocked'}},
                    'motion_timeout': {
                        'perform_motion': {'result_mode': 'convergence_timeout'}
                    },
                    'pick_unreachable': {'pick_object': {'result_mode': 'unreachable'}},
                    'place_no_held': {'place_object': {'result_mode': 'no_held_object'}},
                    'bring_blocked': {'bring_object': {'result_mode': 'delivery_blocked'}},
                    'bring_no_recipient': {
                        'bring_object': {'result_mode': 'recipient_unavailable'}
                    },
                },
            }
        ),
        default_delay_sec=0.0,
        deterministic_seed=7,
    )


def _engine_with_policy(**kwargs) -> FakeSkillEngine:
    return FakeSkillEngine(
        scenario_store=ScenarioStore(
            {
                'default': {
                    'navigate_to': {'result_mode': 'success', 'delay_sec': 0.0},
                    'find_object': {'result_mode': 'found', 'delay_sec': 0.0},
                    'perform_motion': {'result_mode': 'success', 'delay_sec': 0.0},
                    'pick_object': {'result_mode': 'success', 'delay_sec': 0.0},
                    'place_object': {'result_mode': 'success', 'delay_sec': 0.0},
                    'bring_object': {'result_mode': 'success', 'delay_sec': 0.0},
                },
                'scenarios': {
                    'blocked': {'navigate_to': {'result_mode': 'path_blocked'}},
                    'motion_timeout': {
                        'perform_motion': {'result_mode': 'convergence_timeout'}
                    },
                    'pick_unreachable': {'pick_object': {'result_mode': 'unreachable'}},
                    'place_no_held': {'place_object': {'result_mode': 'no_held_object'}},
                    'bring_blocked': {'bring_object': {'result_mode': 'delivery_blocked'}},
                    'bring_no_recipient': {
                        'bring_object': {'result_mode': 'recipient_unavailable'}
                    },
                },
            }
        ),
        default_delay_sec=0.0,
        deterministic_seed=7,
        **kwargs,
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


def test_engine_fail_once_tracks_object_across_argument_aliases() -> None:
    engine = _engine()

    first, _ = engine.execute(
        skill='pick_object',
        args={'object_id': 'cup', 'result_mode': 'fail_once'},
    )
    second, _ = engine.execute(
        skill='pick_object',
        args={'target': 'cup', 'result_mode': 'fail_once'},
    )

    assert first['status'] == 'failed'
    assert second['status'] == 'succeeded'


def test_engine_policy_fail_once_applies_once_per_skill_across_targets() -> None:
    engine = _engine_with_policy(mode_overrides={'navigate_to': 'fail_once'})

    first, _ = engine.execute(skill='navigate_to', args={'target': 'apple'})
    second, _ = engine.execute(skill='navigate_to', args={'target': 'book'})

    assert first['status'] == 'failed'
    assert second['status'] == 'succeeded'


def test_engine_policy_change_starts_a_fresh_failure_sequence() -> None:
    engine = _engine_with_policy(global_mode='always_success')
    engine.execute(skill='navigate_to', args={'target': 'apple'})

    engine.update_policy(
        global_mode='scenario',
        random_failure_prob=0.5,
        mode_overrides={'navigate_to': 'fail_once'},
    )
    first_after_change, _ = engine.execute(
        skill='navigate_to',
        args={'target': 'apple'},
    )

    assert first_after_change['status'] == 'failed'


def test_engine_find_object_treats_success_mode_as_found() -> None:
    payload, _delay = _engine().execute(
        skill='find_object',
        args={'target': 'cup', 'result_mode': 'success'},
    )

    assert payload['status'] == 'succeeded'
    assert payload['target_found'] is True


def test_engine_perform_motion_success_payload_is_traceable() -> None:
    payload, delay_sec = _engine().execute(
        skill='perform_motion',
        args={'object': 'head_look_left'},
    )

    assert delay_sec == 0.0
    assert payload['status'] == 'succeeded'
    assert payload['skill'] == 'perform_motion'
    assert payload['target'] == 'head_look_left'
    assert payload['metadata']['fake'] is True
    assert payload['metadata']['result_mode'] == 'success'
    assert 'head look-left' in payload['summary_text']


def test_engine_perform_motion_scenario_timeout_fails_deterministically() -> None:
    payload, _delay = _engine().execute(
        skill='perform_motion',
        args={'object': 'head_look_left'},
        scenario_id='motion_timeout',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'convergence_timeout'
    assert payload['metadata']['result_mode'] == 'convergence_timeout'


def test_engine_perform_motion_fail_once_switches_to_success_on_second_call() -> None:
    engine = _engine()

    first, _ = engine.execute(
        skill='perform_motion',
        args={'object': 'head_look_left', 'result_mode': 'fail_once'},
    )
    second, _ = engine.execute(
        skill='perform_motion',
        args={'object': 'head_look_left', 'result_mode': 'fail_once'},
    )

    assert first['status'] == 'failed'
    assert first['failure']['code'] == 'convergence_timeout'
    assert second['status'] == 'succeeded'


def test_engine_remembers_successful_fake_posture_state() -> None:
    engine = _engine()

    stand, _ = engine.execute(skill='perform_motion', args={'object': 'stand'})
    sit, _ = engine.execute(skill='perform_motion', args={'object': 'sit'})

    assert stand['evidence']['previous_posture_state'] == 'unknown'
    assert stand['evidence']['current_posture_state'] == 'stand'
    assert sit['evidence']['previous_posture_state'] == 'stand'
    assert sit['evidence']['current_posture_state'] == 'sit'
    assert engine.posture_state == 'sit'


def test_engine_failed_fake_posture_does_not_change_state() -> None:
    engine = _engine()
    engine.execute(skill='perform_motion', args={'object': 'stand'})

    failed, _ = engine.execute(
        skill='perform_motion',
        args={'object': 'sit', 'result_mode': 'motion_unavailable'},
    )

    assert failed['status'] == 'failed'
    assert engine.posture_state == 'stand'


def test_engine_head_motion_does_not_change_posture_state() -> None:
    engine = _engine()
    engine.execute(skill='perform_motion', args={'object': 'stand'})
    head, _ = engine.execute(skill='perform_motion', args={'object': 'head_look_left'})

    assert 'previous_posture_state' not in head['evidence']
    assert engine.posture_state == 'stand'


def test_engine_unknown_skill_returns_structured_failure() -> None:
    payload, delay_sec = _engine().execute(skill='dance', args={})

    assert delay_sec == 0.0
    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'unsupported_skill'
    assert payload['metadata']['result_mode'] == 'unsupported_skill'


def test_engine_global_every_other_alternates_for_same_request() -> None:
    engine = _engine_with_policy(global_mode='every_other')

    first, _ = engine.execute(skill='navigate_to', args={'target': 'kitchen'})
    second, _ = engine.execute(skill='navigate_to', args={'target': 'kitchen'})
    third, _ = engine.execute(skill='navigate_to', args={'target': 'kitchen'})

    assert first['status'] == 'succeeded'
    assert second['status'] == 'failed'
    assert third['status'] == 'succeeded'
    assert first['metadata']['mode_source'] == 'global_mode'


def test_engine_random_seeded_is_reproducible() -> None:
    first_engine = _engine_with_policy(
        global_mode='random_seeded',
        random_failure_prob=0.4,
    )
    second_engine = _engine_with_policy(
        global_mode='random_seeded',
        random_failure_prob=0.4,
    )

    first_sequence = [
        first_engine.execute(skill='navigate_to', args={'target': 'kitchen'})[0]['status']
        for _ in range(6)
    ]
    second_sequence = [
        second_engine.execute(skill='navigate_to', args={'target': 'kitchen'})[0]['status']
        for _ in range(6)
    ]

    assert first_sequence == second_sequence


def test_engine_mode_resolution_precedence() -> None:
    engine = _engine_with_policy(
        global_mode='always_success',
        mode_overrides={'find_object': 'always_fail'},
    )

    payload, _ = engine.execute(
        skill='find_object',
        args={'target': 'cup', 'result_mode': 'found'},
        scenario_id='blocked',
        scenario_override={'result_mode': 'ambiguous'},
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'ambiguous'
    assert payload['metadata']['mode_source'] == 'scenario_override'


def test_engine_skill_override_beats_global_mode() -> None:
    engine = _engine_with_policy(
        global_mode='always_success',
        mode_overrides={'find_object': 'always_fail'},
    )

    payload, _ = engine.execute(skill='find_object', args={'target': 'cup'})

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'not_found'
    assert payload['metadata']['mode_source'] == 'skill_override'


def test_engine_invalid_mode_override_is_ignored() -> None:
    engine = _engine_with_policy(mode_overrides={'find_object': ''})
    payload, _ = engine.execute(skill='find_object', args={'target': 'cup'})

    assert payload['status'] == 'succeeded'
    assert payload['metadata']['mode_source'] == 'scenario_default'


def test_engine_find_object_preserves_frame_qualified_spatial_evidence() -> None:
    payload, _ = _engine().execute(
        skill='find_object',
        args={
            'target': 'cup',
            'frame_id': 'base_link',
            'position': {'x': 0.7, 'y': 0.1, 'z': 0.6},
            'distance_m': 0.93,
        },
    )

    evidence = payload['evidence']['objects'][0]
    assert evidence['frame_id'] == 'base_link'
    assert evidence['position'] == {'x': 0.7, 'y': 0.1, 'z': 0.6}
    assert evidence['distance_m'] == 0.93


def test_engine_pick_object_success_reports_held_kb_effects() -> None:
    payload, _delay = _engine().execute(
        skill='pick_object',
        args={'target': 'cup_1', 'support': 'table_1'},
    )

    assert payload['status'] == 'succeeded'
    assert payload['skill'] == 'pick_object'
    assert payload['evidence']['held'] is True
    expected_effect = {
        'action': 'add',
        'statement': 'robot oro:holds cup_1',
        'simulated': True,
    }
    assert expected_effect in payload['evidence']['kb_effects']


def test_engine_pick_object_failure_is_recoverable() -> None:
    payload, _delay = _engine().execute(
        skill='pick_object',
        args={'target': 'cup_1'},
        scenario_id='pick_unreachable',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'unreachable'
    assert payload['failure']['recoverable'] is True


def test_engine_place_object_success_reports_support_relation() -> None:
    payload, _delay = _engine().execute(
        skill='place_object',
        args={'target': 'cup_1', 'destination': 'shelf_1'},
    )

    assert payload['status'] == 'succeeded'
    assert payload['skill'] == 'place_object'
    expected_effect = {
        'action': 'add',
        'statement': 'cup_1 oro:isOn shelf_1',
        'simulated': True,
    }
    assert expected_effect in payload['evidence']['kb_effects']


def test_engine_place_object_no_held_object_failure() -> None:
    payload, _delay = _engine().execute(
        skill='place_object',
        args={'target': 'cup_1', 'destination': 'shelf_1'},
        scenario_id='place_no_held',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'no_held_object'
    assert payload['failure']['suggested_recovery'] == 'pick_object_first'


def test_engine_bring_object_success_reports_delivery_chain() -> None:
    payload, _delay = _engine().execute(
        skill='bring_object',
        args={'target': 'book_1', 'recipient': 'person_1', 'source': 'table_1'},
    )

    assert payload['status'] == 'succeeded'
    assert payload['skill'] == 'bring_object'
    assert payload['evidence']['chain'] == [
        'find_object',
        'pick_object',
        'navigate_to',
        'place_object',
    ]
    expected_effect = {
        'action': 'add',
        'statement': 'book_1 oro:isAt person_1',
        'simulated': True,
    }
    assert expected_effect in payload['evidence']['kb_effects']
    for predicate in ('oro:isOn', 'oro:isAt', 'oro:isIn'):
        assert {
            'action': 'remove',
            'statement': 'book_1 %s table_1' % predicate,
            'simulated': True,
        } in payload['evidence']['kb_effects']


def test_engine_bring_object_delivery_blocked_failure() -> None:
    payload, _delay = _engine().execute(
        skill='bring_object',
        args={'target': 'book_1', 'recipient': 'person_1'},
        scenario_id='bring_blocked',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'delivery_blocked'
    assert payload['failure']['suggested_recovery'] == 'ask_user_for_delivery_alternative'


def test_engine_bring_object_recipient_unavailable_failure() -> None:
    payload, _delay = _engine().execute(
        skill='bring_object',
        args={'target': 'book_1', 'recipient': 'person_1'},
        scenario_id='bring_no_recipient',
    )

    assert payload['status'] == 'failed'
    assert payload['failure']['code'] == 'recipient_unavailable'
    assert payload['failure']['suggested_recovery'] == 'ask_user_to_identify_recipient'
