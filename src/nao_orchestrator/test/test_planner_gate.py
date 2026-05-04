import json

from nao_orchestrator.planner_gate import PlannerGate


def _payload(goal_id: str, **overrides):
    payload = {
        'request_id': 'turn_1',
        'goal_id': goal_id,
        'request_kind': 'new_goal',
        'goal_text': 'stand up',
        'requested_plan': [],
    }
    payload.update(overrides)
    return payload


def test_planner_gate_accepts_first_new_goal() -> None:
    gate = PlannerGate()

    decision = gate.decide(_payload('goal_1'))

    assert decision.accepted is True
    assert decision.request.goal_id == 'goal_1'
    assert gate.active_goal_id == 'goal_1'


def test_planner_gate_rejects_duplicate_active_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(_payload('goal_1'))

    assert decision.accepted is False
    assert decision.reason == 'duplicate active planner goal'


def test_planner_gate_requires_supersede_for_second_new_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(_payload('goal_2'))

    assert decision.accepted is False
    assert 'supersede or cancel' in decision.reason


def test_planner_gate_accepts_superseding_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(_payload('goal_2', supersedes_goal_id='goal_1'))

    assert decision.accepted is True
    assert gate.active_goal_id == 'goal_2'


def test_planner_gate_accepts_matching_clarification_answer() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(
        _payload(
            'goal_1',
            request_kind='clarification_answer',
            goal_text='the red cup',
        )
    )

    assert decision.accepted is True


def test_planner_gate_rejects_unmatched_clarification_answer() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(_payload('goal_2', request_kind='clarification_answer'))

    assert decision.accepted is False
    assert 'does not match active' in decision.reason


def test_planner_gate_cancel_clears_matching_active_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    decision = gate.decide(_payload('goal_1', request_kind='cancel_request'))

    assert decision.accepted is True
    assert gate.active_goal_id == ''


def test_planner_gate_feedback_completion_clears_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    gate.observe_feedback(json.dumps({'goal_id': 'goal_1', 'event_type': 'plan_completed'}))

    assert gate.active_goal_id == ''


def test_planner_gate_dialogue_failure_clears_goal() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    gate.observe_dialogue_act(json.dumps({'goal_id': 'goal_1', 'act': 'explain_failure'}))

    assert gate.active_goal_id == ''


def test_planner_gate_clarification_keeps_goal_active() -> None:
    gate = PlannerGate()
    assert gate.decide(_payload('goal_1')).accepted is True

    gate.observe_dialogue_act(json.dumps({'goal_id': 'goal_1', 'act': 'ask_clarification'}))

    assert gate.active_goal_id == 'goal_1'
