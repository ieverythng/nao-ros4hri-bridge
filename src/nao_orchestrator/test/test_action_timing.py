from nao_orchestrator.action_timing import ActionDeadline


def test_action_deadline_allows_goal_acceptance_for_the_result_budget() -> None:
    deadline = ActionDeadline.start(8.0, clock=lambda: 10.0)

    assert deadline.remaining(clock=lambda: 10.0) == 8.0
    assert deadline.remaining(clock=lambda: 11.25) == 6.75


def test_action_deadline_never_returns_a_non_positive_wait() -> None:
    deadline = ActionDeadline.start(0.01, clock=lambda: 10.0)

    assert deadline.remaining(clock=lambda: 20.0) == 0.1
