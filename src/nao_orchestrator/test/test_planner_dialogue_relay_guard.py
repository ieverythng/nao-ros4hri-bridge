"""Tests for planner dialogue relay deduplication."""

from nao_orchestrator.orchestrator import _planner_dialogue_act_signature


def test_planner_dialogue_signature_ignores_json_formatting() -> None:
    first = '{"goal_id":"goal_1","act":"notify_completion","context":{"status":"done"}}'
    second = (
        '{ "context": { "status": "done" }, '
        '"act": "notify_completion", "goal_id": "goal_1" }'
    )

    assert _planner_dialogue_act_signature(first) == _planner_dialogue_act_signature(second)


def test_planner_dialogue_signature_suppresses_terminal_wording_variants() -> None:
    first = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"notify_completion","text_hint":"Done."}'
    )
    second = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"notify_completion","text_hint":"I finished."}'
    )

    assert _planner_dialogue_act_signature(first) == _planner_dialogue_act_signature(second)


def test_planner_dialogue_signature_tolerates_malformed_plan_version() -> None:
    payload = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":"not-a-number",'
        '"act":"NOTIFY_COMPLETION"}'
    )

    assert '"act":"notify_completion"' in _planner_dialogue_act_signature(payload)
    assert '"plan_version":0' in _planner_dialogue_act_signature(payload)


def test_planner_dialogue_signature_allows_distinct_progress_events() -> None:
    first = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"progress_update","text_hint":"Scanning."}'
    )
    second = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"progress_update","text_hint":"Checking the table."}'
    )

    assert _planner_dialogue_act_signature(first) != _planner_dialogue_act_signature(second)


def test_planner_dialogue_signature_allows_distinct_clarification_slots() -> None:
    target = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"ask_clarification","slots_needed":["target"],"text_hint":"Which target?"}'
    )
    location = (
        '{"goal_id":"goal_1","plan_id":"plan_1","plan_version":1,'
        '"act":"ask_clarification","slots_needed":["location"],"text_hint":"Which location?"}'
    )

    assert _planner_dialogue_act_signature(target) != _planner_dialogue_act_signature(location)
