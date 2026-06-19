import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'summarize_validation_traces.py'
SPEC = importlib.util.spec_from_file_location('summarize_validation_traces', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_summarize_events_reports_completion_and_duration() -> None:
    metrics = MODULE.summarize_events(
        [
            {'trace_id': 'goal_1', 'timestamp': 10.0, 'event_type': 'planner_output'},
            {
                'trace_id': 'goal_1',
                'timestamp': 12.5,
                'event_type': 'execution_feedback',
                'channel': '/planner/execution_feedback',
                'payload': {'status': 'completed'},
            },
            {'trace_id': 'goal_1', 'timestamp': 13.0, 'event_type': 'planner_dialogue_act'},
        ]
    )

    assert len(metrics) == 1
    assert metrics[0].trace_id == 'goal_1'
    assert metrics[0].duration_sec == 3.0
    assert metrics[0].completed is True
    assert metrics[0].dialogue_act_count == 1


def test_nested_plan_goal_id_takes_precedence_over_viewer_trace_id() -> None:
    metrics = MODULE.summarize_events(
        [
            {
                'trace_id': 'trace_0001',
                'timestamp': 10.0,
                'event_type': 'planner_output',
                'payload': {'data': {'plan': {'goal_id': 'goal_1'}}},
            },
            {
                'trace_id': 'goal_1',
                'timestamp': 11.0,
                'channel': '/planner/execution_feedback',
                'payload': {'goal_id': 'goal_1', 'status': 'completed'},
            },
        ]
    )

    assert len(metrics) == 1
    assert metrics[0].trace_id == 'goal_1'
    assert metrics[0].planner_output_count == 1
