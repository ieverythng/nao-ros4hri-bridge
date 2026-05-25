from interaction_trace_viewer.trace_model import InteractionEvent
from interaction_trace_viewer.trace_model import TraceRecorder


def test_trace_recorder_starts_trace_on_user_utterance() -> None:
    recorder = TraceRecorder()

    first = recorder.add(
        InteractionEvent(
            timestamp=1.0,
            trace_id=None,
            source_node='',
            channel='/humans/voices/alice/speech',
            event_type='user_utterance',
            ab_object_id=None,
            ab_level=0,
            summary='hello',
            payload={'text': 'hello'},
            raw='hello',
        )
    )
    second = recorder.add(
        InteractionEvent(
            timestamp=2.0,
            trace_id=None,
            source_node='',
            channel='/planner/request',
            event_type='planner_request',
            ab_object_id='scan',
            ab_level=1,
            summary='goal=scan',
            payload={'goal_text': 'scan around'},
            raw='{"goal_text":"scan around"}',
        )
    )

    assert first.trace_id is not None
    assert second.trace_id == first.trace_id


def test_trace_recorder_closes_trace_on_dialogue_act() -> None:
    recorder = TraceRecorder()
    one = recorder.add(
        InteractionEvent(
            timestamp=1.0,
            trace_id=None,
            source_node='',
            channel='/planner/request',
            event_type='planner_request',
            ab_object_id=None,
            ab_level=0,
            summary='goal=test',
            payload={'goal_text': 'test'},
            raw='{"goal_text":"test"}',
        )
    )
    recorder.add(
        InteractionEvent(
            timestamp=2.0,
            trace_id=None,
            source_node='',
            channel='/planner/dialogue_act',
            event_type='planner_dialogue_act',
            ab_object_id=None,
            ab_level=0,
            summary='done',
            payload={'text_hint': 'done'},
            raw='{"text_hint":"done"}',
        )
    )
    two = recorder.add(
        InteractionEvent(
            timestamp=3.0,
            trace_id=None,
            source_node='',
            channel='/planner/request',
            event_type='planner_request',
            ab_object_id=None,
            ab_level=0,
            summary='goal=next',
            payload={'goal_text': 'next'},
            raw='{"goal_text":"next"}',
        )
    )

    assert one.trace_id is not None
    assert two.trace_id is not None
    assert two.trace_id != one.trace_id
