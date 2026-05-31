from planner_common import ExecutionFeedback
from planner_common import PlannerRequest

from planner_llm.planner_engine import PlannerDecision
from planner_llm.supervisor import PlannerSupervisor


class _StubEngine:
    def __init__(self) -> None:
        self.calls = []

    def plan_request(self, request, **kwargs):
        self.calls.append((request, kwargs))
        plan_version = kwargs.get('plan_version', 1)
        goal_id = kwargs.get('goal_id', request.goal_id)
        status = kwargs.get('status', 'planning')
        plan_id = 'plan_%s_v%d' % (goal_id, plan_version)
        return PlannerDecision(
            intent_name='raw_user_input',
            plan_id=plan_id,
            mode='plan',
            payload={
                'scene_targets': list(request.scene_targets),
                'grounded_context': request.grounded_context,
                'plan': {
                    'goal_id': goal_id,
                    'plan_id': plan_id,
                    'plan_version': plan_version,
                    'status': status,
                    'validation_status': 'draft',
                    'failure_reason': '',
                    'replan_hint': '',
                    'retry_budget': 1,
                    'scene_targets': list(request.scene_targets),
                    'communication_policy': {
                        'emit_acknowledge': False,
                        'emit_progress': False,
                        'emit_completion': True,
                        'emit_failure': True,
                    },
                    'steps': [
                        {
                            'id': 'step_1',
                            'type': 'skill',
                            'name': 'perform_motion',
                            'args': {'object': 'head_center'},
                            'requires': [],
                            'on_failure': 'replan',
                            'retry_budget': 0,
                        }
                    ],
                },
            },
        )


class _ClarifyEngine(_StubEngine):
    def plan_request(self, request, **kwargs):
        self.calls.append((request, kwargs))
        plan_version = kwargs.get('plan_version', 1)
        goal_id = kwargs.get('goal_id', request.goal_id)
        return PlannerDecision(
            intent_name='raw_user_input',
            plan_id='plan_%s_v%d' % (goal_id, plan_version),
            mode='clarify',
            payload={
                'scene_targets': list(request.scene_targets),
                'grounded_context': request.grounded_context,
                'plan': {
                    'goal_id': goal_id,
                    'plan_id': 'plan_%s_v%d' % (goal_id, plan_version),
                    'plan_version': plan_version,
                    'status': 'waiting_user',
                    'validation_status': 'draft',
                    'failure_reason': '',
                    'replan_hint': 'clarify_user',
                    'retry_budget': 0,
                    'scene_targets': list(request.scene_targets),
                    'communication_policy': {
                        'emit_acknowledge': False,
                        'emit_progress': False,
                        'emit_completion': False,
                        'emit_failure': True,
                    },
                    'steps': [
                        {
                            'id': 'step_1',
                            'type': 'say',
                            'name': 'say',
                            'args': {'text': 'Which cup do you mean?'},
                            'requires': [],
                            'on_failure': 'fail',
                            'retry_budget': 0,
                        }
                    ],
                },
            },
        )


class _BackendUnavailableEngine(_ClarifyEngine):
    def plan_request(self, request, **kwargs):
        decision = super().plan_request(request, **kwargs)
        payload = dict(decision.payload)
        payload['plan'] = dict(decision.payload['plan'])
        payload['plan']['status'] = 'failed'
        payload['plan']['failure_reason'] = (
            'The planning model is not ready yet. Please try again in a moment.'
        )
        payload['plan']['replan_hint'] = 'planner_backend_unavailable'
        return PlannerDecision(
            intent_name=decision.intent_name,
            plan_id=decision.plan_id,
            mode='backend_unavailable',
            payload=payload,
        )


class _AckEngine(_StubEngine):
    def plan_request(self, request, **kwargs):
        decision = super().plan_request(request, **kwargs)
        decision.payload['plan']['communication_policy']['emit_acknowledge'] = True
        return decision


class _MotionSequenceEngine(_StubEngine):
    def plan_request(self, request, **kwargs):
        decision = super().plan_request(request, **kwargs)
        decision.payload['plan']['steps'] = [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'perform_motion',
                'args': {'object': 'head_look_up'},
                'requires': [],
                'on_failure': 'replan',
                'retry_budget': 0,
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'perform_motion',
                'args': {'object': 'head_look_down'},
                'requires': [],
                'on_failure': 'replan',
                'retry_budget': 0,
            },
        ]
        return decision


class _MotionScanEngine(_StubEngine):
    def plan_request(self, request, **kwargs):
        decision = super().plan_request(request, **kwargs)
        decision.payload['plan']['steps'] = [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'perform_motion',
                'args': {'object': 'head_look_left'},
                'requires': [],
                'on_failure': 'replan',
                'retry_budget': 0,
            },
            {
                'id': 'step_2',
                'type': 'skill',
                'name': 'scan',
                'args': {'target': 'people', 'target_kind': 'people'},
                'requires': [],
                'on_failure': 'replan',
                'retry_budget': 0,
            },
        ]
        return decision


class _ResultSayEngine(_StubEngine):
    def plan_request(self, request, **kwargs):
        decision = super().plan_request(request, **kwargs)
        decision.payload['plan']['steps'] = [
            {
                'id': 'step_1',
                'type': 'skill',
                'name': 'scan',
                'args': {'target': 'people'},
                'requires': [],
                'on_failure': 'replan',
                'retry_budget': 0,
            },
            {
                'id': 'step_2',
                'type': 'say',
                'name': 'say',
                'args': {'text': 'I found one person.'},
                'requires': [],
                'on_failure': 'continue',
                'retry_budget': 0,
            },
        ]
        return decision


def test_supervisor_creates_new_goal_session_without_duplicate_ack_dialogue_act() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_1',
            'request_id': 'turn_1',
            'request_kind': 'new_goal',
            'user_text': 'look forward',
            'normalized_intents': ['head_center'],
        }
    )

    outcome = supervisor.handle_request(request)
    assert outcome.decision is not None
    assert outcome.decision.payload['plan']['goal_id'] == 'goal_1'
    assert outcome.decision.payload['plan']['plan_version'] == 1
    assert outcome.dialogue_acts == ()


def test_supervisor_marks_superseded_goal_before_new_plan() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    first_request = PlannerRequest.from_payload(
        {'goal_id': 'goal_old', 'request_id': 'turn_1', 'user_text': 'look left'}
    )
    supervisor.handle_request(first_request)

    second_request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_new',
            'request_id': 'turn_2',
            'user_text': 'look right instead',
            'supersedes_goal_id': 'goal_old',
        }
    )
    outcome = supervisor.handle_request(second_request)
    assert outcome.decision is not None
    assert outcome.decision.payload['plan']['goal_id'] == 'goal_new'


def test_supervisor_replans_same_goal_after_clarification_answer() -> None:
    engine = _StubEngine()
    supervisor = PlannerSupervisor(engine, auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_1', 'request_id': 'turn_1', 'user_text': 'bring the cup'}
    )
    supervisor.handle_request(request)

    clarification = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_1',
            'request_id': 'turn_2',
            'request_kind': 'clarification_answer',
            'user_text': 'the blue cup',
        }
    )
    outcome = supervisor.handle_request(clarification)
    assert outcome.decision is not None
    assert outcome.decision.payload['plan']['goal_id'] == 'goal_1'
    assert outcome.decision.payload['plan']['plan_version'] == 2


def test_supervisor_handles_cancel_request_without_publishing_intent() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_cancel', 'request_id': 'turn_1', 'user_text': 'look left'}
    )
    supervisor.handle_request(request)

    cancel_request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_cancel',
            'request_id': 'turn_2',
            'request_kind': 'cancel_request',
        }
    )
    outcome = supervisor.handle_request(cancel_request)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'notify_cancellation'


def test_supervisor_replans_after_retryable_failure() -> None:
    engine = _StubEngine()
    supervisor = PlannerSupervisor(engine, auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_retry', 'request_id': 'turn_1', 'user_text': 'look at the cup'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_retry',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'step_failed',
            'status': 'failed',
            'reason': 'target moved',
            'retry_budget': 1,
            'blocking': False,
        }
    )

    outcome = supervisor.handle_feedback(feedback)
    assert outcome.decision is not None
    assert outcome.decision.payload['plan']['plan_version'] == 2


def test_supervisor_replans_blocking_retryable_failure_without_unmet_preconditions() -> None:
    engine = _StubEngine()
    supervisor = PlannerSupervisor(engine, auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_retry_blocking', 'request_id': 'turn_1', 'user_text': 'find the cup'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_retry_blocking',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'step_failed',
            'status': 'failed',
            'reason': 'vision unstable',
            'retry_budget': 1,
            'blocking': True,
            'unmet_preconditions': [],
            'step': {
                'id': 'step_1',
                'type': 'skill',
                'name': 'find_object',
                'on_failure': 'replan',
                'retry_budget': 1,
            },
        }
    )

    outcome = supervisor.handle_feedback(feedback)
    assert outcome.decision is not None
    assert outcome.decision.payload['plan']['plan_version'] == 2


def test_supervisor_asks_for_help_when_retry_budget_exhausted_for_retryable_failure() -> None:
    engine = _StubEngine()
    supervisor = PlannerSupervisor(engine, auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_retry_exhausted', 'request_id': 'turn_1', 'user_text': 'go to the cup'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_retry_exhausted',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'step_failed',
            'status': 'failed',
            'reason': 'path blocked',
            'retry_budget': 0,
            'blocking': True,
        }
    )

    outcome = supervisor.handle_feedback(feedback)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'ask_for_help'
    assert outcome.dialogue_acts[0].await_user_response is True


def test_supervisor_does_not_replan_when_step_failure_policy_is_fail() -> None:
    engine = _StubEngine()
    supervisor = PlannerSupervisor(engine, auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_fail', 'request_id': 'turn_1', 'user_text': 'look at the cup'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_fail',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'step_failed',
            'status': 'failed',
            'reason': 'motion controller ignored the command',
            'retry_budget': 2,
            'step': {
                'id': 'step_1',
                'type': 'skill',
                'name': 'perform_motion',
                'on_failure': 'fail',
                'retry_budget': 2,
            },
        }
    )

    outcome = supervisor.handle_feedback(feedback)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'explain_failure'


def test_supervisor_emits_dialogue_act_instead_of_intent_for_clarification() -> None:
    supervisor = PlannerSupervisor(_ClarifyEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_clarify', 'request_id': 'turn_1', 'user_text': 'bring me the cup'}
    )

    outcome = supervisor.handle_request(request)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'ask_clarification'


def test_supervisor_reports_backend_unavailable_as_failure_not_clarification() -> None:
    supervisor = PlannerSupervisor(_BackendUnavailableEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_backend', 'request_id': 'turn_1', 'goal_text': 'scan the room'}
    )

    outcome = supervisor.handle_request(request)

    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'explain_failure'
    assert outcome.dialogue_acts[0].await_user_response is False


def test_supervisor_emits_completion_dialogue_act_when_policy_allows_it() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_done', 'request_id': 'turn_1', 'user_text': 'look ahead'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_done',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'plan_completed',
            'status': 'completed',
        }
    )

    outcome = supervisor.handle_feedback(feedback)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'notify_completion'
    assert outcome.dialogue_acts[0].text_hint == 'I am looking straight ahead now.'


def test_supervisor_uses_task_specific_completion_for_motion_sequence() -> None:
    supervisor = PlannerSupervisor(_MotionSequenceEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_motion_sequence', 'request_id': 'turn_1', 'user_text': 'nod'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_motion_sequence',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'plan_completed',
            'status': 'completed',
        }
    )

    outcome = supervisor.handle_feedback(feedback)

    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].text_hint == 'I am looking down now.'


def test_supervisor_suppresses_completion_when_plan_ended_with_result_say() -> None:
    supervisor = PlannerSupervisor(_ResultSayEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_scan_result', 'request_id': 'turn_1', 'user_text': 'scan'}
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_scan_result',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'plan_completed',
            'status': 'completed',
        }
    )

    outcome = supervisor.handle_feedback(feedback)

    assert outcome.decision is None
    assert outcome.dialogue_acts == ()


def test_supervisor_completion_act_carries_latest_result_summary() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_scan_summary', 'request_id': 'turn_1', 'user_text': 'scan'}
    )
    first_outcome = supervisor.handle_request(request)
    step_feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_scan_summary',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'step_succeeded',
            'status': 'succeeded',
            'result_summary': 'I found one person.',
            'result_payload': {
                'skill': 'scan',
                'target_kind': 'people',
                'target_found': True,
                'people': [{'id': 'anonymous_person_1', 'source': 'hri_tracked_persons'}],
                'summary_text': 'I found one person.',
            },
        }
    )
    supervisor.handle_feedback(step_feedback)

    outcome = supervisor.handle_feedback(
        ExecutionFeedback.from_payload(
            {
                'goal_id': 'goal_scan_summary',
                'plan_id': first_outcome.decision.plan_id,
                'plan_version': 1,
                'event_type': 'plan_completed',
                'status': 'completed',
            }
        )
    )

    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].text_hint == 'I found one person.'
    assert outcome.dialogue_acts[0].context['result_summary'] == 'I found one person.'
    assert outcome.dialogue_acts[0].context['result_payload']['skill'] == 'scan'


def test_supervisor_prefers_scan_result_over_motion_completion_copy() -> None:
    supervisor = PlannerSupervisor(_MotionScanEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_scan_summary', 'request_id': 'turn_1', 'user_text': 'scan for people'}
    )
    first_outcome = supervisor.handle_request(request)
    supervisor.handle_feedback(
        ExecutionFeedback.from_payload(
            {
                'goal_id': 'goal_scan_summary',
                'plan_id': first_outcome.decision.plan_id,
                'plan_version': 1,
                'event_type': 'step_succeeded',
                'status': 'succeeded',
                'step_id': 'step_2',
                'result_summary': (
                    'I completed the scan for people, but no confirmed detection result was reported.'
                ),
            }
        )
    )

    outcome = supervisor.handle_feedback(
        ExecutionFeedback.from_payload(
            {
                'goal_id': 'goal_scan_summary',
                'plan_id': first_outcome.decision.plan_id,
                'plan_version': 1,
                'event_type': 'plan_completed',
                'status': 'completed',
            }
        )
    )

    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].text_hint.startswith('I completed the scan for people')
    assert 'looking to the left' not in outcome.dialogue_acts[0].text_hint


def test_supervisor_emits_acknowledgement_dialogue_act_when_policy_allows_it() -> None:
    supervisor = PlannerSupervisor(_AckEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_ack',
            'request_id': 'turn_1',
            'user_text': 'look ahead',
        }
    )
    first_outcome = supervisor.handle_request(request)
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_ack',
            'plan_id': first_outcome.decision.plan_id,
            'plan_version': 1,
            'event_type': 'plan_accepted',
            'status': 'accepted',
        }
    )

    outcome = supervisor.handle_feedback(feedback)

    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'acknowledge'
    assert outcome.dialogue_acts[0].text_hint == 'Okay, I am starting now.'


def test_supervisor_resets_cached_result_context_between_replans_of_same_goal() -> None:
    class _NoStepEngine(_StubEngine):
        def plan_request(self, request, **kwargs):
            decision = super().plan_request(request, **kwargs)
            decision.payload['plan']['steps'] = []
            return decision

    supervisor = PlannerSupervisor(_NoStepEngine(), auto_replan=True)
    first_request = PlannerRequest.from_payload(
        {'goal_id': 'goal_stale_result_cache', 'request_id': 'turn_1', 'user_text': 'first pass'}
    )
    first_outcome = supervisor.handle_request(first_request)
    supervisor.handle_feedback(
        ExecutionFeedback.from_payload(
            {
                'goal_id': 'goal_stale_result_cache',
                'plan_id': first_outcome.decision.plan_id,
                'plan_version': 1,
                'event_type': 'plan_completed',
                'status': 'completed',
                'result_summary': 'I already found one person.',
            }
        )
    )

    second_request = PlannerRequest.from_payload(
        {'goal_id': 'goal_stale_result_cache', 'request_id': 'turn_2', 'user_text': 'second pass'}
    )
    second_outcome = supervisor.handle_request(second_request)
    outcome = supervisor.handle_feedback(
        ExecutionFeedback.from_payload(
            {
                'goal_id': 'goal_stale_result_cache',
                'plan_id': second_outcome.decision.plan_id,
                'plan_version': 2,
                'event_type': 'plan_completed',
                'status': 'completed',
            }
        )
    )
    assert outcome.dialogue_acts == ()
