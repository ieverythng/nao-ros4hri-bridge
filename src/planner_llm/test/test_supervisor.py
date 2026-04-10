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
                'goal_id': goal_id,
                'ack_text': '',
                'ack_mode': '',
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
                'goal_id': goal_id,
                'ack_text': '',
                'ack_mode': '',
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


def test_supervisor_creates_new_goal_session_without_duplicate_ack_dialogue_act() -> None:
    supervisor = PlannerSupervisor(_StubEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {
            'goal_id': 'goal_1',
            'request_id': 'turn_1',
            'request_kind': 'new_goal',
            'user_text': 'look forward',
            'ack_text': 'I will do that.',
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
            'ack_text': 'Okay, cancel that.',
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


def test_supervisor_emits_dialogue_act_instead_of_intent_for_clarification() -> None:
    supervisor = PlannerSupervisor(_ClarifyEngine(), auto_replan=True)
    request = PlannerRequest.from_payload(
        {'goal_id': 'goal_clarify', 'request_id': 'turn_1', 'user_text': 'bring me the cup'}
    )

    outcome = supervisor.handle_request(request)
    assert outcome.decision is None
    assert len(outcome.dialogue_acts) == 1
    assert outcome.dialogue_acts[0].act == 'ask_clarification'


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
