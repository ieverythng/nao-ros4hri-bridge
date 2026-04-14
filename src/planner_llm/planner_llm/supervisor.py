"""Goal-keyed supervisory loop for planner_llm."""

from __future__ import annotations

from dataclasses import dataclass, field

from planner_common import ExecutionFeedback
from planner_common import PlannerDialogueAct
from planner_common import PlannerRequest
from planner_common import build_dialogue_act_payload
from planner_llm.planner_engine import PlannerDecision
from planner_llm.planner_engine import PlannerEngine

_MOTION_PROGRESS_TEXT = {
    'stand': 'standing up',
    'standinit': 'standing up',
    'posture_stand': 'standing up',
    'head_center': 'looking straight ahead',
    'head_look_left': 'looking to the left',
    'head_look_right': 'looking to the right',
    'head_look_up': 'looking up',
    'head_look_down': 'looking down',
    'sit': 'sitting down',
    'sitrelax': 'sitting down',
    'posture_sit': 'sitting down',
    'kneel': 'kneeling down',
    'crouch': 'kneeling down',
    'posture_kneel': 'kneeling down',
}
_MOTION_COMPLETION_TEXT = {
    'stand': 'I am standing now.',
    'standinit': 'I am standing now.',
    'posture_stand': 'I am standing now.',
    'head_center': 'I am looking straight ahead now.',
    'head_look_left': 'I am looking to the left now.',
    'head_look_right': 'I am looking to the right now.',
    'head_look_up': 'I am looking up now.',
    'head_look_down': 'I am looking down now.',
    'sit': 'I am sitting now.',
    'sitrelax': 'I am sitting now.',
    'posture_sit': 'I am sitting now.',
    'kneel': 'I am kneeling now.',
    'crouch': 'I am kneeling now.',
    'posture_kneel': 'I am kneeling now.',
}


@dataclass
class SupervisorState:
    """Supervisor-owned state for one goal."""

    goal_id: str
    parent_goal_id: str = ''
    supersedes_goal_id: str = ''
    current_status: str = 'idle'
    active_plan_id: str = ''
    plan_version: int = 0
    retry_budget_remaining: int = 0
    latest_world_timestamp_sec: float = 0.0
    last_execution_feedback: ExecutionFeedback | None = None
    awaiting_user_response: bool = False
    active_scene_targets: tuple[str, ...] = ()
    active_plan_steps: tuple[dict, ...] = ()
    communication_policy: dict = field(default_factory=dict)
    last_request: PlannerRequest | None = None


@dataclass(frozen=True)
class SupervisorOutcome:
    """One supervisor turn result."""

    decision: PlannerDecision | None = None
    dialogue_acts: tuple[PlannerDialogueAct, ...] = ()


class PlannerSupervisor:
    """Manage multi-turn supervisory state around planner requests and feedback."""

    def __init__(self, engine: PlannerEngine, *, auto_replan: bool = True) -> None:
        self._engine = engine
        self._auto_replan = bool(auto_replan)
        self._states: dict[str, SupervisorState] = {}
        self._plan_to_goal: dict[str, str] = {}

    def handle_request(
        self,
        request: PlannerRequest,
        *,
        world_model_text: str = '',
        world_model_snapshot: dict | None = None,
    ) -> SupervisorOutcome:
        state = self._states.get(request.goal_id)
        if request.request_kind == 'cancel_request':
            return self._cancel_goal(request, state)

        if request.supersedes_goal_id:
            self._supersede_goal(request.supersedes_goal_id)

        state = self._prepare_state_for_request(request, state)
        decision = self._engine.plan_request(
            request,
            world_model_text=world_model_text,
            world_model_snapshot=world_model_snapshot or {},
            feedback=state.last_execution_feedback if request.request_kind == 'clarification_answer' else None,
            goal_id=state.goal_id,
            plan_version=state.plan_version + 1,
            status='replanning' if request.request_kind == 'clarification_answer' else 'planning',
            communication_policy=state.communication_policy,
        )
        return self._finalize_plan(
            state,
            request=request,
            decision=decision,
        )

    def handle_feedback(
        self,
        feedback: ExecutionFeedback,
        *,
        world_model_text: str = '',
        world_model_snapshot: dict | None = None,
    ) -> SupervisorOutcome:
        state = self._state_for_feedback(feedback)
        if state is None:
            return SupervisorOutcome()

        state.last_execution_feedback = feedback
        if feedback.timestamp_sec > 0:
            state.latest_world_timestamp_sec = feedback.timestamp_sec

        if feedback.event_type == 'plan_accepted':
            state.current_status = 'executing'
            state.retry_budget_remaining = feedback.retry_budget
            if self._emit_acknowledge(state):
                return SupervisorOutcome(
                    dialogue_acts=(self._dialogue_act(
                        state,
                        act='acknowledge',
                        reason=feedback.reason or 'goal accepted',
                        text_hint=self._acknowledgement_text(state),
                    ),)
                )
            return SupervisorOutcome()

        if feedback.event_type == 'step_started':
            state.current_status = 'executing'
            state.retry_budget_remaining = feedback.retry_budget
            if self._emit_progress(state):
                return SupervisorOutcome(
                    dialogue_acts=(self._dialogue_act(
                        state,
                        act='progress_update',
                        reason=feedback.reason or 'step started',
                        text_hint=self._progress_text(state, feedback),
                    ),)
                )
            return SupervisorOutcome()

        if feedback.event_type == 'step_succeeded':
            state.current_status = 'executing'
            state.retry_budget_remaining = feedback.retry_budget
            return SupervisorOutcome()

        if feedback.event_type == 'plan_completed':
            state.current_status = 'completed'
            state.active_plan_id = ''
            state.awaiting_user_response = False
            self._forget_plan(feedback.plan_id)
            if self._emit_completion(state):
                return SupervisorOutcome(
                    dialogue_acts=(self._dialogue_act(
                        state,
                        act='notify_completion',
                        reason=feedback.reason or 'goal completed',
                        text_hint=self._completion_text(state),
                    ),)
                )
            return SupervisorOutcome()

        if feedback.event_type in ('plan_invalid', 'step_failed'):
            return self._handle_failure_feedback(
                state,
                feedback=feedback,
                world_model_text=world_model_text,
                world_model_snapshot=world_model_snapshot or {},
            )

        return SupervisorOutcome()

    def _prepare_state_for_request(
        self,
        request: PlannerRequest,
        state: SupervisorState | None,
    ) -> SupervisorState:
        if state is None:
            state = SupervisorState(goal_id=request.goal_id)

        state.parent_goal_id = request.parent_goal_id
        state.supersedes_goal_id = request.supersedes_goal_id
        state.current_status = 'planning'
        state.awaiting_user_response = False
        state.active_plan_steps = ()
        state.last_request = request
        if not state.communication_policy:
            state.communication_policy = {}
        self._states[state.goal_id] = state
        return state

    def _finalize_plan(
        self,
        state: SupervisorState,
        *,
        request: PlannerRequest,
        decision: PlannerDecision,
    ) -> SupervisorOutcome:
        plan_payload = dict(decision.payload.get('plan', {}))
        state.plan_version = max(1, int(plan_payload.get('plan_version', 1) or 1))
        state.active_plan_id = str(plan_payload.get('plan_id', '')).strip()
        state.retry_budget_remaining = int(plan_payload.get('retry_budget', 0) or 0)
        state.active_scene_targets = tuple(plan_payload.get('scene_targets', []))
        state.active_plan_steps = tuple(
            step for step in plan_payload.get('steps', []) if isinstance(step, dict)
        )
        state.communication_policy = dict(plan_payload.get('communication_policy', {}))

        if decision.mode in ('clarify', 'fail'):
            state.current_status = 'waiting_user' if decision.mode == 'clarify' else 'failed'
            state.awaiting_user_response = decision.mode == 'clarify'
            self._forget_plan(state.active_plan_id)
            return SupervisorOutcome(
                dialogue_acts=(self._dialogue_act(
                    state,
                    act='ask_clarification' if decision.mode == 'clarify' else 'explain_failure',
                    reason=self._decision_reason(decision),
                    text_hint=self._decision_reason(decision),
                    await_user_response=decision.mode == 'clarify',
                ),)
            )

        state.current_status = 'executing'
        state.awaiting_user_response = False
        self._plan_to_goal[state.active_plan_id] = state.goal_id
        return SupervisorOutcome(
            decision=decision,
        )

    def _handle_failure_feedback(
        self,
        state: SupervisorState,
        *,
        feedback: ExecutionFeedback,
        world_model_text: str,
        world_model_snapshot: dict,
    ) -> SupervisorOutcome:
        state.current_status = 'blocked'
        state.retry_budget_remaining = feedback.retry_budget

        if feedback.needs_user_input:
            state.current_status = 'waiting_user'
            state.awaiting_user_response = True
            return SupervisorOutcome(
                dialogue_acts=(self._dialogue_act(
                    state,
                    act='ask_clarification',
                    reason=feedback.reason or 'planner needs more detail',
                    text_hint=feedback.reason or 'I need a bit more detail before I continue.',
                    await_user_response=True,
                    slots_needed=list(feedback.unmet_preconditions),
                ),)
            )

        if feedback.blocking and feedback.unmet_preconditions:
            state.current_status = 'waiting_user'
            state.awaiting_user_response = True
            return SupervisorOutcome(
                dialogue_acts=(self._dialogue_act(
                    state,
                    act='ask_for_help',
                    reason=feedback.reason or 'execution is blocked',
                    text_hint=feedback.reason or 'I need help to continue this task.',
                    await_user_response=True,
                    slots_needed=list(feedback.unmet_preconditions),
                ),)
            )

        if not self._auto_replan or feedback.retry_budget <= 0 or state.last_request is None:
            state.current_status = 'failed'
            state.awaiting_user_response = False
            self._forget_plan(feedback.plan_id)
            return SupervisorOutcome(
                dialogue_acts=(self._dialogue_act(
                    state,
                    act='explain_failure',
                    reason=feedback.reason or 'execution failed',
                    text_hint=feedback.reason or 'I could not complete that task.',
                ),)
            )

        state.current_status = 'replanning'
        decision = self._engine.plan_request(
            state.last_request,
            world_model_text=world_model_text,
            world_model_snapshot=world_model_snapshot,
            feedback=feedback,
            goal_id=state.goal_id,
            plan_version=state.plan_version + 1,
            status='replanning',
            communication_policy=state.communication_policy,
        )
        return self._finalize_plan(state, request=state.last_request, decision=decision)

    def _cancel_goal(
        self,
        request: PlannerRequest,
        state: SupervisorState | None,
    ) -> SupervisorOutcome:
        if state is None:
            state = SupervisorState(goal_id=request.goal_id)

        state.current_status = 'cancelled'
        state.awaiting_user_response = False
        self._forget_plan(state.active_plan_id)
        state.active_plan_id = ''
        state.active_plan_steps = ()
        self._states[state.goal_id] = state
        return SupervisorOutcome(
            dialogue_acts=(self._dialogue_act(
                state,
                act='notify_cancellation',
                reason='goal cancelled',
                text_hint=request.ack_text or 'Okay, I will stop working on that.',
            ),)
        )

    def _supersede_goal(self, goal_id: str) -> None:
        state = self._states.get(str(goal_id or '').strip())
        if state is None:
            return
        state.current_status = 'superseded'
        state.awaiting_user_response = False
        self._forget_plan(state.active_plan_id)
        state.active_plan_id = ''
        state.active_plan_steps = ()

    def _state_for_feedback(self, feedback: ExecutionFeedback) -> SupervisorState | None:
        goal_id = feedback.goal_id or self._plan_to_goal.get(feedback.plan_id, '')
        if not goal_id:
            return None
        state = self._states.get(goal_id)
        if state is None:
            return None
        if feedback.plan_id and state.active_plan_id and feedback.plan_id != state.active_plan_id:
            return None
        return state

    def _dialogue_act(
        self,
        state: SupervisorState,
        *,
        act: str,
        reason: str = '',
        text_hint: str = '',
        await_user_response: bool = False,
        slots_needed: list[str] | None = None,
    ) -> PlannerDialogueAct:
        payload = build_dialogue_act_payload(
            goal_id=state.goal_id,
            plan_id=state.active_plan_id,
            plan_version=state.plan_version,
            act=act,
            await_user_response=await_user_response,
            reason=reason,
            text_hint=text_hint,
            slots_needed=slots_needed,
            context={
                'scene_targets': list(state.active_scene_targets),
                'plan_step_count': len(state.active_plan_steps),
                'requested_intents': list(state.last_request.normalized_intents)
                if state.last_request is not None
                else [],
                'user_text': state.last_request.user_text if state.last_request is not None else '',
                'status': state.current_status,
            },
        )
        return PlannerDialogueAct.from_payload(payload)

    def _emit_completion(self, state: SupervisorState) -> bool:
        if (
            len(state.active_plan_steps) == 1
            and state.active_plan_steps[0].get('type') == 'say'
        ):
            return False
        policy = dict(state.communication_policy or {})
        return bool(policy.get('emit_completion', False))

    def _emit_acknowledge(self, state: SupervisorState) -> bool:
        policy = dict(state.communication_policy or {})
        return bool(policy.get('emit_acknowledge', False))

    def _emit_progress(self, state: SupervisorState) -> bool:
        policy = dict(state.communication_policy or {})
        return bool(policy.get('emit_progress', False))

    def _acknowledgement_text(self, state: SupervisorState) -> str:
        if state.last_request is not None and state.last_request.ack_text:
            return state.last_request.ack_text
        return 'Okay, I am starting now.'

    def _progress_text(
        self,
        state: SupervisorState,
        feedback: ExecutionFeedback,
    ) -> str:
        step = self._step_for_feedback(state, feedback)
        summary = self._step_summary(step, scene_targets=state.active_scene_targets)
        if summary:
            return 'I am %s.' % summary
        return 'I am working on it now.'

    def _completion_text(self, state: SupervisorState) -> str:
        if len(state.active_plan_steps) > 1:
            return 'I finished that sequence.'
        if state.active_plan_steps:
            step_text = self._completion_text_for_step(
                state.active_plan_steps[0],
                scene_targets=state.active_scene_targets,
            )
            if step_text:
                return step_text
        if state.last_request is not None and len(state.last_request.normalized_intents) == 1:
            intent_name = state.last_request.normalized_intents[0]
            step_text = self._completion_text_for_step(
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': intent_name},
                },
                scene_targets=state.active_scene_targets,
            )
            if step_text:
                return step_text
        return 'I finished that task.'

    @staticmethod
    def _step_for_feedback(state: SupervisorState, feedback: ExecutionFeedback) -> dict:
        for step in state.active_plan_steps:
            if not isinstance(step, dict):
                continue
            if feedback.step_id and str(step.get('id', '')).strip() == feedback.step_id:
                return step
        if len(state.active_plan_steps) == 1:
            return state.active_plan_steps[0]
        return {}

    @staticmethod
    def _step_summary(step: dict, *, scene_targets: tuple[str, ...]) -> str:
        if not isinstance(step, dict):
            return ''
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        step_args = dict(step.get('args', {}))

        if step_type == 'look_at' or step_name == 'look_at':
            if scene_targets:
                return 'looking at %s' % ', '.join(scene_targets)
            if step_args.get('target_frame'):
                return 'looking at the requested target'
            if str(step_args.get('policy', '')).strip().lower() == 'reset':
                return 'resetting my gaze'

        if step_type == 'skill' and step_name in ('', 'motion', 'perform_motion'):
            motion_name = str(step_args.get('object', '')).strip().lower()
            return _MOTION_PROGRESS_TEXT.get(motion_name, '')

        return ''

    @staticmethod
    def _completion_text_for_step(step: dict, *, scene_targets: tuple[str, ...]) -> str:
        if not isinstance(step, dict):
            return ''
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        step_args = dict(step.get('args', {}))

        if step_type == 'look_at' or step_name == 'look_at':
            if scene_targets:
                return 'I am looking at %s now.' % ', '.join(scene_targets)
            if str(step_args.get('policy', '')).strip().lower() == 'reset':
                return 'I am looking straight ahead now.'
            return 'I finished redirecting my gaze.'

        if step_type == 'skill' and step_name in ('', 'motion', 'perform_motion'):
            motion_name = str(step_args.get('object', '')).strip().lower()
            return _MOTION_COMPLETION_TEXT.get(motion_name, '')

        return ''

    @staticmethod
    def _decision_reason(decision: PlannerDecision) -> str:
        plan_payload = dict(decision.payload.get('plan', {}))
        user_facing_reason = str(
            plan_payload.get('user_facing_reason', '')
            or decision.payload.get('user_facing_reason', '')
        ).strip()
        if user_facing_reason:
            return user_facing_reason
        steps = plan_payload.get('steps', [])
        if isinstance(steps, list) and steps:
            first_step = steps[0]
            if isinstance(first_step, dict):
                step_args = dict(first_step.get('args', {}))
                step_text = str(
                    step_args.get('text', step_args.get('object', ''))
                ).strip()
                if step_text:
                    return step_text
        return str(
            plan_payload.get('failure_reason', '')
            or plan_payload.get('replan_hint', '')
            or decision.payload.get('ack_text', '')
            or 'I need more detail before I continue.'
        ).strip()

    def _forget_plan(self, plan_id: str) -> None:
        clean_plan_id = str(plan_id or '').strip()
        if clean_plan_id:
            self._plan_to_goal.pop(clean_plan_id, None)
