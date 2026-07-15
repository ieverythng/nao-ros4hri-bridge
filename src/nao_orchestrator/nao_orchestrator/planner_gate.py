"""Admission gate for planner requests owned by ``nao_orchestrator``."""

from __future__ import annotations

from dataclasses import dataclass
import sys

from planner_common import PlannerRequest
from planner_common import parse_json_object

_FROZEN_DATACLASS_KWARGS = {'frozen': True}
if sys.version_info >= (3, 10):  # pragma: no branch - local macOS uses Python 3.9
    _FROZEN_DATACLASS_KWARGS['slots'] = True

_NON_EXECUTION_INTENTS = frozenset(
    {
        'greet',
        'identity',
        'wellbeing',
        'help',
        'kb_query_visible_entities',
        'kb_query_visible_people',
        'kb_query_visible_objects',
        'kb_query_scene_change',
    }
)


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class PlannerGateDecision:
    """Decision returned for one incoming planner request."""

    accepted: bool
    request: PlannerRequest
    reason: str = ''
    forward_payload: dict | None = None


class PlannerGate:
    """Track active planner goal admission without planning or executing."""

    def __init__(self) -> None:
        self._active_goal_id = ''
        self._active_plan_id = ''
        self._active_plan_version = 0
        self._active_status = ''

    @property
    def active_goal_id(self) -> str:
        return self._active_goal_id

    @property
    def active_plan_id(self) -> str:
        return self._active_plan_id

    def decide(self, payload) -> PlannerGateDecision:
        request_payload = parse_json_object(payload)
        request = PlannerRequest.from_payload(request_payload)
        kind = request.request_kind

        if kind == 'new_goal' and _is_non_execution_request(request):
            return PlannerGateDecision(
                False,
                request,
                'non-execution request must not enter planner execution',
            )

        if kind == 'cancel_request':
            if self._matches_active_goal(request):
                self._active_goal_id = ''
                self._active_plan_id = ''
                self._active_plan_version = 0
                self._active_status = ''
            return PlannerGateDecision(True, request)

        if kind == 'clarification_answer':
            if not self._matches_active_goal(request):
                return PlannerGateDecision(
                    False,
                    request,
                    'clarification answer does not match active planner goal',
                )
            return PlannerGateDecision(True, request)

        if kind == 'goal_update':
            if not self._matches_active_goal(request):
                return PlannerGateDecision(
                    False,
                    request,
                    'goal update does not match active planner goal',
                )
            return PlannerGateDecision(True, request)

        if self._active_goal_id and request.goal_id == self._active_goal_id:
            return PlannerGateDecision(False, request, 'duplicate active planner goal')

        if self._active_goal_id and request.supersedes_goal_id != self._active_goal_id:
            if (
                kind == 'new_goal'
                and self._active_status == 'waiting_user'
                and request.goal_id != self._active_goal_id
            ):
                forwarded_payload = dict(request_payload)
                forwarded_payload['supersedes_goal_id'] = self._active_goal_id
                forwarded_request = PlannerRequest.from_payload(forwarded_payload)
                self._active_goal_id = forwarded_request.goal_id
                self._active_plan_id = ''
                self._active_plan_version = 0
                self._active_status = 'planning'
                return PlannerGateDecision(
                    True,
                    forwarded_request,
                    reason='auto_supersede_waiting_user',
                    forward_payload=forwarded_payload,
                )
            return PlannerGateDecision(
                False,
                request,
                'planner goal already active; request must supersede or cancel it',
            )

        self._active_goal_id = request.goal_id
        self._active_plan_id = ''
        self._active_plan_version = 0
        self._active_status = 'planning'
        return PlannerGateDecision(True, request)

    def observe_feedback(self, payload) -> None:
        feedback = parse_json_object(payload)
        goal_id = str(feedback.get('goal_id', '')).strip()
        plan_id = str(feedback.get('plan_id', '')).strip()
        try:
            plan_version = max(0, int(feedback.get('plan_version', 0) or 0))
        except (TypeError, ValueError):
            plan_version = 0
        if goal_id and goal_id != self._active_goal_id:
            return
        if plan_id and self._active_plan_id and plan_id != self._active_plan_id:
            return
        if plan_version and self._active_plan_version and plan_version < self._active_plan_version:
            return
        if plan_id:
            self._active_plan_id = plan_id
        if plan_version and plan_version >= self._active_plan_version:
            self._active_plan_version = plan_version
        event_type = str(feedback.get('event_type', '')).strip().lower()
        status = str(feedback.get('status', '')).strip().lower()
        if status:
            self._active_status = status
        if event_type in ('plan_completed', 'plan_cancelled') or status in (
            'completed',
            'cancelled',
            'failed',
            'invalid',
        ):
            self._active_goal_id = ''
            self._active_plan_id = ''
            self._active_plan_version = 0
            self._active_status = ''

    def observe_dialogue_act(self, payload) -> None:
        dialogue_act = parse_json_object(payload)
        goal_id = str(dialogue_act.get('goal_id', '')).strip()
        if goal_id and goal_id != self._active_goal_id:
            return
        act = str(dialogue_act.get('act', '')).strip().lower()
        if act in ('explain_failure', 'notify_cancellation', 'notify_completion'):
            self._active_goal_id = ''
            self._active_plan_id = ''
            self._active_plan_version = 0
            self._active_status = ''
            return
        if act in ('ask_clarification', 'ask_for_help'):
            if _is_terminal_dialogue_act(dialogue_act):
                self._active_goal_id = ''
                self._active_plan_id = ''
                self._active_plan_version = 0
                self._active_status = ''
                return
            self._active_status = 'waiting_user'

    def _matches_active_goal(self, request: PlannerRequest) -> bool:
        if not self._active_goal_id:
            return False
        return self._active_goal_id in (
            request.goal_id,
            request.parent_goal_id,
            request.supersedes_goal_id,
        )


def _is_non_execution_request(request: PlannerRequest) -> bool:
    """Reject dialogue/knowledge turns that leaked into planner admission."""
    normalized_intents = {
        str(intent).strip().lower()
        for intent in request.normalized_intents
        if str(intent).strip()
    }
    if normalized_intents and normalized_intents.issubset(_NON_EXECUTION_INTENTS):
        return True

    return _is_dialogue_only_capability_question(request.goal_text)


def _is_dialogue_only_capability_question(text: str) -> bool:
    """Backstop known capability questions if their intent was misclassified."""
    normalized = ''.join(
        char for char in ' '.join(str(text or '').strip().lower().split())
        if char.isalnum() or char.isspace()
    ).strip()
    return any(
        marker in normalized
        for marker in (
            'what can you do',
            'what are you able to do',
            'what capabilities do you have',
            'what are your capabilities',
            'tell me what you can do',
            'what skills do you have',
            'which skills do you have',
            'what fake skills do you have',
            'do you have fake skills',
            'do you have any fake skills',
            'tell me about your skills',
        )
    )


def _is_terminal_dialogue_act(dialogue_act: dict) -> bool:
    """Return true when a planner act ends the goal instead of awaiting repair."""
    if bool(dialogue_act.get('await_user_response', False)):
        return False
    context = dialogue_act.get('context', {})
    if not isinstance(context, dict):
        context = {}
    status = str(
        dialogue_act.get('status', context.get('status', ''))
    ).strip().lower()
    terminal_reason = str(
        dialogue_act.get('terminal_reason', context.get('terminal_reason', ''))
    ).strip()
    return bool(terminal_reason) or status in (
        'completed',
        'cancelled',
        'failed',
        'invalid',
        'terminal',
    )
