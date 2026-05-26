"""Admission gate for planner requests owned by ``nao_orchestrator``."""

from __future__ import annotations

from dataclasses import dataclass

from planner_common import PlannerRequest
from planner_common import parse_json_object


@dataclass(frozen=True, slots=True)
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
        self._active_goal_token = ''
        self._active_plan_version = 0
        self._active_status = ''

    @property
    def active_goal_id(self) -> str:
        return self._active_goal_id

    @property
    def active_goal_token(self) -> str:
        return self._active_goal_token

    def decide(self, payload) -> PlannerGateDecision:
        request_payload = parse_json_object(payload)
        request = PlannerRequest.from_payload(request_payload)
        kind = request.request_kind

        if kind == 'cancel_request':
            if self._matches_active_goal(request):
                self._active_goal_id = ''
                self._active_goal_token = ''
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
            if not self._active_goal_token or request.goal_token == self._active_goal_token:
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
                self._active_goal_token = forwarded_request.goal_token or forwarded_request.goal_id
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
        self._active_goal_token = request.goal_token or request.goal_id
        self._active_plan_version = 0
        self._active_status = 'planning'
        return PlannerGateDecision(True, request)

    def observe_feedback(self, payload) -> None:
        feedback = parse_json_object(payload)
        goal_id = str(feedback.get('goal_id', '')).strip()
        goal_token = str(feedback.get('goal_token', '')).strip()
        try:
            plan_version = max(0, int(feedback.get('plan_version', 0) or 0))
        except (TypeError, ValueError):
            plan_version = 0
        if goal_id and goal_id != self._active_goal_id:
            return
        if goal_token and self._active_goal_token and goal_token != self._active_goal_token:
            return
        if plan_version and self._active_plan_version and plan_version < self._active_plan_version:
            return
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
            self._active_goal_token = ''
            self._active_plan_version = 0
            self._active_status = ''

    def observe_dialogue_act(self, payload) -> None:
        dialogue_act = parse_json_object(payload)
        goal_id = str(dialogue_act.get('goal_id', '')).strip()
        goal_token = str(dialogue_act.get('goal_token', '')).strip()
        if goal_id and goal_id != self._active_goal_id:
            return
        if goal_token and self._active_goal_token and goal_token != self._active_goal_token:
            return
        act = str(dialogue_act.get('act', '')).strip().lower()
        if act in ('explain_failure', 'notify_cancellation'):
            self._active_goal_id = ''
            self._active_goal_token = ''
            self._active_plan_version = 0
            self._active_status = ''
            return
        if act in ('ask_clarification', 'ask_for_help'):
            self._active_status = 'waiting_user'

    def _matches_active_goal(self, request: PlannerRequest) -> bool:
        if not self._active_goal_id:
            return False
        if self._active_goal_token and request.goal_token:
            return request.goal_token == self._active_goal_token
        return self._active_goal_id in (
            request.goal_id,
            request.parent_goal_id,
            request.supersedes_goal_id,
        )
