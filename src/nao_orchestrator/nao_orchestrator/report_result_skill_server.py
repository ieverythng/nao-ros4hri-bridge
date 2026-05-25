#!/usr/bin/env python3
"""Lifecycle report-result skill action server.

This server owns result-reporting execution so the orchestrator can keep
deterministic routing responsibilities only.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

from communication_skills.action import Say
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_skills.msg import Result as SkillResult


@dataclass(slots=True)
class _ForwardResult:
    accepted: bool
    success: bool
    error_code: int
    error_msg: str
    reason: str = ''


class ReportResultSkillServer(Node):
    """Implement `/skill/report_result` as a lifecycle action server."""

    def __init__(self) -> None:
        super().__init__('report_result_skill_server')

        self.declare_parameter('action_name', '/skill/report_result')
        self.declare_parameter('say_action', '/skill/say')
        self.declare_parameter('say_action_wait_sec', 0.2)
        self.declare_parameter('say_action_result_timeout_sec', 8.0)

        self.action_name = str(self.get_parameter('action_name').value).strip() or '/skill/report_result'
        self.say_action = str(self.get_parameter('say_action').value).strip() or '/skill/say'
        self.say_action_wait_sec = max(
            0.0,
            float(self.get_parameter('say_action_wait_sec').value),
        )
        self.say_action_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('say_action_result_timeout_sec').value),
        )

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._is_active = False

        self._action_server = None
        self._say_client = None

        self.get_logger().info('report_result_skill_server created; waiting for lifecycle configure')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._teardown_runtime_interfaces()
        self._say_client = ActionClient(
            self,
            Say,
            self.say_action,
        )
        self._action_server = ActionServer(
            self,
            Say,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'report_result_skill_server configured | action:%s say:%s'
            % (self.action_name, self.say_action)
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return super().on_deactivate(state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._teardown_runtime_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._teardown_runtime_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def _teardown_runtime_interfaces(self) -> None:
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._say_client is not None:
            self._say_client.destroy()
            self._say_client = None

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, _goal_request: Say.Goal) -> GoalResponse:
        if not self._is_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = Say.Result()

        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            result.result.error_code = SkillResult.ROS_EOTHER
            result.result.error_msg = 'Another report_result goal is already executing'
            return result

        try:
            request = goal_handle.request
            summary_text = str(request.input or '').strip()
            if not summary_text:
                goal_handle.abort()
                result.result.error_code = SkillResult.ROS_EBADMSG
                result.result.error_msg = 'report_result goal missing input summary text'
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result.error_code = SkillResult.ROS_ECANCELED
                result.result.error_msg = 'report_result goal canceled before dispatch'
                return result

            goal_handle.publish_feedback(self._feedback('forwarding_to_skill_say'))

            forward_goal = Say.Goal()
            forward_goal.meta = request.meta
            forward_goal.person_id = str(request.person_id or '').strip()
            forward_goal.group_id = str(request.group_id or '').strip()
            forward_goal.input = summary_text

            forward_result = self._forward_to_say_action(forward_goal)
            if forward_result.success:
                goal_handle.succeed()
                result.result.error_code = SkillResult.ROS_ENOERR
                result.result.error_msg = ''
                goal_handle.publish_feedback(self._feedback('completed'))
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result.error_code = SkillResult.ROS_ECANCELED
                result.result.error_msg = forward_result.error_msg or 'report_result goal canceled'
                return result

            goal_handle.abort()
            result.result.error_code = int(forward_result.error_code)
            result.result.error_msg = forward_result.error_msg or forward_result.reason or 'report_result forward dispatch failed'
            goal_handle.publish_feedback(self._feedback('failed'))
            return result
        finally:
            self._execution_lock.release()

    # ------------------------------------------------------------------
    # Internal forwarding
    # ------------------------------------------------------------------

    def _forward_to_say_action(self, goal: Say.Goal) -> _ForwardResult:
        client = self._say_client
        if client is None:
            return _ForwardResult(
                accepted=False,
                success=False,
                error_code=SkillResult.ROS_ENOTSUP,
                error_msg='report_result say client unavailable',
                reason='say client unavailable',
            )
        if not client.wait_for_server(timeout_sec=self.say_action_wait_sec):
            return _ForwardResult(
                accepted=False,
                success=False,
                error_code=SkillResult.ROS_ENOTSUP,
                error_msg='report_result target say action unavailable',
                reason='say action unavailable',
            )

        acceptance_event = threading.Event()
        result_event = threading.Event()
        active_goal_handle = {'value': None}
        outcome = {
            'accepted': False,
            'success': False,
            'error_code': SkillResult.ROS_EOTHER,
            'error_msg': 'report_result forward result timed out',
            'reason': 'report_result forward result timed out',
        }

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as err:  # pragma: no cover - ROS transport failure
                outcome['reason'] = 'report_result goal response failed: %s' % err
                outcome['error_msg'] = outcome['reason']
                acceptance_event.set()
                result_event.set()
                return

            if goal_handle is None or not goal_handle.accepted:
                outcome['reason'] = 'report_result forward goal rejected'
                outcome['error_msg'] = outcome['reason']
                acceptance_event.set()
                result_event.set()
                return

            active_goal_handle['value'] = goal_handle
            outcome['accepted'] = True
            acceptance_event.set()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                wrapped_result = future.result()
                action_result = getattr(wrapped_result, 'result', None)
                success, error_code, error_msg = self._forward_result_status(action_result)
                outcome['success'] = success
                outcome['error_code'] = int(error_code)
                outcome['error_msg'] = error_msg
                outcome['reason'] = error_msg
            except Exception as err:  # pragma: no cover - ROS transport failure
                outcome['success'] = False
                outcome['error_code'] = SkillResult.ROS_EOTHER
                outcome['reason'] = 'report_result result retrieval failed: %s' % err
                outcome['error_msg'] = outcome['reason']
            finally:
                result_event.set()

        goal_future = client.send_goal_async(goal)
        goal_future.add_done_callback(_goal_response_callback)

        goal_response_timeout = max(float(self.say_action_wait_sec), 1.0)
        if not acceptance_event.wait(timeout=goal_response_timeout):
            return _ForwardResult(
                accepted=False,
                success=False,
                error_code=SkillResult.ROS_EOTHER,
                error_msg='report_result forward goal response timed out',
                reason='goal response timed out',
            )
        if not outcome['accepted']:
            return _ForwardResult(
                accepted=False,
                success=False,
                error_code=int(outcome['error_code']),
                error_msg=str(outcome['error_msg']).strip(),
                reason=str(outcome['reason']).strip(),
            )
        if not result_event.wait(timeout=max(float(self.say_action_result_timeout_sec), 0.1)):
            goal_handle = active_goal_handle.get('value')
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:  # pragma: no cover - best effort cancel
                    pass
            return _ForwardResult(
                accepted=True,
                success=False,
                error_code=SkillResult.ROS_EOTHER,
                error_msg='report_result forward result timed out',
                reason='result timed out',
            )
        return _ForwardResult(
            accepted=True,
            success=bool(outcome['success']),
            error_code=int(outcome['error_code']),
            error_msg=str(outcome['error_msg']).strip(),
            reason=str(outcome['reason']).strip(),
        )

    @staticmethod
    def _forward_result_status(action_result) -> tuple[bool, int, str]:
        if action_result is None:
            return False, SkillResult.ROS_EOTHER, 'report_result target returned no result'
        skill_result = getattr(action_result, 'result', None)
        if skill_result is None:
            return False, SkillResult.ROS_EOTHER, 'report_result target result payload missing'
        error_code = int(getattr(skill_result, 'error_code', SkillResult.ROS_EOTHER))
        error_msg = str(getattr(skill_result, 'error_msg', '')).strip()
        if error_code == SkillResult.ROS_ENOERR:
            return True, error_code, ''
        return False, error_code, error_msg or ('report_result target failed with error_code=%d' % error_code)

    @staticmethod
    def _feedback(message: str) -> Say.Feedback:
        feedback = Say.Feedback()
        feedback.feedback.data_str = str(message)
        return feedback
