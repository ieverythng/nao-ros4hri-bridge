#!/usr/bin/env python3
"""Head-motion skill action client for mission controller."""

from typing import Callable
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from nao_skills.action import DoHeadMotion


class HeadMotionSkillClient:
    """Non-blocking action client for head-motion skills."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/do_head_motion",
        default_speed: float = 0.25,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._default_speed = default_speed

        self._client = ActionClient(node, DoHeadMotion, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[DoHeadMotion.Result], None]] = None
        self._active_turn_id = ""

        node.get_logger().info(f"Head-motion skill client initialized: {action_name}")

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for action server availability."""
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def server_is_ready(self) -> bool:
        """Return whether the action server is currently ready."""
        return self._client.server_is_ready()

    def ensure_server(self, wait_timeout_sec: float = 0.0) -> bool:
        """Return server readiness, optionally waiting for discovery."""
        if self.server_is_ready():
            return True
        if wait_timeout_sec <= 0.0:
            return False
        return self.wait_for_server(timeout_sec=wait_timeout_sec)

    def send_goal(
        self,
        yaw: float,
        pitch: float,
        speed: Optional[float] = None,
        relative: bool = False,
        turn_id: str = "",
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable[[DoHeadMotion.Result], None]] = None,
    ) -> None:
        """Send head-motion goal asynchronously."""
        if not self.server_is_ready():
            self._node.get_logger().warn(
                f"Head-motion action server '{self._action_name}' is not ready"
            )
            return

        resolved_speed = self._default_speed if speed is None else float(speed)

        goal = DoHeadMotion.Goal()
        goal.yaw = float(yaw)
        goal.pitch = float(pitch)
        goal.speed = resolved_speed
        goal.relative = bool(relative)

        self._result_callback = result_callback
        self._active_turn_id = str(turn_id).strip()
        mode = "relative" if goal.relative else "absolute"

        self._node.get_logger().info(
            "%s HEAD_SEND | yaw=%.4f pitch=%.4f speed=%.2f mode=%s"
            % (
                self._turn_label(self._active_turn_id),
                goal.yaw,
                goal.pitch,
                resolved_speed,
                mode,
            )
        )

        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=(
                feedback_callback if feedback_callback is not None else self._default_feedback
            ),
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _default_feedback(self, feedback_msg) -> None:
        """Default feedback handler."""
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            "%s HEAD_PROGRESS | %s (%0.0f%%)"
            % (
                self._turn_label(self._active_turn_id),
                feedback.status,
                feedback.progress * 100.0,
            )
        )

    def _goal_response_callback(self, future) -> None:
        """Handle goal acceptance/rejection."""
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Head-motion goal submission failed: {exc}")
            return

        if self._goal_handle is None or not self._goal_handle.accepted:
            self._node.get_logger().error(
                "%s HEAD_REJECTED | action server rejected goal"
                % self._turn_label(self._active_turn_id)
            )
            return

        self._node.get_logger().info(
            "%s HEAD_ACCEPTED | awaiting result"
            % self._turn_label(self._active_turn_id)
        )
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Handle goal result."""
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Failed to get head-motion result: {exc}")
            return

        result = wrapped_result.result
        if result.success:
            self._node.get_logger().info(
                "%s HEAD_DONE | success %s"
                % (self._turn_label(self._active_turn_id), result.message)
            )
        else:
            self._node.get_logger().error(
                "%s HEAD_DONE | failed %s"
                % (self._turn_label(self._active_turn_id), result.message)
            )

        if self._result_callback is not None:
            self._result_callback(result)

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        if self._goal_handle is None:
            self._node.get_logger().warn("No active head-motion goal to cancel")
            return

        self._node.get_logger().info("Cancelling active head-motion goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self._node.get_logger().info(
                "Head-motion goal cancel request sent"
            )
        )

    @staticmethod
    def _turn_label(turn_id: str) -> str:
        clean_turn_id = str(turn_id).strip()
        if not clean_turn_id:
            return "[turn:unknown]"
        return f"[turn:{clean_turn_id}]"
