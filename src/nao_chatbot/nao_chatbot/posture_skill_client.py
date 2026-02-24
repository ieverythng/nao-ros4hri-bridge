#!/usr/bin/env python3
"""Posture skill action client for mission controller."""

from typing import Callable
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from nao_skills.action import DoPosture


class PostureSkillClient:
    """Non-blocking action client for posture skills."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/do_posture",
        default_speed: float = 0.8,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._default_speed = default_speed

        self._client = ActionClient(node, DoPosture, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[DoPosture.Result], None]] = None

        node.get_logger().info(f"Posture skill client initialized: {action_name}")

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
        posture_name: str,
        speed: Optional[float] = None,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable[[DoPosture.Result], None]] = None,
    ) -> None:
        """Send posture goal asynchronously."""
        cleaned_posture_name = posture_name.strip()
        if not cleaned_posture_name:
            self._node.get_logger().warn("Refusing to send empty posture goal")
            return
        if not self.server_is_ready():
            self._node.get_logger().warn(
                f"Posture action server '{self._action_name}' is not ready"
            )
            return

        resolved_speed = self._default_speed if speed is None else float(speed)

        goal = DoPosture.Goal()
        goal.posture_name = cleaned_posture_name
        goal.speed = resolved_speed

        self._result_callback = result_callback

        self._node.get_logger().info(
            f"Sending posture goal: {cleaned_posture_name} @ {resolved_speed:.2f}"
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
            f"Posture skill progress: {feedback.status} ({feedback.progress * 100.0:.0f}%)"
        )

    def _goal_response_callback(self, future) -> None:
        """Handle goal acceptance/rejection."""
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Posture goal submission failed: {exc}")
            return

        if self._goal_handle is None or not self._goal_handle.accepted:
            self._node.get_logger().error("Posture goal rejected by server")
            return

        self._node.get_logger().info("Posture goal accepted, awaiting result")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Handle goal result."""
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Failed to get posture result: {exc}")
            return

        result = wrapped_result.result
        if result.success:
            self._node.get_logger().info(f"Posture goal succeeded: {result.message}")
        else:
            self._node.get_logger().error(f"Posture goal failed: {result.message}")

        if self._result_callback is not None:
            self._result_callback(result)

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        if self._goal_handle is None:
            self._node.get_logger().warn("No active posture goal to cancel")
            return

        self._node.get_logger().info("Cancelling active posture goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self._node.get_logger().info("Posture goal cancel request sent")
        )
