#!/usr/bin/env python3
"""Say skill action client for dialogue manager."""

from typing import Callable
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from nao_skills.action import SayText


class SaySkillClient:
    """Non-blocking action client for `/skill/say`."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/say",
        default_language: str = "en-US",
        default_volume: float = 1.0,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._default_language = default_language
        self._default_volume = default_volume

        self._client = ActionClient(node, SayText, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[SayText.Result], None]] = None

        node.get_logger().info(f"Say skill client initialized: {action_name}")

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
        text: str,
        language: Optional[str] = None,
        volume: Optional[float] = None,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable[[SayText.Result], None]] = None,
    ) -> bool:
        """Send say goal asynchronously."""
        cleaned_text = text.strip()
        if not cleaned_text:
            self._node.get_logger().warn("Refusing to send empty say goal")
            return False
        if not self.server_is_ready():
            self._node.get_logger().warn(
                f"Say action server '{self._action_name}' is not ready"
            )
            return False

        resolved_language = self._default_language
        if language is not None and language.strip():
            resolved_language = language.strip()
        resolved_volume = self._default_volume if volume is None else float(volume)

        goal = SayText.Goal()
        goal.text = cleaned_text
        goal.language = resolved_language
        goal.volume = resolved_volume

        self._result_callback = result_callback
        self._node.get_logger().info(
            f"Sending say goal | language={resolved_language} volume={resolved_volume:.2f}"
        )

        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=(
                feedback_callback if feedback_callback is not None else self._default_feedback
            ),
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _default_feedback(self, feedback_msg) -> None:
        """Default feedback handler."""
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            f"Say skill progress: {feedback.status} ({feedback.progress * 100.0:.0f}%)"
        )

    def _goal_response_callback(self, future) -> None:
        """Handle goal acceptance/rejection."""
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Say goal submission failed: {exc}")
            return

        if self._goal_handle is None or not self._goal_handle.accepted:
            self._node.get_logger().error("Say goal rejected by server")
            return

        self._node.get_logger().info("Say goal accepted, awaiting result")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Handle goal result."""
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Failed to get say result: {exc}")
            return

        result = wrapped_result.result
        if result.success:
            self._node.get_logger().info(f"Say goal succeeded: {result.message}")
        else:
            self._node.get_logger().error(f"Say goal failed: {result.message}")

        if self._result_callback is not None:
            self._result_callback(result)

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        if self._goal_handle is None:
            self._node.get_logger().warn("No active say goal to cancel")
            return

        self._node.get_logger().info("Cancelling active say goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self._node.get_logger().info("Say goal cancel request sent")
        )
