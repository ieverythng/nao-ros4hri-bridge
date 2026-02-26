#!/usr/bin/env python3
"""Chat skill action client for mission controller."""

from typing import Callable
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from nao_skills.action import Chat


class ChatSkillClient:
    """Non-blocking action client for `/skill/chat`."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/chat",
    ) -> None:
        self._node = node
        self._action_name = action_name

        self._client = ActionClient(node, Chat, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[Chat.Result], None]] = None

        node.get_logger().info(f"Chat skill client initialized: {action_name}")

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
        user_message: str,
        conversation_history: list[str],
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable[[Chat.Result], None]] = None,
    ) -> bool:
        """Send chat goal asynchronously."""
        clean_message = user_message.strip()
        if not clean_message:
            self._node.get_logger().warn("Refusing to send empty chat goal")
            return False
        if not self.server_is_ready():
            self._node.get_logger().warn(
                f"Chat action server '{self._action_name}' is not ready"
            )
            return False

        goal = Chat.Goal()
        goal.user_message = clean_message
        goal.conversation_history = list(conversation_history)

        self._result_callback = result_callback
        self._node.get_logger().info(
            "Sending chat goal with %d history entries"
            % len(goal.conversation_history)
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
            "Chat skill progress: %s (%0.0f%%)"
            % (feedback.status, feedback.progress * 100.0)
        )

    def _goal_response_callback(self, future) -> None:
        """Handle goal acceptance/rejection."""
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Chat goal submission failed: {exc}")
            return

        if self._goal_handle is None or not self._goal_handle.accepted:
            self._node.get_logger().error("Chat goal rejected by server")
            return

        self._node.get_logger().info("Chat goal accepted, awaiting result")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Handle goal result."""
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Failed to get chat result: {exc}")
            return

        result = wrapped_result.result
        if result.success:
            self._node.get_logger().info("Chat goal succeeded")
        else:
            self._node.get_logger().warn("Chat goal finished with failure status")

        if self._result_callback is not None:
            self._result_callback(result)

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        if self._goal_handle is None:
            self._node.get_logger().warn("No active chat goal to cancel")
            return

        self._node.get_logger().info("Cancelling active chat goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self._node.get_logger().info("Chat goal cancel request sent")
        )
