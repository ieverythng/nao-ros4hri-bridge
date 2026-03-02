#!/usr/bin/env python3
"""Chat skill action client for mission controller."""

import json
from dataclasses import dataclass
from typing import Callable
from typing import Optional

from chatbot_msgs.msg import DialogueRole
from communication_skills.action import Chat
from rclpy.action import ActionClient
from rclpy.node import Node

from std_skills.msg import Meta as SkillMeta
from std_skills.msg import Result as SkillResult


@dataclass
class ChatSkillResult:
    """Compatibility result used by mission_controller."""

    success: bool
    assistant_response: str
    updated_history: list[str]
    message: str = ""


class ChatSkillClient:
    """Non-blocking action client for `/skill/chat`."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/chat",
        role_name: str = DialogueRole.DEFAULT_ROLE,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._role_name = role_name or DialogueRole.DEFAULT_ROLE

        self._client = ActionClient(node, Chat, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[ChatSkillResult], None]] = None

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
        result_callback: Optional[Callable[[ChatSkillResult], None]] = None,
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
        goal.meta.caller = self._node.get_name()
        goal.meta.priority = SkillMeta.NORMAL_PRIORITY
        goal.person_id = ""
        goal.group_id = ""
        goal.role = DialogueRole()
        goal.role.name = self._role_name
        goal.role.configuration = json.dumps(
            {
                "user_message": clean_message,
                "conversation_history": list(conversation_history),
            },
            separators=(",", ":"),
        )
        goal.initiate = False
        goal.initial_input = ""

        self._result_callback = result_callback
        self._node.get_logger().info(
            "Sending chat goal with %d history entries (role=%s)"
            % (len(conversation_history), goal.role.name)
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
        feedback = feedback_msg.feedback.feedback
        self._node.get_logger().info(
            "Chat skill progress: %s (%0.0f%%)"
            % (feedback.data_str, feedback.data_float * 100.0)
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
        parsed = self._parse_role_results(result.role_results)
        assistant_response = parsed.get("assistant_response", "")
        updated_history = parsed.get("updated_history", [])
        if not isinstance(updated_history, list):
            updated_history = []
        updated_history = [
            str(entry).strip() for entry in updated_history if str(entry).strip()
        ]

        success = bool(
            parsed.get("success", False)
            or result.result.error_code == SkillResult.ROS_ENOERR
        )
        message = str(parsed.get("message", result.result.error_msg))
        compatibility_result = ChatSkillResult(
            success=success,
            assistant_response=str(assistant_response).strip(),
            updated_history=updated_history,
            message=message.strip(),
        )

        if compatibility_result.success:
            self._node.get_logger().info("Chat goal succeeded")
        else:
            self._node.get_logger().warn(
                "Chat goal finished with failure status: %s"
                % (compatibility_result.message or "unknown")
            )

        if self._result_callback is not None:
            self._result_callback(compatibility_result)

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

    @staticmethod
    def _parse_role_results(payload: str) -> dict:
        if not payload:
            return {}
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            return {}
        if not isinstance(parsed, dict):
            return {}
        return parsed
