#!/usr/bin/env python3
"""Say skill action client for dialogue manager."""

import time
from dataclasses import dataclass
from typing import Callable, Optional

from communication_skills.action import Say
from rclpy.action import ActionClient
from rclpy.node import Node
from std_skills.msg import Meta as SkillMeta
from std_skills.msg import Result as SkillResult


@dataclass
class SaySkillResult:
    """Compatibility result used by dialogue_manager."""

    success: bool
    message: str
    duration: float


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

        self._client = ActionClient(node, Say, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[SaySkillResult], None]] = None
        self._goal_start_ts = 0.0

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
        result_callback: Optional[Callable[[SaySkillResult], None]] = None,
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

        goal = Say.Goal()
        goal.meta.caller = self._node.get_name()
        goal.meta.priority = SkillMeta.NORMAL_PRIORITY
        goal.person_id = ""
        goal.group_id = ""
        goal.input = cleaned_text

        self._result_callback = result_callback
        self._goal_start_ts = time.monotonic()
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
        feedback = feedback_msg.feedback.feedback
        self._node.get_logger().info(
            f"Say skill progress: {feedback.data_str} ({feedback.data_float * 100.0:.0f}%)"
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
        duration = 0.0
        if self._goal_start_ts > 0.0:
            duration = max(0.0, time.monotonic() - self._goal_start_ts)
        success = result.result.error_code == SkillResult.ROS_ENOERR
        message = result.result.error_msg
        if not message and success:
            message = "Spoken via TTS action server"
        if success:
            self._node.get_logger().info(f"Say goal succeeded: {message}")
        else:
            self._node.get_logger().error(f"Say goal failed: {message or 'unknown'}")

        if self._result_callback is not None:
            self._result_callback(
                SaySkillResult(
                    success=success,
                    message=message,
                    duration=duration,
                )
            )

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
