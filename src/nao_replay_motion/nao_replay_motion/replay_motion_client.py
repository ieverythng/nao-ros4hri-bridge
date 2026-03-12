#!/usr/bin/env python3
"""Action client for `/skill/replay_motion`."""

from typing import Callable, Optional

from rclpy.action import ActionClient
from rclpy.node import Node

from nao_skills.action import ReplayMotion


class ReplayMotionSkillClient:
    """Non-blocking client for replay-motion requests."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/skill/replay_motion",
        default_speed: float = 0.8,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._default_speed = default_speed
        self._client = ActionClient(node, ReplayMotion, action_name)
        self._goal_handle = None
        self._result_callback: Optional[Callable[[ReplayMotion.Result], None]] = None
        self._active_turn_id = ""

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def server_is_ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal(
        self,
        motion_name: str,
        speed: Optional[float] = None,
        turn_id: str = "",
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable[[ReplayMotion.Result], None]] = None,
    ) -> None:
        clean_name = motion_name.strip()
        if not clean_name:
            self._node.get_logger().warn("Refusing to send empty replay motion goal")
            return
        if not self.server_is_ready():
            self._node.get_logger().warn(
                "Replay motion action server '%s' is not ready" % self._action_name
            )
            return

        goal = ReplayMotion.Goal()
        goal.motion_name = clean_name
        goal.speed = self._default_speed if speed is None else float(speed)
        self._result_callback = result_callback
        self._active_turn_id = str(turn_id).strip()

        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=feedback_callback or self._default_feedback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _default_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            "%s REPLAY_PROGRESS | %s (%0.0f%%)"
            % (
                self._turn_label(self._active_turn_id),
                feedback.status,
                feedback.progress * 100.0,
            )
        )

    def _goal_response_callback(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Replay motion goal submission failed: {exc}")
            return

        if self._goal_handle is None or not self._goal_handle.accepted:
            self._node.get_logger().error(
                "%s REPLAY_REJECTED | action server rejected goal"
                % self._turn_label(self._active_turn_id)
            )
            return

        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"Failed to get replay motion result: {exc}")
            return

        result = wrapped_result.result
        if self._result_callback is not None:
            self._result_callback(result)

    @staticmethod
    def _turn_label(turn_id: str) -> str:
        clean_turn_id = str(turn_id).strip()
        if not clean_turn_id:
            return "[turn:unknown]"
        return f"[turn:{clean_turn_id}]"
