#!/usr/bin/env python3
"""Servers for replay-motion and temporary do-posture compatibility."""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from nao_skills.action import DoPosture, ReplayMotion

try:
    import qi
except ImportError:  # pragma: no cover - depends on runtime environment
    qi = None


class ReplayMotionSkillServer(Node):
    """Expose `/skill/replay_motion` and the temporary `/skill/do_posture` bridge."""

    _MOTION_ALIASES = {
        "stand": ("stand", "Stand"),
        "standinit": ("standinit", "StandInit"),
        "standfull": ("stand", "Stand"),
        "standzero": ("standzero", "StandZero"),
        "sit": ("sit", "Sit"),
        "sitrelax": ("sitrelax", "SitRelax"),
        "kneel": ("kneel", "Crouch"),
        "crouch": ("crouch", "Crouch"),
        "lyingback": ("lyingback", "LyingBack"),
        "lyingbelly": ("lyingbelly", "LyingBelly"),
    }

    def __init__(self) -> None:
        super().__init__("replay_motion_skill_server")

        self.declare_parameter("nao_ip", "172.26.112.62")
        self.declare_parameter("nao_port", 9559)
        self.declare_parameter("action_name", "/skill/replay_motion")
        self.declare_parameter("posture_compat_action_name", "/skill/do_posture")
        self.declare_parameter("default_speed", 0.8)
        self.declare_parameter("reconnect_on_failure", True)
        self.declare_parameter("fallback_to_posture_topic", True)
        self.declare_parameter("posture_command_topic", "/chatbot/posture_command")

        self.nao_ip = str(self.get_parameter("nao_ip").value)
        self.nao_port = int(self.get_parameter("nao_port").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.posture_compat_action_name = str(
            self.get_parameter("posture_compat_action_name").value
        )
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.reconnect_on_failure = bool(
            self.get_parameter("reconnect_on_failure").value
        )
        self.fallback_to_posture_topic = bool(
            self.get_parameter("fallback_to_posture_topic").value
        )
        self.posture_command_topic = str(
            self.get_parameter("posture_command_topic").value
        )

        self._session = None
        self._posture_proxy = None
        self._execution_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._posture_command_publisher = self.create_publisher(
            String, self.posture_command_topic, 10
        )
        has_naoqi_connection = self._connect_naoqi()

        self._replay_motion_server = ActionServer(
            self,
            ReplayMotion,
            self.action_name,
            execute_callback=self.execute_replay_callback,
            goal_callback=self.replay_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self._posture_compat_server = ActionServer(
            self,
            DoPosture,
            self.posture_compat_action_name,
            execute_callback=self.execute_posture_compat_callback,
            goal_callback=self.posture_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        mode = "direct_naoqi"
        if not has_naoqi_connection:
            mode = "topic_fallback" if self.fallback_to_posture_topic else "disconnected"
        self.get_logger().info(
            "replay_motion_skill_server ready | action:%s posture_compat:%s mode:%s"
            % (
                self.action_name,
                self.posture_compat_action_name,
                mode,
            )
        )

    @staticmethod
    def _normalize_name(name: str) -> str:
        return "".join(str(name).lower().split())

    def _resolve_motion(self, name: str) -> Optional[tuple[str, str]]:
        normalized = self._normalize_name(name)
        if not normalized:
            return None
        return self._MOTION_ALIASES.get(normalized)

    def _resolve_speed(self, requested_speed: float) -> Optional[float]:
        speed = float(requested_speed)
        if speed <= 0.0:
            speed = self.default_speed
        if speed <= 0.0 or speed > 1.0:
            return None
        return speed

    def _connect_naoqi(self) -> bool:
        if qi is None:
            self._session = None
            self._posture_proxy = None
            self.get_logger().warn(
                "Python qi module is missing; replay motion will use topic fallback when possible"
            )
            return False

        url = f"tcp://{self.nao_ip}:{self.nao_port}"
        try:
            self._session = qi.Session()
            self._session.connect(url)
            self._posture_proxy = self._session.service("ALRobotPosture")
            self.get_logger().info(f"Connected to NAOqi at {url}")
            return True
        except Exception as exc:  # pragma: no cover - runtime bound
            self._session = None
            self._posture_proxy = None
            self.get_logger().error(f"NAOqi connection failed ({url}): {exc}")
            return False

    def _ensure_connection(self) -> bool:
        if qi is None:
            return False
        if self._posture_proxy is not None:
            return True
        return self._connect_naoqi()

    def replay_goal_callback(self, goal_request: ReplayMotion.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            return GoalResponse.REJECT
        resolved_motion = self._resolve_motion(goal_request.motion_name)
        speed = self._resolve_speed(goal_request.speed)
        if resolved_motion is None or speed is None:
            return GoalResponse.REJECT
        if not self._ensure_connection() and not self.fallback_to_posture_topic:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def posture_goal_callback(self, goal_request: DoPosture.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            return GoalResponse.REJECT
        resolved_motion = self._resolve_motion(goal_request.posture_name)
        speed = self._resolve_speed(goal_request.speed)
        if resolved_motion is None or speed is None:
            return GoalResponse.REJECT
        if not self._ensure_connection() and not self.fallback_to_posture_topic:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for replay motion goal")
        return CancelResponse.ACCEPT

    async def execute_replay_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._replay_result(False, "Another motion is already executing", 0.0)
        try:
            goal = goal_handle.request
            return self._execute_motion(
                goal_handle=goal_handle,
                requested_name=goal.motion_name,
                requested_speed=goal.speed,
                feedback_builder=self._publish_replay_feedback,
                result_builder=self._replay_result,
            )
        finally:
            self._execution_lock.release()

    async def execute_posture_compat_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._posture_result(False, "Another motion is already executing", 0.0)
        try:
            goal = goal_handle.request
            return self._execute_motion(
                goal_handle=goal_handle,
                requested_name=goal.posture_name,
                requested_speed=goal.speed,
                feedback_builder=self._publish_posture_feedback,
                result_builder=self._posture_result,
            )
        finally:
            self._execution_lock.release()

    def _execute_motion(
        self,
        *,
        goal_handle,
        requested_name: str,
        requested_speed: float,
        feedback_builder,
        result_builder,
    ):
        start_time = time.monotonic()
        resolved_motion = self._resolve_motion(requested_name)
        speed = self._resolve_speed(requested_speed)
        if resolved_motion is None or speed is None:
            goal_handle.abort()
            return result_builder(
                False,
                "Goal became invalid before execution",
                time.monotonic() - start_time,
            )

        motion_name, posture_name = resolved_motion
        feedback_builder(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result_builder(False, "Cancelled before execution", 0.0)

        feedback_builder(goal_handle, "executing", 0.4)
        try:
            execution_mode = "direct_naoqi"
            if self._ensure_connection():
                self._execute_posture_with_retry(posture_name, speed)
            else:
                self._execute_posture_via_topic_fallback(posture_name)
                execution_mode = "topic_fallback"
        except Exception as exc:  # pragma: no cover - runtime bound
            duration = time.monotonic() - start_time
            goal_handle.abort()
            return result_builder(
                False,
                f"Failed to execute motion '{motion_name}': {exc}",
                duration,
            )

        if goal_handle.is_cancel_requested:
            duration = time.monotonic() - start_time
            goal_handle.canceled()
            return result_builder(
                False,
                "Cancelled during execution (motion may have completed)",
                duration,
            )

        feedback_builder(goal_handle, "completing", 1.0)
        duration = time.monotonic() - start_time
        goal_handle.succeed()
        return result_builder(
            True,
            f"Executed motion '{motion_name}' via {execution_mode}",
            duration,
        )

    def _execute_posture_with_retry(self, posture_name: str, speed: float) -> None:
        if not self._ensure_connection():
            raise RuntimeError("NAOqi is disconnected")

        try:
            ok = bool(self._posture_proxy.goToPosture(posture_name, speed))
            if not ok:
                raise RuntimeError("ALRobotPosture.goToPosture returned false")
            return
        except Exception as exc:
            if not self.reconnect_on_failure:
                raise RuntimeError(str(exc)) from exc

        if not self._connect_naoqi():
            raise RuntimeError("Reconnect failed")

        ok = bool(self._posture_proxy.goToPosture(posture_name, speed))
        if not ok:
            raise RuntimeError("ALRobotPosture.goToPosture returned false after reconnect")

    def _execute_posture_via_topic_fallback(self, posture_name: str) -> None:
        if not self.fallback_to_posture_topic:
            raise RuntimeError("Fallback to posture topic is disabled")
        msg = String()
        msg.data = posture_name
        self._posture_command_publisher.publish(msg)

    @staticmethod
    def _publish_replay_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = ReplayMotion.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _publish_posture_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = DoPosture.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _replay_result(success: bool, message: str, duration: float):
        result = ReplayMotion.Result()
        result.success = bool(success)
        result.message = message
        result.duration = float(duration)
        return result

    @staticmethod
    def _posture_result(success: bool, message: str, duration: float):
        result = DoPosture.Result()
        result.success = bool(success)
        result.message = message
        result.duration = float(duration)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplayMotionSkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
