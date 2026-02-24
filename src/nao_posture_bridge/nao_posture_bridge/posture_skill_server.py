#!/usr/bin/env python3
"""Action server wrapper for NAO posture execution."""

import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from nao_skills.action import DoPosture

try:
    import qi
except ImportError:  # pragma: no cover - depends on runtime environment
    qi = None


class PostureSkillServer(Node):
    """Serve `/skill/do_posture` goals using NAOqi `ALRobotPosture`."""

    _POSTURE_ALIASES = {
        "stand": "Stand",
        "standinit": "StandInit",
        "standfull": "Stand",
        "standzero": "StandZero",
        "sit": "Sit",
        "sitrelax": "SitRelax",
        "kneel": "Crouch",
        "crouch": "Crouch",
        "lyingback": "LyingBack",
        "lyingbelly": "LyingBelly",
    }

    def __init__(self) -> None:
        super().__init__("posture_skill_server")

        self.declare_parameter("nao_ip", "172.26.112.62")
        self.declare_parameter("nao_port", 9559)
        self.declare_parameter("action_name", "/skill/do_posture")
        self.declare_parameter("default_speed", 0.8)
        self.declare_parameter("reconnect_on_failure", True)
        self.declare_parameter("fallback_to_posture_topic", True)
        self.declare_parameter("posture_command_topic", "/chatbot/posture_command")

        self.nao_ip = str(self.get_parameter("nao_ip").value)
        self.nao_port = int(self.get_parameter("nao_port").value)
        self.action_name = str(self.get_parameter("action_name").value)
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

        self._action_server = ActionServer(
            self,
            DoPosture,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        mode = "direct_naoqi"
        if not has_naoqi_connection:
            mode = "topic_fallback" if self.fallback_to_posture_topic else "disconnected"
        self.get_logger().info(
            f"posture_skill_server ready | action:{self.action_name} "
            f"nao:{self.nao_ip}:{self.nao_port} default_speed:{self.default_speed:.2f} "
            f"mode:{mode}"
        )

    @staticmethod
    def _normalize_name(posture_name: str) -> str:
        return "".join(posture_name.lower().split())

    def _resolve_posture_name(self, posture_name: str) -> Optional[str]:
        normalized = self._normalize_name(posture_name)
        if not normalized:
            return None
        return self._POSTURE_ALIASES.get(normalized)

    def _resolve_speed(self, requested_speed: float) -> Optional[float]:
        speed = float(requested_speed)
        if speed <= 0.0:
            speed = self.default_speed
        if speed <= 0.0 or speed > 1.0:
            return None
        return speed

    def _connect_naoqi(self) -> bool:
        if qi is None:
            message = (
                "Python qi module is missing; direct NAOqi mode is unavailable"
            )
            if self.fallback_to_posture_topic:
                self.get_logger().warn(f"{message}. Using topic fallback mode.")
            else:
                self.get_logger().error(f"{message}.")
            self._session = None
            self._posture_proxy = None
            return False

        url = f"tcp://{self.nao_ip}:{self.nao_port}"
        try:
            self._session = qi.Session()
            self._session.connect(url)
            self._posture_proxy = self._session.service("ALRobotPosture")
            self.get_logger().info(f"Connected to NAOqi at {url}")
            return True
        except Exception as exc:  # pragma: no cover - hardware/runtime bound
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

    def goal_callback(self, goal_request: DoPosture.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            self.get_logger().warn("Rejected posture goal because another goal is running")
            return GoalResponse.REJECT

        posture_name = self._resolve_posture_name(goal_request.posture_name)
        if posture_name is None:
            self.get_logger().warn(
                f"Rejected posture goal with unknown posture "
                f"'{goal_request.posture_name}'"
            )
            return GoalResponse.REJECT

        speed = self._resolve_speed(goal_request.speed)
        if speed is None:
            self.get_logger().warn(
                f"Rejected posture goal with invalid speed "
                f"{float(goal_request.speed):.3f} (valid range: 0.0-1.0, "
                f"with 0.0 meaning default)"
            )
            return GoalResponse.REJECT

        if not self._ensure_connection():
            if not self.fallback_to_posture_topic:
                self.get_logger().warn(
                    "Rejected posture goal because NAOqi is not connected and "
                    "fallback_to_posture_topic is false"
                )
                return GoalResponse.REJECT
            self.get_logger().warn(
                f"Accepting posture goal using topic fallback "
                f"'{self.posture_command_topic}'"
            )

        self.get_logger().info(
            f"Accepted posture goal: '{goal_request.posture_name}' -> "
            f"'{posture_name}' @ {speed:.2f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for posture goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(
                False,
                "Another posture goal is already executing",
                0.0,
            )

        try:
            return self._execute_locked(goal_handle)
        finally:
            self._execution_lock.release()

    def _execute_locked(self, goal_handle):
        start_time = time.monotonic()
        goal = goal_handle.request
        posture_name = self._resolve_posture_name(goal.posture_name)
        speed = self._resolve_speed(goal.speed)

        if posture_name is None or speed is None:
            goal_handle.abort()
            return self._result(
                False,
                "Goal became invalid before execution",
                time.monotonic() - start_time,
            )

        self._publish_feedback(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "Cancelled before execution", 0.0)

        self._publish_feedback(goal_handle, "executing", 0.3)
        try:
            execution_mode = "direct_naoqi"
            if self._ensure_connection():
                self._execute_posture_with_retry(posture_name, speed)
            else:
                self._execute_posture_via_topic_fallback(posture_name)
                execution_mode = "topic_fallback"
        except Exception as exc:  # pragma: no cover - hardware/runtime bound
            duration = time.monotonic() - start_time
            goal_handle.abort()
            return self._result(
                False,
                f"Failed to execute posture '{posture_name}': {exc}",
                duration,
            )

        if goal_handle.is_cancel_requested:
            duration = time.monotonic() - start_time
            goal_handle.canceled()
            return self._result(
                False,
                "Cancelled during execution (motion may have completed)",
                duration,
            )

        self._publish_feedback(goal_handle, "completing", 0.9)
        duration = time.monotonic() - start_time
        goal_handle.succeed()
        return self._result(
            True,
            f"Executed posture '{posture_name}' via {execution_mode}",
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
            self.get_logger().warn(f"Posture call failed, trying reconnect: {exc}")

        if not self._connect_naoqi():
            raise RuntimeError("Reconnect failed")

        ok = bool(self._posture_proxy.goToPosture(posture_name, speed))
        if not ok:
            raise RuntimeError("ALRobotPosture.goToPosture returned false after reconnect")

    def _execute_posture_via_topic_fallback(self, posture_name: str) -> None:
        if not self.fallback_to_posture_topic:
            raise RuntimeError(
                "NAOqi is disconnected and fallback_to_posture_topic is false"
            )
        msg = String()
        msg.data = posture_name
        self._posture_command_publisher.publish(msg)
        self.get_logger().warn(
            f"Forwarded posture '{posture_name}' to topic "
            f"'{self.posture_command_topic}' because direct NAOqi is unavailable"
        )

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = DoPosture.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, message: str, duration: float):
        result = DoPosture.Result()
        result.success = bool(success)
        result.message = message
        result.duration = float(duration)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PostureSkillServer()
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
