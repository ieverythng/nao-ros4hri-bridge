#!/usr/bin/env python3
"""Servers for replay-motion and temporary do-posture compatibility."""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nao_skills.action import DoPosture, ReplayMotion
except ImportError:  # pragma: no cover - import-light unit tests
    class ReplayMotion:  # type: ignore[no-redef]
        class Goal:
            def __init__(self) -> None:
                self.motion_name = ""
                self.speed = 0.0

        class Feedback:
            def __init__(self) -> None:
                self.status = ""
                self.progress = 0.0

        class Result:
            def __init__(self) -> None:
                self.success = False
                self.message = ""
                self.duration = 0.0

    class DoPosture:  # type: ignore[no-redef]
        class Goal:
            def __init__(self) -> None:
                self.posture_name = ""
                self.speed = 0.0

        class Feedback:
            def __init__(self) -> None:
                self.status = ""
                self.progress = 0.0

        class Result:
            def __init__(self) -> None:
                self.success = False
                self.message = ""
                self.duration = 0.0

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

        self.declare_parameter("nao_ip", "127.0.0.1")
        self.declare_parameter("nao_port", 9559)
        self.declare_parameter("action_name", "/skill/replay_motion")
        self.declare_parameter("posture_compat_action_name", "/skill/do_posture")
        self.declare_parameter("default_speed", 0.8)
        self.declare_parameter("reconnect_on_failure", True)
        self.declare_parameter("fallback_to_posture_topic", True)
        self.declare_parameter("allow_open_loop_without_naoqi", False)
        self.declare_parameter("posture_command_topic", "/chatbot/posture_command")
        self.declare_parameter("posture_result_topic", "/chatbot/posture_command_result")
        self.declare_parameter("posture_result_timeout_sec", 5.0)

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
        self.allow_open_loop_without_naoqi = bool(
            self.get_parameter("allow_open_loop_without_naoqi").value
        )
        self.posture_command_topic = str(
            self.get_parameter("posture_command_topic").value
        )
        self.posture_result_topic = str(
            self.get_parameter("posture_result_topic").value
        )
        self.posture_result_timeout_sec = max(
            0.1,
            float(self.get_parameter("posture_result_timeout_sec").value),
        )

        self._session = None
        self._posture_proxy = None
        self._execution_lock = threading.Lock()
        self._posture_result_lock = threading.Lock()
        self._posture_result_event = threading.Event()
        self._latest_posture_result: dict | None = None
        self._callback_group = ReentrantCallbackGroup()
        self._posture_command_publisher = self.create_publisher(
            String, self.posture_command_topic, 10
        )
        self._posture_result_subscription = self.create_subscription(
            String,
            self.posture_result_topic,
            self._on_posture_result,
            10,
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
            "replay_motion_skill_server ready | action:%s posture_compat:%s mode:%s result_topic:%s"
            % (
                self.action_name,
                self.posture_compat_action_name,
                mode,
                self.posture_result_topic,
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
        if not self._can_accept_motion_goal(
            requested_name=goal_request.motion_name,
            requested_speed=goal_request.speed,
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def posture_goal_callback(self, goal_request: DoPosture.Goal) -> GoalResponse:
        if not self._can_accept_motion_goal(
            requested_name=goal_request.posture_name,
            requested_speed=goal_request.speed,
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for replay motion goal")
        return CancelResponse.ACCEPT

    def _can_accept_motion_goal(
        self,
        *,
        requested_name: str,
        requested_speed: float,
    ) -> bool:
        if self._execution_lock.locked():
            return False
        if self._resolve_motion(requested_name) is None:
            return False
        if self._resolve_speed(requested_speed) is None:
            return False
        if (
            not self._ensure_connection()
            and not self.fallback_to_posture_topic
            and not self.allow_open_loop_without_naoqi
        ):
            return False
        return True

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
            execution_note = ""
            if self._ensure_connection():
                self._execute_posture_with_retry(posture_name, speed)
            elif self.fallback_to_posture_topic and self._posture_bridge_available():
                try:
                    execution_note = self._execute_posture_via_topic_fallback(posture_name)
                    execution_mode = "topic_fallback"
                except RuntimeError:
                    if not self.allow_open_loop_without_naoqi:
                        raise
                    execution_mode = "open_loop"
            elif self.allow_open_loop_without_naoqi:
                execution_mode = "open_loop"
            else:
                raise RuntimeError("No NAOqi or posture bridge execution path is available")
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
            self._success_message(
                motion_name=motion_name,
                execution_mode=execution_mode,
                execution_note=execution_note,
            ),
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

    def _execute_posture_via_topic_fallback(self, posture_name: str) -> str:
        if not self.fallback_to_posture_topic:
            raise RuntimeError("Fallback to posture topic is disabled")
        with self._posture_result_lock:
            self._latest_posture_result = None
        self._posture_result_event.clear()
        msg = String()
        msg.data = posture_name
        self._posture_command_publisher.publish(msg)
        deadline = time.monotonic() + self.posture_result_timeout_sec
        while time.monotonic() <= deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._posture_result_event.wait(timeout=min(0.1, remaining))
            self._posture_result_event.clear()
            with self._posture_result_lock:
                result_payload = dict(self._latest_posture_result or {})
            if not _posture_result_matches(result_payload, posture_name):
                continue
            if bool(result_payload.get("success", False)):
                return str(result_payload.get("message", "")).strip()
            raise RuntimeError(
                str(result_payload.get("message", "")).strip()
                or f"Posture bridge reported failure for '{posture_name}'"
            )
        raise RuntimeError(
            f"Timed out waiting for posture bridge result on '{self.posture_result_topic}'"
        )

    def _posture_bridge_available(self) -> bool:
        return self._posture_command_publisher.get_subscription_count() > 0

    def _on_posture_result(self, msg: String) -> None:
        payload = _parse_posture_result_message(msg.data)
        if not payload:
            return
        with self._posture_result_lock:
            self._latest_posture_result = payload
        self._posture_result_event.set()

    @staticmethod
    def _success_message(
        *,
        motion_name: str,
        execution_mode: str,
        execution_note: str,
    ) -> str:
        clean_note = str(execution_note or "").strip()
        if execution_mode == "topic_fallback" and clean_note:
            return (
                f"Executed motion '{motion_name}' via {execution_mode}"
                f" ({clean_note})"
            )
        return f"Executed motion '{motion_name}' via {execution_mode}"

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


def _parse_posture_result_message(payload: str) -> dict:
    try:
        parsed = json.loads(str(payload or "").strip())
    except json.JSONDecodeError:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _posture_result_matches(result_payload: dict, posture_name: str) -> bool:
    if not isinstance(result_payload, dict):
        return False
    expected = ReplayMotionSkillServer._normalize_name(posture_name)
    if not expected:
        return False
    candidates = (
        result_payload.get("posture_name", ""),
        result_payload.get("normalized_command", ""),
        result_payload.get("command", ""),
    )
    return any(
        ReplayMotionSkillServer._normalize_name(candidate) == expected
        for candidate in candidates
        if str(candidate).strip()
    )


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
