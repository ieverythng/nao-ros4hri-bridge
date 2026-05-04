#!/usr/bin/env python3
"""Action server wrapper for retained NAO head motion via joint-angle topics."""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    from nao_skills.action import DoHeadMotion
except ImportError:  # pragma: no cover - import-light unit tests
    class DoHeadMotion:  # type: ignore[no-redef]
        class Goal:
            def __init__(self) -> None:
                self.yaw = 0.0
                self.pitch = 0.0
                self.speed = 0.0
                self.relative = False

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
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - depends on runtime environment
    JointAnglesWithSpeed = None

try:
    from sensor_msgs.msg import JointState
except ImportError:  # pragma: no cover - depends on runtime environment
    JointState = None


class HeadMotionSkillServer(Node):
    """Serve `/skill/do_head_motion` goals via `/joint_angles` publishing."""

    _HEAD_JOINTS = ("HeadYaw", "HeadPitch")

    def __init__(self) -> None:
        super().__init__("head_motion_skill_server")

        self.declare_parameter("action_name", "/skill/do_head_motion")
        self.declare_parameter("default_speed", 0.2)
        self.declare_parameter("yaw_min", -2.0857)
        self.declare_parameter("yaw_max", 2.0857)
        self.declare_parameter("pitch_min", -0.6720)
        self.declare_parameter("pitch_max", 0.5149)
        self.declare_parameter("joint_angles_topic", "/joint_angles")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("require_joint_angles_subscribers", False)
        self.declare_parameter("joint_state_wait_sec", 1.0)
        self.declare_parameter("convergence_timeout_sec", 3.0)
        self.declare_parameter("convergence_tolerance_rad", 0.08)
        self.declare_parameter("retry_on_convergence_timeout", True)
        self.declare_parameter("retry_convergence_timeout_sec", 1.5)
        self.declare_parameter("allow_open_loop_without_joint_state", False)
        self.declare_parameter("assume_success_on_convergence_timeout", False)

        self.action_name = str(self.get_parameter("action_name").value)
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.yaw_min = float(self.get_parameter("yaw_min").value)
        self.yaw_max = float(self.get_parameter("yaw_max").value)
        self.pitch_min = float(self.get_parameter("pitch_min").value)
        self.pitch_max = float(self.get_parameter("pitch_max").value)
        self.joint_angles_topic = str(self.get_parameter("joint_angles_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.require_joint_angles_subscribers = bool(
            self.get_parameter("require_joint_angles_subscribers").value
        )
        self.joint_state_wait_sec = max(
            0.0, float(self.get_parameter("joint_state_wait_sec").value)
        )
        self.convergence_timeout_sec = max(
            0.1, float(self.get_parameter("convergence_timeout_sec").value)
        )
        self.convergence_tolerance_rad = max(
            1e-3, float(self.get_parameter("convergence_tolerance_rad").value)
        )
        self.retry_on_convergence_timeout = bool(
            self.get_parameter("retry_on_convergence_timeout").value
        )
        self.retry_convergence_timeout_sec = max(
            0.1,
            float(self.get_parameter("retry_convergence_timeout_sec").value),
        )
        self.allow_open_loop_without_joint_state = bool(
            self.get_parameter("allow_open_loop_without_joint_state").value
        )
        self.assume_success_on_convergence_timeout = bool(
            self.get_parameter("assume_success_on_convergence_timeout").value
        )

        if JointAnglesWithSpeed is None or JointState is None:
            raise RuntimeError(
                "HeadMotionSkillServer requires JointAnglesWithSpeed and JointState messages"
            )

        self._execution_lock = threading.Lock()
        self._joint_state_lock = threading.Lock()
        self._head_joint_positions: dict[str, float] = {}
        self._last_joint_state_monotonic = 0.0
        self._head_joint_state_event = threading.Event()
        self._callback_group = ReentrantCallbackGroup()
        self._joint_angles_publisher = self.create_publisher(
            JointAnglesWithSpeed, self.joint_angles_topic, 10
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._on_joint_state,
            10,
        )
        self._action_server = ActionServer(
            self,
            DoHeadMotion,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

    def _resolve_speed(self, requested_speed: float) -> Optional[float]:
        speed = float(requested_speed)
        if speed <= 0.0:
            speed = self.default_speed
        if speed <= 0.0 or speed > 1.0 or math.isnan(speed):
            return None
        return speed

    def _validate_angles(self, yaw: float, pitch: float, *, relative: bool) -> Optional[str]:
        if not math.isfinite(yaw) or not math.isfinite(pitch):
            return "Yaw/pitch must be finite numbers"
        if relative:
            return None
        if yaw < self.yaw_min or yaw > self.yaw_max:
            return (
                f"Yaw out of range [{self.yaw_min:.4f}, {self.yaw_max:.4f}] "
                f"(got {yaw:.4f})"
            )
        if pitch < self.pitch_min or pitch > self.pitch_max:
            return (
                f"Pitch out of range [{self.pitch_min:.4f}, {self.pitch_max:.4f}] "
                f"(got {pitch:.4f})"
            )
        return None

    def _publisher_ready(self) -> bool:
        if not self.require_joint_angles_subscribers:
            return True
        return self._joint_angles_publisher.get_subscription_count() > 0

    def goal_callback(self, goal_request: DoHeadMotion.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            return GoalResponse.REJECT
        speed = self._resolve_speed(goal_request.speed)
        if speed is None:
            return GoalResponse.REJECT
        validation_error = self._validate_angles(
            float(goal_request.yaw),
            float(goal_request.pitch),
            relative=bool(goal_request.relative),
        )
        if validation_error or not self._publisher_ready():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, "Another head-motion goal is already executing", 0.0)
        try:
            return self._execute_locked(goal_handle)
        finally:
            self._execution_lock.release()

    def _execute_locked(self, goal_handle):
        start_time = time.monotonic()
        goal = goal_handle.request
        yaw = float(goal.yaw)
        pitch = float(goal.pitch)
        relative = bool(goal.relative)
        speed = self._resolve_speed(goal.speed)
        validation_error = self._validate_angles(yaw, pitch, relative=relative)
        self.get_logger().info(
            "HEAD_MOTION goal received | yaw=%.3f pitch=%.3f speed=%.3f relative=%s"
            % (yaw, pitch, speed or -1.0, relative)
        )

        if speed is None or validation_error:
            self.get_logger().warn("HEAD_MOTION rejected before execution | %s" % (validation_error or "Invalid speed"))
            goal_handle.abort()
            reason = validation_error or "Invalid speed"
            return self._result(
                False,
                f"Goal became invalid before execution: {reason}",
                time.monotonic() - start_time,
            )
        if not self._publisher_ready():
            goal_handle.abort()
            return self._result(
                False,
                f"Topic '{self.joint_angles_topic}' has no subscribers",
                time.monotonic() - start_time,
            )

        current_state = self._wait_for_head_state(self.joint_state_wait_sec)
        if current_state is None:
            if self.allow_open_loop_without_joint_state and not relative:
                target_yaw, target_pitch = yaw, pitch
                return self._execute_open_loop(
                    goal_handle,
                    start_time=start_time,
                    yaw=yaw,
                    pitch=pitch,
                    speed=speed,
                    relative=relative,
                    target_yaw=target_yaw,
                    target_pitch=target_pitch,
                    reason=(
                        f"No recent head joint state available on '{self.joint_states_topic}'"
                    ),
                )
            goal_handle.abort()
            reason = (
                f"No recent head joint state available on '{self.joint_states_topic}'"
            )
            self.get_logger().warn("HEAD_MOTION failed | %s" % reason)
            return self._result(False, reason, time.monotonic() - start_time)

        target_yaw, target_pitch = self._resolve_target_angles(
            yaw=yaw,
            pitch=pitch,
            relative=relative,
            current_state=current_state,
        )
        joint_state_age = self._joint_state_age_sec()
        self.get_logger().info(
            "HEAD_MOTION state before command | current_yaw=%.3f current_pitch=%.3f joint_state_age=%.3fs"
            % (
                current_state.get("HeadYaw", float("nan")),
                current_state.get("HeadPitch", float("nan")),
                joint_state_age if joint_state_age is not None else -1.0,
            )
        )
        target_validation_error = self._validate_angles(
            target_yaw,
            target_pitch,
            relative=False,
        )
        if target_validation_error:
            goal_handle.abort()
            self.get_logger().warn("HEAD_MOTION failed | %s" % target_validation_error)
            return self._result(
                False,
                target_validation_error,
                time.monotonic() - start_time,
            )

        self._publish_feedback(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "Cancelled before execution", 0.0)

        self._publish_feedback(goal_handle, "executing", 0.4)
        self._publish_joint_angles(yaw=yaw, pitch=pitch, speed=speed, relative=relative)
        self.get_logger().info(
            "HEAD_MOTION command published | target_yaw=%.3f target_pitch=%.3f speed=%.3f relative=%s"
            % (target_yaw, target_pitch, speed, relative)
        )

        if goal_handle.is_cancel_requested:
            duration = time.monotonic() - start_time
            goal_handle.canceled()
            return self._result(False, "Cancelled after command dispatch", duration)

        if not self._wait_for_convergence(
            goal_handle,
            target_yaw=target_yaw,
            target_pitch=target_pitch,
            timeout_sec=self.convergence_timeout_sec,
        ):
            initial_reason = self._convergence_timeout_reason(
                target_yaw=target_yaw,
                target_pitch=target_pitch,
                initial_state=current_state,
            )
            if self.retry_on_convergence_timeout and not goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    "HEAD_MOTION retrying after convergence timeout | %s" % initial_reason
                )
                self._publish_feedback(goal_handle, "retrying", 0.7)
                self._publish_joint_angles(
                    yaw=yaw,
                    pitch=pitch,
                    speed=speed,
                    relative=relative,
                )
                if self._wait_for_convergence(
                    goal_handle,
                    target_yaw=target_yaw,
                    target_pitch=target_pitch,
                    timeout_sec=self.retry_convergence_timeout_sec,
                ):
                    self.get_logger().info(
                        "HEAD_MOTION converged after retry | target_yaw=%.3f target_pitch=%.3f"
                        % (target_yaw, target_pitch)
                    )
                else:
                    reason = self._convergence_timeout_reason(
                        target_yaw=target_yaw,
                        target_pitch=target_pitch,
                        initial_state=current_state,
                    )
                    return self._outcome_after_convergence_timeout(
                        goal_handle,
                        start_time=start_time,
                        reason=reason,
                        target_yaw=target_yaw,
                        target_pitch=target_pitch,
                    )
            else:
                return self._outcome_after_convergence_timeout(
                    goal_handle,
                    start_time=start_time,
                    reason=initial_reason,
                    target_yaw=target_yaw,
                    target_pitch=target_pitch,
                )

        return self._finalize_converged_head_motion(
            goal_handle,
            start_time=start_time,
            relative=relative,
            target_yaw=target_yaw,
            target_pitch=target_pitch,
        )

    def _outcome_after_convergence_timeout(
        self,
        goal_handle,
        *,
        start_time: float,
        reason: str,
        target_yaw: float,
        target_pitch: float,
    ):
        duration = time.monotonic() - start_time
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "Cancelled during convergence wait", duration)
        if self.assume_success_on_convergence_timeout:
            return self._complete_open_loop_after_timeout(
                goal_handle,
                start_time=start_time,
                target_yaw=target_yaw,
                target_pitch=target_pitch,
                reason=reason,
            )
        self.get_logger().warn("HEAD_MOTION failed | %s" % reason)
        goal_handle.abort()
        return self._result(False, reason, duration)

    def _finalize_converged_head_motion(
        self,
        goal_handle,
        *,
        start_time: float,
        relative: bool,
        target_yaw: float,
        target_pitch: float,
    ):
        self._publish_feedback(goal_handle, "completing", 1.0)
        duration = time.monotonic() - start_time
        goal_handle.succeed()
        mode = "relative" if relative else "absolute"
        self.get_logger().info(
            "HEAD_MOTION converged | target_yaw=%.3f target_pitch=%.3f duration=%.3fs"
            % (target_yaw, target_pitch, duration)
        )
        return self._result(
            True,
            f"Head motion converged on '{self.joint_angles_topic}' ({mode})",
            duration,
        )

    def _execute_open_loop(
        self,
        goal_handle,
        *,
        start_time: float,
        yaw: float,
        pitch: float,
        speed: float,
        relative: bool,
        target_yaw: float,
        target_pitch: float,
        reason: str,
    ):
        target_validation_error = self._validate_angles(
            target_yaw,
            target_pitch,
            relative=False,
        )
        if target_validation_error:
            goal_handle.abort()
            self.get_logger().warn("HEAD_MOTION failed | %s" % target_validation_error)
            return self._result(
                False,
                target_validation_error,
                time.monotonic() - start_time,
            )
        self.get_logger().warn("HEAD_MOTION using open-loop dispatch | %s" % reason)
        self._publish_feedback(goal_handle, "executing_open_loop", 0.5)
        self._publish_joint_angles(yaw=yaw, pitch=pitch, speed=speed, relative=relative)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(
                False,
                "Cancelled after open-loop dispatch",
                time.monotonic() - start_time,
            )
        self._publish_feedback(goal_handle, "completing", 1.0)
        goal_handle.succeed()
        return self._result(
            True,
            f"Head motion command published open-loop on '{self.joint_angles_topic}' ({reason})",
            time.monotonic() - start_time,
        )

    def _complete_open_loop_after_timeout(
        self,
        goal_handle,
        *,
        start_time: float,
        target_yaw: float,
        target_pitch: float,
        reason: str,
    ):
        self.get_logger().warn(
            "HEAD_MOTION accepting command despite convergence timeout | %s" % reason
        )
        self._publish_feedback(goal_handle, "completed_without_convergence", 1.0)
        goal_handle.succeed()
        return self._result(
            True,
            (
                "Head motion command published but convergence was not observed "
                f"(target_yaw={target_yaw:.3f}, target_pitch={target_pitch:.3f})"
            ),
            time.monotonic() - start_time,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        updated = False
        with self._joint_state_lock:
            for index, joint_name in enumerate(msg.name):
                if joint_name not in self._HEAD_JOINTS or index >= len(msg.position):
                    continue
                self._head_joint_positions[joint_name] = float(msg.position[index])
                self._last_joint_state_monotonic = time.monotonic()
                updated = True
        if updated:
            self._head_joint_state_event.set()

    def _current_head_state(self) -> Optional[dict[str, float]]:
        with self._joint_state_lock:
            if any(joint_name not in self._head_joint_positions for joint_name in self._HEAD_JOINTS):
                return None
            return {
                joint_name: float(self._head_joint_positions[joint_name])
                for joint_name in self._HEAD_JOINTS
            }

    def _wait_for_head_state(self, timeout_sec: float) -> Optional[dict[str, float]]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while True:
            current_state = self._current_head_state()
            if current_state is not None:
                return current_state
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            self._head_joint_state_event.wait(timeout=min(0.1, remaining))
            self._head_joint_state_event.clear()

    def _resolve_target_angles(
        self,
        *,
        yaw: float,
        pitch: float,
        relative: bool,
        current_state: dict[str, float],
    ) -> tuple[float, float]:
        if not relative:
            return float(yaw), float(pitch)
        return (
            float(current_state.get("HeadYaw", 0.0)) + float(yaw),
            float(current_state.get("HeadPitch", 0.0)) + float(pitch),
        )

    def _wait_for_convergence(
        self,
        goal_handle,
        *,
        target_yaw: float,
        target_pitch: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() <= deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._has_reached_target(target_yaw, target_pitch):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._head_joint_state_event.wait(timeout=min(0.1, remaining))
            self._head_joint_state_event.clear()
        return self._has_reached_target(target_yaw, target_pitch)

    def _has_reached_target(self, target_yaw: float, target_pitch: float) -> bool:
        current_state = self._current_head_state()
        if current_state is None:
            return False
        return (
            abs(current_state["HeadYaw"] - float(target_yaw)) <= self.convergence_tolerance_rad
            and abs(current_state["HeadPitch"] - float(target_pitch)) <= self.convergence_tolerance_rad
        )

    def _joint_state_age_sec(self) -> Optional[float]:
        with self._joint_state_lock:
            if self._last_joint_state_monotonic <= 0.0:
                return None
            return max(0.0, time.monotonic() - self._last_joint_state_monotonic)

    def _convergence_timeout_reason(
        self,
        *,
        target_yaw: float,
        target_pitch: float,
        initial_state: Optional[dict[str, float]] = None,
    ) -> str:
        latest_state = self._current_head_state() or {}
        joint_state_age = self._joint_state_age_sec()
        joint_state_age_text = (
            "unknown"
            if joint_state_age is None
            else f"{joint_state_age:.3f}s"
        )
        reason = (
            "Head motion timed out before convergence "
            f"(target_yaw={target_yaw:.3f}, target_pitch={target_pitch:.3f}, "
            f"latest_yaw={latest_state.get('HeadYaw', float('nan')):.3f}, "
            f"latest_pitch={latest_state.get('HeadPitch', float('nan')):.3f}, "
            f"joint_state_age={joint_state_age_text})"
        )
        if self._joint_state_unchanged_since(initial_state, latest_state):
            return (
                reason
                + " The head joint state did not change after publishing to "
                + self.joint_angles_topic
                + "."
            )
        return reason

    def _joint_state_unchanged_since(
        self,
        initial_state: Optional[dict[str, float]],
        latest_state: dict[str, float],
    ) -> bool:
        if not initial_state or not latest_state:
            return False
        tolerance = min(max(self.convergence_tolerance_rad / 4.0, 1e-3), 0.02)
        for joint_name in self._HEAD_JOINTS:
            if joint_name not in initial_state or joint_name not in latest_state:
                return False
            if abs(float(latest_state[joint_name]) - float(initial_state[joint_name])) > tolerance:
                return False
        return True

    def _publish_joint_angles(self, *, yaw: float, pitch: float, speed: float, relative: bool) -> None:
        msg = JointAnglesWithSpeed()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._HEAD_JOINTS)
        msg.joint_angles = [float(yaw), float(pitch)]
        msg.speed = float(speed)
        msg.relative = 1 if relative else 0
        self._joint_angles_publisher.publish(msg)

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = DoHeadMotion.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, message: str, duration: float):
        result = DoHeadMotion.Result()
        result.success = bool(success)
        result.message = message
        result.duration = float(duration)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadMotionSkillServer()
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
