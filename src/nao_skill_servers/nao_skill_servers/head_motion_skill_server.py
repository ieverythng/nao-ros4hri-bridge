#!/usr/bin/env python3
"""Action server wrapper for NAO head motion via naoqi_driver topics."""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nao_skills.action import DoHeadMotion

try:
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - depends on runtime environment
    JointAnglesWithSpeed = None


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
        self.declare_parameter("require_joint_angles_subscribers", False)

        self.action_name = str(self.get_parameter("action_name").value)
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.yaw_min = float(self.get_parameter("yaw_min").value)
        self.yaw_max = float(self.get_parameter("yaw_max").value)
        self.pitch_min = float(self.get_parameter("pitch_min").value)
        self.pitch_max = float(self.get_parameter("pitch_max").value)
        self.joint_angles_topic = str(self.get_parameter("joint_angles_topic").value)
        self.require_joint_angles_subscribers = bool(
            self.get_parameter("require_joint_angles_subscribers").value
        )

        if JointAnglesWithSpeed is None:
            raise RuntimeError(
                "naoqi_bridge_msgs.msg.JointAnglesWithSpeed is unavailable"
            )

        self._execution_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._joint_angles_publisher = self.create_publisher(
            JointAnglesWithSpeed, self.joint_angles_topic, 10
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

        self.get_logger().info(
            f"head_motion_skill_server ready | action:{self.action_name} "
            f"topic:{self.joint_angles_topic} default_speed:{self.default_speed:.2f}"
        )

    def _resolve_speed(self, requested_speed: float) -> Optional[float]:
        speed = float(requested_speed)
        if speed <= 0.0:
            speed = self.default_speed
        if speed <= 0.0 or speed > 1.0 or math.isnan(speed):
            return None
        return speed

    def _validate_angles(
        self,
        yaw: float,
        pitch: float,
        *,
        relative: bool,
    ) -> Optional[str]:
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
            self.get_logger().warn(
                "Rejected head-motion goal because another goal is running"
            )
            return GoalResponse.REJECT

        speed = self._resolve_speed(goal_request.speed)
        if speed is None:
            self.get_logger().warn(
                f"Rejected head-motion goal with invalid speed "
                f"{float(goal_request.speed):.3f} (valid range: 0.0-1.0, "
                f"with 0.0 meaning default)"
            )
            return GoalResponse.REJECT

        validation_error = self._validate_angles(
            float(goal_request.yaw),
            float(goal_request.pitch),
            relative=bool(goal_request.relative),
        )
        if validation_error:
            self.get_logger().warn(f"Rejected head-motion goal: {validation_error}")
            return GoalResponse.REJECT

        if not self._publisher_ready():
            self.get_logger().warn(
                f"Rejected head-motion goal because topic '{self.joint_angles_topic}' "
                "has no subscribers"
            )
            return GoalResponse.REJECT

        mode = "relative" if bool(goal_request.relative) else "absolute"
        self.get_logger().info(
            f"Accepted head-motion goal: yaw={float(goal_request.yaw):.4f} "
            f"pitch={float(goal_request.pitch):.4f} speed={speed:.2f} mode={mode}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for head-motion goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(
                False,
                "Another head-motion goal is already executing",
                0.0,
            )

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

        if speed is None or validation_error:
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

        self._publish_feedback(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "Cancelled before execution", 0.0)

        self._publish_feedback(goal_handle, "executing", 0.4)
        self._publish_joint_angles(
            yaw=yaw,
            pitch=pitch,
            speed=speed,
            relative=relative,
        )

        if goal_handle.is_cancel_requested:
            duration = time.monotonic() - start_time
            goal_handle.canceled()
            return self._result(
                False,
                "Cancelled after command dispatch",
                duration,
            )

        self._publish_feedback(goal_handle, "completing", 1.0)
        duration = time.monotonic() - start_time
        goal_handle.succeed()
        mode = "relative" if relative else "absolute"
        return self._result(
            True,
            f"Published head motion to '{self.joint_angles_topic}' ({mode})",
            duration,
        )

    def _publish_joint_angles(
        self,
        *,
        yaw: float,
        pitch: float,
        speed: float,
        relative: bool,
    ) -> None:
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
