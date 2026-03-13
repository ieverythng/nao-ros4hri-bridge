#!/usr/bin/env python3
"""Lifecycle scaffold for the ROS4HRI /skill/look_at action."""

from __future__ import annotations

from dataclasses import dataclass
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from interaction_skills.action import LookAt
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_skills.msg import Result as SkillResult

try:
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - runtime dependent
    JointAnglesWithSpeed = None


@dataclass(slots=True)
class _LookAtStats:
    goals_started: int = 0
    goals_succeeded: int = 0
    goals_failed: int = 0
    last_policy: str = ""


class NaoLookAtSkill(Node):
    """Expose `/skill/look_at` while the full gaze pipeline is being built."""

    _SUPPORTED_POLICIES = {
        "",
        LookAt.Goal.RESET,
        LookAt.Goal.GLANCE,
        LookAt.Goal.RANDOM,
        LookAt.Goal.SOCIAL,
        LookAt.Goal.AUTO,
    }

    def __init__(self) -> None:
        super().__init__("nao_look_at")

        self.declare_parameter("look_at_action_name", "/skill/look_at")
        self.declare_parameter("joint_angles_topic", "/joint_angles")
        self.declare_parameter("require_joint_angles_subscribers", False)
        self.declare_parameter("reset_yaw", 0.0)
        self.declare_parameter("reset_pitch", 0.0)
        self.declare_parameter("default_speed", 0.2)

        self.action_name = str(self.get_parameter("look_at_action_name").value)
        self.joint_angles_topic = str(self.get_parameter("joint_angles_topic").value)
        self.require_joint_angles_subscribers = bool(
            self.get_parameter("require_joint_angles_subscribers").value
        )
        self.reset_yaw = float(self.get_parameter("reset_yaw").value)
        self.reset_pitch = float(self.get_parameter("reset_pitch").value)
        self.default_speed = float(self.get_parameter("default_speed").value)

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._is_active = False
        self._stats = _LookAtStats()
        self._action_server = None
        self._diag_pub = None
        self._diag_timer = None
        self._joint_angles_pub = None
        self._joint_angles_available = JointAnglesWithSpeed is not None

        self.get_logger().info("nao_look_at created; waiting for lifecycle configure")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._joint_angles_available = JointAnglesWithSpeed is not None

        if self._joint_angles_available:
            self._joint_angles_pub = self.create_publisher(
                JointAnglesWithSpeed,
                self.joint_angles_topic,
                10,
            )
        else:
            self._joint_angles_pub = None
            self.get_logger().warn(
                "JointAnglesWithSpeed is unavailable; nao_look_at will stay in scaffold mode"
            )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._action_server = ActionServer(
            self,
            LookAt,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "nao_look_at configured | action:%s joint_topic:%s"
            % (self.action_name, self.joint_angles_topic)
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        return super().on_deactivate(state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._joint_angles_pub is not None:
            self.destroy_publisher(self._joint_angles_pub)
            self._joint_angles_pub = None
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self, goal_request: LookAt.Goal) -> GoalResponse:
        if not self._is_active or self._execution_lock.locked():
            return GoalResponse.REJECT

        policy = self._normalize_policy(goal_request.policy)
        if policy not in self._SUPPORTED_POLICIES:
            return GoalResponse.REJECT
        if policy == LookAt.Goal.GLANCE and not self._has_target(goal_request):
            return GoalResponse.REJECT
        if (
            self.require_joint_angles_subscribers
            and self._joint_angles_pub is not None
            and self._joint_angles_pub.get_subscription_count() <= 0
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, "Another look_at goal is already executing", SkillResult.ROS_ECANCELED)

        self._stats.goals_started += 1
        try:
            request = goal_handle.request
            policy = self._normalize_policy(request.policy)
            self._stats.last_policy = policy or "<target>"

            self._publish_feedback(goal_handle, "preparing", 0.0)
            if policy == LookAt.Goal.RESET:
                if not self._publish_reset_pose():
                    self._stats.goals_failed += 1
                    goal_handle.abort()
                    return self._result(
                        False,
                        "JointAnglesWithSpeed is unavailable; reset policy cannot publish",
                        SkillResult.ROS_ENOTSUP,
                    )
                self._publish_feedback(goal_handle, "completing", 1.0)
                self._stats.goals_succeeded += 1
                goal_handle.succeed()
                return self._result(True, "", SkillResult.ROS_ENOERR)

            if policy == "" and self._has_target(request):
                self._stats.goals_failed += 1
                goal_handle.abort()
                return self._result(
                    False,
                    "Target tracking scaffolded but not implemented yet",
                    SkillResult.ROS_ENOTSUP,
                )

            if policy == LookAt.Goal.GLANCE and self._has_target(request):
                self._stats.goals_failed += 1
                goal_handle.abort()
                return self._result(
                    False,
                    "Glance policy scaffolded but not implemented yet",
                    SkillResult.ROS_ENOTSUP,
                )

            self._stats.goals_failed += 1
            goal_handle.abort()
            return self._result(
                False,
                f"Policy '{policy}' is scaffolded but not implemented yet",
                SkillResult.ROS_ENOTSUP,
            )
        finally:
            self._execution_lock.release()

    def _publish_reset_pose(self) -> bool:
        if self._joint_angles_pub is None or JointAnglesWithSpeed is None:
            return False
        msg = JointAnglesWithSpeed()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["HeadYaw", "HeadPitch"]
        msg.joint_angles = [self.reset_yaw, self.reset_pitch]
        msg.speed = float(self.default_speed)
        msg.relative = 0
        self._joint_angles_pub.publish(msg)
        return True

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        degraded = not self._joint_angles_available
        status = DiagnosticStatus(
            level=DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK,
            name="/nao_look_at",
            message=(
                "nao_look_at running (scaffold mode only)"
                if degraded
                else "nao_look_at running"
            ),
            values=[
                KeyValue(key="state", value="active" if self._is_active else "inactive"),
                KeyValue(key="look_at_action_name", value=self.action_name),
                KeyValue(
                    key="joint_angles_available",
                    value=str(self._joint_angles_available),
                ),
                KeyValue(key="goals_started", value=str(self._stats.goals_started)),
                KeyValue(key="goals_succeeded", value=str(self._stats.goals_succeeded)),
                KeyValue(key="goals_failed", value=str(self._stats.goals_failed)),
                KeyValue(key="last_policy", value=self._stats.last_policy),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    @staticmethod
    def _normalize_policy(policy: str) -> str:
        return str(policy).strip().lower()

    @staticmethod
    def _has_target(goal_request: LookAt.Goal) -> bool:
        header = getattr(goal_request.target, "header", None)
        return bool(header and str(header.frame_id).strip())

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = LookAt.Feedback()
        feedback.feedback.data_str = status
        feedback.feedback.data_float = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, message: str, error_code: int):
        result = LookAt.Result()
        result.result.error_code = SkillResult.ROS_ENOERR if success else int(error_code)
        result.result.error_msg = "" if success else message
        return result
