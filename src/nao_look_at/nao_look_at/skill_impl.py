#!/usr/bin/env python3
"""NAO-side implementation of the ROS4HRI interaction_skills/look_at action.

The upstream skill definition lives in `interaction_skills`. This node only
implements that contract for NAO by translating accepted gaze requests into
either head-joint commands or TF-based target tracking.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
import random
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from interaction_skills.action import LookAt
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.time import Time
from std_skills.msg import Result as SkillResult

try:
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - runtime dependent
    JointAnglesWithSpeed = None

try:  # pragma: no cover - runtime dependent
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:  # pragma: no cover - runtime dependent
    Buffer = None
    TransformException = Exception
    TransformListener = None


@dataclass(slots=True)
class _LookAtStats:
    goals_started: int = 0
    goals_succeeded: int = 0
    goals_failed: int = 0
    last_policy: str = ""


class NaoLookAtSkill(Node):
    """Implement the upstream `/skill/look_at` contract for NAO."""

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
        self.declare_parameter("social_yaw", 0.0)
        self.declare_parameter("social_pitch", -0.08)
        self.declare_parameter("random_yaw_abs", 0.55)
        self.declare_parameter("random_pitch_min", -0.25)
        self.declare_parameter("random_pitch_max", 0.25)
        self.declare_parameter("look_from_frame", "CameraTop_frame")
        self.declare_parameter("fallback_look_from_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)
        self.declare_parameter("glance_hold_sec", 0.7)
        self.declare_parameter("minimum_target_distance_m", 0.05)
        self.declare_parameter("max_yaw_abs", 1.5)
        self.declare_parameter("min_pitch", -0.67)
        self.declare_parameter("max_pitch", 0.51)

        self.action_name = str(self.get_parameter("look_at_action_name").value)
        self.joint_angles_topic = str(self.get_parameter("joint_angles_topic").value)
        self.require_joint_angles_subscribers = bool(
            self.get_parameter("require_joint_angles_subscribers").value
        )
        self.reset_yaw = float(self.get_parameter("reset_yaw").value)
        self.reset_pitch = float(self.get_parameter("reset_pitch").value)
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.social_yaw = float(self.get_parameter("social_yaw").value)
        self.social_pitch = float(self.get_parameter("social_pitch").value)
        self.random_yaw_abs = max(
            0.0,
            float(self.get_parameter("random_yaw_abs").value),
        )
        self.random_pitch_min = float(self.get_parameter("random_pitch_min").value)
        self.random_pitch_max = float(self.get_parameter("random_pitch_max").value)
        self.look_from_frame = (
            str(self.get_parameter("look_from_frame").value).strip()
            or "CameraTop_frame"
        )
        self.fallback_look_from_frame = (
            str(self.get_parameter("fallback_look_from_frame").value).strip()
            or "base_link"
        )
        self.tf_lookup_timeout_sec = max(
            0.0,
            float(self.get_parameter("tf_lookup_timeout_sec").value),
        )
        self.glance_hold_sec = max(
            0.0,
            float(self.get_parameter("glance_hold_sec").value),
        )
        self.minimum_target_distance_m = max(
            0.001,
            float(self.get_parameter("minimum_target_distance_m").value),
        )
        self.max_yaw_abs = max(0.0, float(self.get_parameter("max_yaw_abs").value))
        self.min_pitch = float(self.get_parameter("min_pitch").value)
        self.max_pitch = float(self.get_parameter("max_pitch").value)

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._is_active = False
        self._stats = _LookAtStats()
        self._action_server = None
        self._diag_pub = None
        self._diag_timer = None
        self._joint_angles_pub = None
        self._joint_angles_available = JointAnglesWithSpeed is not None
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_available = Buffer is not None and TransformListener is not None

        self.get_logger().info("nao_look_at created; waiting for lifecycle configure")

    # -------------------------------------------------------------------------
    # Lifecycle configuration
    # -------------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Create the action server, diagnostics, and optional TF/joint wiring."""
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
                "JointAnglesWithSpeed is unavailable; nao_look_at will stay in reset-disabled mode"
            )
        if self._tf_available:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        else:
            self._tf_buffer = None
            self._tf_listener = None
            self.get_logger().warn(
                "tf2_ros is unavailable; nao_look_at target tracking will stay disabled"
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
            "nao_look_at configured | action:%s joint_topic:%s look_from:%s"
            % (self.action_name, self.joint_angles_topic, self.look_from_frame)
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Accept action goals once the lifecycle node becomes active."""
        self._is_active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Reject new work while keeping the configured runtime state intact."""
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
        self._tf_listener = None
        self._tf_buffer = None
        return TransitionCallbackReturn.SUCCESS

    # -------------------------------------------------------------------------
    # Action server callbacks
    # -------------------------------------------------------------------------

    def goal_callback(self, goal_request: LookAt.Goal) -> GoalResponse:
        """Validate supported policies and basic runtime preconditions."""
        if self._goal_rejection_reason(goal_request) is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute reset or target-tracking behavior for one look-at goal."""
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
                    return self._abort_goal(
                        goal_handle,
                        "JointAnglesWithSpeed is unavailable; reset policy cannot publish",
                        SkillResult.ROS_ENOTSUP,
                    )
                self._publish_feedback(goal_handle, "completing", 1.0)
                return self._succeed_goal(goal_handle)

            if not self._has_target(request):
                policy_pose = self._resolve_policy_pose(policy)
                if policy_pose is None:
                    return self._abort_goal(
                        goal_handle,
                        f"Policy '{policy}' requires a target frame",
                        SkillResult.ROS_ENOTSUP,
                    )
                yaw, pitch, policy_status = policy_pose
                if not self._publish_joint_pose(yaw, pitch):
                    return self._abort_goal(
                        goal_handle,
                        "JointAnglesWithSpeed is unavailable; policy dispatch cannot publish",
                        SkillResult.ROS_ENOTSUP,
                    )
                self._publish_feedback(goal_handle, policy_status, 0.75)
                if policy == LookAt.Goal.GLANCE and self.glance_hold_sec > 0.0:
                    await asyncio.sleep(self.glance_hold_sec)
                    self._publish_reset_pose()
                self._publish_feedback(goal_handle, "completing", 1.0)
                return self._succeed_goal(goal_handle)

            if self._has_target(request):
                resolved = self._resolve_target_angles(request)
                if isinstance(resolved, str):
                    return self._abort_goal(goal_handle, resolved, SkillResult.ROS_ENOTSUP)

                yaw, pitch, resolved_frame = resolved
                if not self._publish_joint_pose(yaw, pitch):
                    return self._abort_goal(
                        goal_handle,
                        "JointAnglesWithSpeed is unavailable; target tracking cannot publish",
                        SkillResult.ROS_ENOTSUP,
                    )

                self._publish_feedback(goal_handle, "tracking_target", 0.75)
                if policy == LookAt.Goal.GLANCE and self.glance_hold_sec > 0.0:
                    await asyncio.sleep(self.glance_hold_sec)
                    self._publish_reset_pose()
                self._publish_feedback(goal_handle, "completing", 1.0)
                self._stats.last_policy = (
                    f"{policy or 'track'}:{resolved_frame}"
                )
                return self._succeed_goal(goal_handle)

        finally:
            self._execution_lock.release()

    # -------------------------------------------------------------------------
    # Joint-command publishing
    # -------------------------------------------------------------------------

    def _publish_reset_pose(self) -> bool:
        return self._publish_joint_pose(self.reset_yaw, self.reset_pitch)

    def _publish_joint_pose(self, yaw: float, pitch: float) -> bool:
        """Publish the NAO head command used by reset and target tracking."""
        if self._joint_angles_pub is None or JointAnglesWithSpeed is None:
            return False
        msg = JointAnglesWithSpeed()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["HeadYaw", "HeadPitch"]
        msg.joint_angles = [float(yaw), float(pitch)]
        msg.speed = float(self.default_speed)
        msg.relative = 0
        self._joint_angles_pub.publish(msg)
        return True

    # -------------------------------------------------------------------------
    # TF and geometry helpers
    # -------------------------------------------------------------------------

    def _resolve_target_angles(self, goal_request: LookAt.Goal):
        """Resolve a target frame into yaw/pitch angles for NAO head joints."""
        resolved_vector = self._resolve_target_vector(goal_request)
        if isinstance(resolved_vector, str):
            return resolved_vector

        x_value, y_value, z_value, resolved_frame = resolved_vector
        try:
            yaw, pitch = self._vector_to_angles(x_value, y_value, z_value)
        except ValueError as exc:
            return str(exc)
        return (
            self._clamp(yaw, -self.max_yaw_abs, self.max_yaw_abs),
            self._clamp(pitch, self.min_pitch, self.max_pitch),
            resolved_frame,
        )

    def _resolve_target_vector(self, goal_request: LookAt.Goal):
        """Resolve the incoming target into the preferred local reference frame."""
        if not self._has_target(goal_request):
            return "No target frame was provided for look_at"

        target = goal_request.target
        source_frame = str(target.header.frame_id).strip()
        point = (
            float(target.point.x),
            float(target.point.y),
            float(target.point.z),
        )

        for reference_frame in self._reference_frames():
            if source_frame == reference_frame:
                return (*point, reference_frame)

            if self._tf_buffer is None:
                continue

            try:
                transform = self._tf_buffer.lookup_transform(
                    reference_frame,
                    source_frame,
                    self._target_time(target),
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
            except TransformException:
                continue

            transformed = self._apply_transform(point, transform)
            distance = math.sqrt(
                transformed[0] ** 2 + transformed[1] ** 2 + transformed[2] ** 2
            )
            if distance < self.minimum_target_distance_m:
                return (
                    "Target is too close to the look reference frame to compute "
                    "a stable gaze command"
                )
            return (*transformed, reference_frame)

        return (
            "Could not resolve target frame '%s' into %s or %s"
            % (
                source_frame,
                self.look_from_frame,
                self.fallback_look_from_frame,
            )
        )

    def _reference_frames(self) -> tuple[str, ...]:
        ordered = []
        for frame_id in (self.look_from_frame, self.fallback_look_from_frame):
            clean = str(frame_id).strip()
            if clean and clean not in ordered:
                ordered.append(clean)
        return tuple(ordered)

    @staticmethod
    def _target_time(target) -> Time:
        stamp = getattr(getattr(target, "header", None), "stamp", None)
        if stamp is None:
            return Time()
        if int(getattr(stamp, "sec", 0)) == 0 and int(getattr(stamp, "nanosec", 0)) == 0:
            return Time()
        return Time.from_msg(stamp)

    @staticmethod
    def _apply_transform(point, transform) -> tuple[float, float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotated_x, rotated_y, rotated_z = NaoLookAtSkill._rotate_vector(
            (float(point[0]), float(point[1]), float(point[2])),
            (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        return (
            rotated_x + float(translation.x),
            rotated_y + float(translation.y),
            rotated_z + float(translation.z),
        )

    @staticmethod
    def _rotate_vector(vector, quaternion) -> tuple[float, float, float]:
        x_value, y_value, z_value = vector
        qx, qy, qz, qw = quaternion

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        return (
            r00 * x_value + r01 * y_value + r02 * z_value,
            r10 * x_value + r11 * y_value + r12 * z_value,
            r20 * x_value + r21 * y_value + r22 * z_value,
        )

    @staticmethod
    def _vector_to_angles(x_value: float, y_value: float, z_value: float) -> tuple[float, float]:
        horizontal = math.hypot(x_value, y_value)
        distance = math.sqrt(horizontal * horizontal + z_value * z_value)
        if distance <= 1e-6:
            raise ValueError("Target vector is empty; cannot compute look_at angles")
        yaw = math.atan2(y_value, x_value)
        pitch = -math.atan2(z_value, max(horizontal, 1e-6))
        return yaw, pitch

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

    def _resolve_policy_pose(self, policy: str) -> tuple[float, float, str] | None:
        """Map targetless look_at policies to concrete head-joint commands."""
        clean_policy = self._normalize_policy(policy)
        if clean_policy in ("", LookAt.Goal.AUTO):
            return (
                self._clamp(self.reset_yaw, -self.max_yaw_abs, self.max_yaw_abs),
                self._clamp(self.reset_pitch, self.min_pitch, self.max_pitch),
                "auto_reset",
            )
        if clean_policy == LookAt.Goal.SOCIAL:
            return (
                self._clamp(self.social_yaw, -self.max_yaw_abs, self.max_yaw_abs),
                self._clamp(self.social_pitch, self.min_pitch, self.max_pitch),
                "social_focus",
            )
        if clean_policy == LookAt.Goal.RANDOM:
            random_yaw = random.uniform(-self.random_yaw_abs, self.random_yaw_abs)
            lower_pitch = min(self.random_pitch_min, self.random_pitch_max)
            upper_pitch = max(self.random_pitch_min, self.random_pitch_max)
            random_pitch = random.uniform(lower_pitch, upper_pitch)
            return (
                self._clamp(random_yaw, -self.max_yaw_abs, self.max_yaw_abs),
                self._clamp(random_pitch, self.min_pitch, self.max_pitch),
                "random_scan",
            )
        if clean_policy == LookAt.Goal.RESET:
            return (
                self._clamp(self.reset_yaw, -self.max_yaw_abs, self.max_yaw_abs),
                self._clamp(self.reset_pitch, self.min_pitch, self.max_pitch),
                "reset",
            )
        return None

    # -------------------------------------------------------------------------
    # Diagnostics and small action helpers
    # -------------------------------------------------------------------------

    def _goal_rejection_reason(self, goal_request: LookAt.Goal) -> str | None:
        if not self._is_active:
            return "node not active"
        if self._execution_lock.locked():
            return "another goal is running"

        policy = self._normalize_policy(goal_request.policy)
        has_target = self._has_target(goal_request)
        if policy not in self._SUPPORTED_POLICIES:
            return f"unsupported policy: {policy}"
        if policy == LookAt.Goal.GLANCE and not has_target:
            return "glance policy requires a target"
        if policy == "" and not has_target:
            return "target tracking requires a target"
        if (
            self.require_joint_angles_subscribers
            and self._joint_angles_pub is not None
            and self._joint_angles_pub.get_subscription_count() <= 0
        ):
            return "joint command topic has no subscribers"
        return None

    def _abort_goal(self, goal_handle, message: str, error_code: int):
        self._stats.goals_failed += 1
        goal_handle.abort()
        return self._result(False, message, error_code)

    def _succeed_goal(self, goal_handle):
        self._stats.goals_succeeded += 1
        goal_handle.succeed()
        return self._result(True, "", SkillResult.ROS_ENOERR)

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        degraded = not self._joint_angles_available or self._tf_available is False
        status = DiagnosticStatus(
            level=DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK,
            name="/nao_look_at",
            message=(
                "nao_look_at running with reset-only fallback"
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
                KeyValue(key="tf_available", value=str(self._tf_available)),
                KeyValue(key="look_from_frame", value=self.look_from_frame),
                KeyValue(
                    key="fallback_look_from_frame",
                    value=self.fallback_look_from_frame,
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
