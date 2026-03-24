#!/usr/bin/env python3
"""Lifecycle orchestrator for ROS4HRI intents.

This node is intentionally downstream-only: it receives normalized intents from
the dialogue stack, deduplicates them, and dispatches the corresponding NAO
skill endpoints without taking over prompt or dialogue ownership.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from hri_actions_msgs.msg import Intent
from interaction_skills.action import LookAt
from kb_skills.intent_labels import KB_QUERY_INTENTS
from nao_skills.action import DoHeadMotion, ReplayMotion
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String

from nao_orchestrator.intent_rules import (
    classify_motion_target,
    make_intent_signature,
    normalize_incoming_intent,
    normalize_legacy_intent,
    parse_execution_plan,
    parse_intent_data,
    posture_topic_fallback_for_motion,
    resolve_ack_text,
    resolve_say_text,
)

try:  # pragma: no cover - runtime dependency
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - runtime dependency
    JointAnglesWithSpeed = None


@dataclass(slots=True)
class _RuntimeStats:
    intents_received: int = 0
    duplicates_ignored: int = 0
    dispatched_say: int = 0
    dispatched_replay_motion: int = 0
    dispatched_head_motion: int = 0
    dispatched_look_at: int = 0
    dispatch_failures: int = 0
    last_intent: str = ''
    last_route: str = ''


class NaoOrchestrator(Node):
    """Dispatch ROS4HRI intents to NAO-specific skill endpoints."""

    def __init__(self) -> None:
        super().__init__('nao_orchestrator')
        self.get_logger().info('nao_orchestrator created; waiting for lifecycle configure')

        self.declare_parameter('intent_topic', '/intents')
        self.declare_parameter('enable_legacy_intent_bridge', False)
        self.declare_parameter('legacy_intent_topic', '/chatbot/intent')
        self.declare_parameter('nao_say_action', '/nao/say')
        self.declare_parameter('dispatch_speech_intents', False)
        self.declare_parameter('nao_say_wait_sec', 0.2)
        self.declare_parameter('replay_motion_action', '/skill/replay_motion')
        self.declare_parameter('replay_motion_speed', 0.8)
        self.declare_parameter('replay_motion_wait_sec', 0.2)
        self.declare_parameter('head_motion_action', '/skill/do_head_motion')
        self.declare_parameter('head_motion_speed', 0.25)
        self.declare_parameter('head_motion_wait_sec', 0.2)
        self.declare_parameter('look_at_action', '/skill/look_at')
        self.declare_parameter('look_at_wait_sec', 0.2)
        self.declare_parameter('posture_command_topic', '/chatbot/posture_command')
        self.declare_parameter('fallback_to_posture_topic', True)
        self.declare_parameter('head_motion_joint_angles_topic', '/joint_angles')
        self.declare_parameter('fallback_to_joint_angles_topic', True)
        self.declare_parameter('dedupe_window_sec', 0.8)
        self.declare_parameter('default_greeting', 'Hello! Nice to meet you.')

        self.intent_topic = str(self.get_parameter('intent_topic').value)
        self.enable_legacy_intent_bridge = bool(
            self.get_parameter('enable_legacy_intent_bridge').value
        )
        self.legacy_intent_topic = str(self.get_parameter('legacy_intent_topic').value)
        self.nao_say_action = str(self.get_parameter('nao_say_action').value)
        self.dispatch_speech_intents = bool(
            self.get_parameter('dispatch_speech_intents').value
        )
        self.nao_say_wait_sec = max(
            0.0,
            float(self.get_parameter('nao_say_wait_sec').value),
        )
        self.replay_motion_action = str(self.get_parameter('replay_motion_action').value)
        self.replay_motion_speed = float(
            self.get_parameter('replay_motion_speed').value
        )
        self.replay_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('replay_motion_wait_sec').value),
        )
        self.head_motion_action = str(self.get_parameter('head_motion_action').value)
        self.head_motion_speed = float(
            self.get_parameter('head_motion_speed').value
        )
        self.head_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('head_motion_wait_sec').value),
        )
        self.look_at_action = str(self.get_parameter('look_at_action').value)
        self.look_at_wait_sec = max(
            0.0,
            float(self.get_parameter('look_at_wait_sec').value),
        )
        self.posture_command_topic = str(
            self.get_parameter('posture_command_topic').value
        )
        self.fallback_to_posture_topic = bool(
            self.get_parameter('fallback_to_posture_topic').value
        )
        self.head_motion_joint_angles_topic = str(
            self.get_parameter('head_motion_joint_angles_topic').value
        )
        self.fallback_to_joint_angles_topic = bool(
            self.get_parameter('fallback_to_joint_angles_topic').value
        )
        self.dedupe_window_sec = max(
            0.0,
            float(self.get_parameter('dedupe_window_sec').value),
        )
        self.default_greeting = str(self.get_parameter('default_greeting').value)

        self._intent_sub = None
        self._legacy_intent_sub = None
        self._diag_pub = None
        self._diag_timer = None
        self._posture_command_pub = None
        self._joint_angles_pub = None
        self._is_active = False
        self._stats = _RuntimeStats()
        self._last_intent_signature = ''
        self._last_intent_ts = 0.0

        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None

    # -------------------------------------------------------------------------
    # Lifecycle configuration
    # -------------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Create action clients, diagnostics, and topic fallbacks."""
        self._destroy_runtime_interfaces()
        self._say_client = ActionClient(self, Say, self.nao_say_action)
        self._replay_motion_client = ActionClient(
            self,
            ReplayMotion,
            self.replay_motion_action,
        )
        self._head_motion_client = ActionClient(
            self,
            DoHeadMotion,
            self.head_motion_action,
        )
        self._look_at_client = ActionClient(self, LookAt, self.look_at_action)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._posture_command_pub = self.create_publisher(
            String,
            self.posture_command_topic,
            10,
        )
        if JointAnglesWithSpeed is not None:
            self._joint_angles_pub = self.create_publisher(
                JointAnglesWithSpeed,
                self.head_motion_joint_angles_topic,
                10,
            )
        else:
            self.get_logger().warn(
                'JointAnglesWithSpeed unavailable; joint-topic fallback is disabled'
            )
        self.get_logger().info(
            'nao_orchestrator configured | intents:%s legacy:%s say:%s replay:%s head:%s look:%s'
            % (
                self.intent_topic,
                self.legacy_intent_topic,
                self.nao_say_action,
                self.replay_motion_action,
                self.head_motion_action,
                self.look_at_action,
            )
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start intent subscriptions once the lifecycle node becomes active."""
        self._is_active = True
        self._intent_sub = self.create_subscription(
            Intent,
            self.intent_topic,
            self._on_intent,
            10,
        )
        if self.enable_legacy_intent_bridge:
            self._legacy_intent_sub = self.create_subscription(
                String,
                self.legacy_intent_topic,
                self._on_legacy_intent,
                10,
            )
        self.get_logger().info('nao_orchestrator active')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop subscriptions while keeping the configured action clients alive."""
        self._is_active = False
        if self._intent_sub is not None:
            self.destroy_subscription(self._intent_sub)
            self._intent_sub = None
        if self._legacy_intent_sub is not None:
            self.destroy_subscription(self._legacy_intent_sub)
            self._legacy_intent_sub = None
        self.get_logger().info('nao_orchestrator inactive')
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._destroy_runtime_interfaces()
        self.get_logger().info('nao_orchestrator cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self._destroy_runtime_interfaces()
        self.get_logger().info('nao_orchestrator shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def _destroy_runtime_interfaces(self) -> None:
        """Tear down publishers, subscriptions, timers, and action clients."""
        if self._intent_sub is not None:
            self.destroy_subscription(self._intent_sub)
            self._intent_sub = None
        if self._legacy_intent_sub is not None:
            self.destroy_subscription(self._legacy_intent_sub)
            self._legacy_intent_sub = None
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._posture_command_pub is not None:
            self.destroy_publisher(self._posture_command_pub)
            self._posture_command_pub = None
        if self._joint_angles_pub is not None:
            self.destroy_publisher(self._joint_angles_pub)
            self._joint_angles_pub = None
        for client in (
            self._say_client,
            self._replay_motion_client,
            self._head_motion_client,
            self._look_at_client,
        ):
            if client is not None:
                client.destroy()
        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None

    # -------------------------------------------------------------------------
    # Intent ingestion
    # -------------------------------------------------------------------------

    def _on_legacy_intent(self, msg: String) -> None:
        """Normalize the old `/chatbot/intent` bridge onto the new routing path."""
        intent_name, data = normalize_legacy_intent(
            msg.data,
            default_greeting=self.default_greeting,
        )
        self._handle_intent(
            intent_name=intent_name,
            data=data,
            source='legacy_intent_bridge',
        )

    def _on_intent(self, msg: Intent) -> None:
        """Normalize canonical `hri_actions_msgs/Intent` messages for dispatch."""
        data = parse_intent_data(msg.data)
        intent_name, normalized_data = normalize_incoming_intent(
            intent_name=msg.intent,
            data=data,
            default_greeting=self.default_greeting,
        )
        self._handle_intent(
            intent_name=intent_name,
            data=normalized_data,
            source=str(msg.source or msg.modality or Intent.UNKNOWN),
        )

    def _handle_intent(self, intent_name: str, data: dict, source: str) -> None:
        """Route one normalized intent through planned or legacy dispatch paths."""
        if not self._is_active:
            return

        self._stats.intents_received += 1
        self._stats.last_intent = str(intent_name).strip()
        signature = make_intent_signature(intent_name, data)
        if self._is_duplicate(signature):
            self._stats.duplicates_ignored += 1
            self._stats.last_route = 'ignored:duplicate'
            self.get_logger().warn(
                'Ignored duplicate intent within %.2fs | intent=%s source=%s'
                % (
                    self.dedupe_window_sec,
                    intent_name,
                    source,
                )
            )
            return

        plan = parse_execution_plan(data)
        if plan and self._handle_planned_intent(
            intent_name=intent_name,
            data=data,
            plan=plan,
            source=source,
        ):
            return

        if intent_name in (Intent.GREET, Intent.SAY):
            if not self.dispatch_speech_intents:
                self._stats.last_route = 'ignored:speech_owned_by_dialogue_manager'
                self.get_logger().info(
                    'Ignored conversational speech intent: %s source=%s'
                    % (intent_name, source)
                )
                return
            text = resolve_say_text(
                intent_name=intent_name,
                data=data,
                default_greeting=self.default_greeting,
            )
            if self._dispatch_say(text, data):
                self._stats.dispatched_say += 1
                self._stats.last_route = 'say'
            return

        if intent_name == Intent.PERFORM_MOTION:
            route, payload = classify_motion_target(intent_name, data)
            if route == 'replay_motion':
                if self._dispatch_replay_motion(payload['motion_name']):
                    self._stats.dispatched_replay_motion += 1
                    self._stats.last_route = 'replay_motion:%s' % payload['motion_name']
                return
            if route == 'head_motion':
                if self._dispatch_head_motion(payload):
                    self._stats.dispatched_head_motion += 1
                    self._stats.last_route = 'head_motion'
                return
            if route == 'look_at_reset':
                if self._dispatch_look_at_reset():
                    self._stats.dispatched_look_at += 1
                    self._stats.last_route = 'look_at_reset'
                return

            self._stats.dispatch_failures += 1
            self._stats.last_route = 'ignored:unsupported_motion'
            self.get_logger().warn('Unsupported motion payload: %s' % payload)
            return

        if intent_name in KB_QUERY_INTENTS:
            self._stats.last_route = 'ignored:kb_query'
            self.get_logger().info(
                'Observed KB query intent for future routing: %s source=%s data=%s'
                % (intent_name, source, data)
            )
            return

        self._stats.last_route = 'ignored:unhandled'
        self.get_logger().warn(
            'Unhandled intent: %s source=%s data=%s'
            % (intent_name, source, data)
        )

    # -------------------------------------------------------------------------
    # Structured plan execution
    # -------------------------------------------------------------------------

    def _handle_planned_intent(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        source: str,
    ) -> bool:
        self._maybe_dispatch_acknowledgement(
            intent_name=intent_name,
            data=data,
            plan=plan,
        )

        executed_any = False
        for step in plan:
            if self._dispatch_plan_step(step, fallback_data=data):
                executed_any = True
                continue

            self._stats.dispatch_failures += 1
            self._stats.last_route = 'planned:failed'
            self.get_logger().warn(
                'Planned intent step failed | intent=%s source=%s step=%s'
                % (intent_name, source, step)
            )
            return False

        if executed_any:
            self._stats.last_route = 'planned'
            return True
        return False

    def _maybe_dispatch_acknowledgement(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
    ) -> None:
        if not self.dispatch_speech_intents:
            return
        if intent_name in (Intent.GREET, Intent.SAY):
            return
        if any(step.get('type') == 'say' for step in plan):
            return

        ack_text = resolve_ack_text(
            intent_name=intent_name,
            data=data,
            default_greeting=self.default_greeting,
        )
        if not ack_text:
            return
        if self._dispatch_say(ack_text, data):
            self._stats.dispatched_say += 1

    def _dispatch_plan_step(self, step: dict, fallback_data: dict) -> bool:
        """Execute one step from the optional structured `Intent.data.plan`."""
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        step_args = dict(step.get('args', {}))

        if step_type == 'noop':
            return True

        if step_type == 'say':
            text = resolve_say_text(
                intent_name=Intent.SAY,
                data={
                    'object': step_args.get('text', step_args.get('object', '')),
                    'suggested_response': step_args.get('text', ''),
                    'recipient': step_args.get(
                        'recipient',
                        fallback_data.get('recipient', ''),
                    ),
                },
                default_greeting=self.default_greeting,
            )
            if self._dispatch_say(text, {**fallback_data, **step_args}):
                self._stats.dispatched_say += 1
                return True
            return False

        if step_type == 'look_at':
            return self._dispatch_planned_look_at(step_name, step_args)

        if step_type == 'skill':
            if step_name in ('perform_motion', 'motion', ''):
                route, payload = classify_motion_target(Intent.PERFORM_MOTION, step_args)
                if route == 'replay_motion':
                    if self._dispatch_replay_motion(payload['motion_name']):
                        self._stats.dispatched_replay_motion += 1
                        return True
                    return False
                if route == 'head_motion':
                    if self._dispatch_head_motion(payload):
                        self._stats.dispatched_head_motion += 1
                        return True
                    return False
                if route == 'look_at_reset':
                    if self._dispatch_look_at_reset():
                        self._stats.dispatched_look_at += 1
                        return True
                    return False
                return False
            if step_name == 'look_at':
                return self._dispatch_planned_look_at(step_name, step_args)

        return False

    def _dispatch_planned_look_at(self, step_name: str, step_args: dict) -> bool:
        """Map a planned look-at step onto reset or target-frame dispatch."""
        policy = str(
            step_args.get('policy', step_args.get('object', step_name))
        ).strip().lower()
        if policy in ('reset', 'look_at_reset'):
            if self._dispatch_look_at_reset():
                self._stats.dispatched_look_at += 1
                return True
            return False
        target_frame = str(
            step_args.get('target_frame', step_args.get('frame_id', ''))
        ).strip()
        if target_frame and self._dispatch_look_at_target(
            frame_id=target_frame,
            x_value=step_args.get('x', 0.0),
            y_value=step_args.get('y', 0.0),
            z_value=step_args.get('z', 0.0),
            policy=policy,
        ):
            self._stats.dispatched_look_at += 1
            return True
        return False

    # -------------------------------------------------------------------------
    # Skill dispatch helpers
    # -------------------------------------------------------------------------

    def _dispatch_say(self, text: str, data: dict) -> bool:
        """Send one text payload to the canonical `/nao/say` action."""
        clean_text = str(text).strip()
        if not clean_text:
            self._stats.dispatch_failures += 1
            self.get_logger().warn('No text resolved for say/greet dispatch')
            return False
        if self._say_client is None:
            self._stats.dispatch_failures += 1
            return False
        if not self._say_client.wait_for_server(timeout_sec=self.nao_say_wait_sec):
            self._stats.dispatch_failures += 1
            self.get_logger().warn('nao_say action server unavailable')
            return False

        goal = Say.Goal()
        goal.input = clean_text
        goal.person_id = str(data.get('recipient', '')).strip()
        self._say_client.send_goal_async(goal)
        self.get_logger().info('ORCH SAY_DISPATCH | %s' % clean_text)
        return True

    def _dispatch_replay_motion(self, motion_name: str) -> bool:
        """Dispatch replay-motion or fall back to the legacy posture topic."""
        clean_motion = str(motion_name).strip()
        if not clean_motion:
            self._stats.dispatch_failures += 1
            return False
        if (
            self._replay_motion_client is not None
            and self._replay_motion_client.wait_for_server(
                timeout_sec=self.replay_motion_wait_sec
            )
        ):
            goal = ReplayMotion.Goal()
            goal.motion_name = clean_motion
            goal.speed = float(self.replay_motion_speed)
            self._replay_motion_client.send_goal_async(goal)
            self.get_logger().info('ORCH REPLAY_DISPATCH | %s' % clean_motion)
            return True

        if self.fallback_to_posture_topic and self._posture_command_pub is not None:
            fallback_command = posture_topic_fallback_for_motion(clean_motion)
            if fallback_command:
                msg = String()
                msg.data = fallback_command
                self._posture_command_pub.publish(msg)
                self.get_logger().warn(
                    'ORCH REPLAY_TOPIC_FALLBACK | motion=%s command=%s topic=%s'
                    % (
                        clean_motion,
                        fallback_command,
                        self.posture_command_topic,
                    )
                )
                return True

        self._stats.dispatch_failures += 1
        self.get_logger().warn(
            'Replay motion action server unavailable and no topic fallback for %s'
            % clean_motion
        )
        return False

    def _dispatch_head_motion(self, payload: dict) -> bool:
        """Dispatch head motion or fall back to direct joint-angle publishing."""
        yaw = float(payload.get('yaw', 0.0))
        pitch = float(payload.get('pitch', 0.0))
        relative = bool(payload.get('relative', False))

        if (
            self._head_motion_client is not None
            and self._head_motion_client.wait_for_server(
                timeout_sec=self.head_motion_wait_sec
            )
        ):
            goal = DoHeadMotion.Goal()
            goal.yaw = yaw
            goal.pitch = pitch
            goal.speed = float(self.head_motion_speed)
            goal.relative = relative
            self._head_motion_client.send_goal_async(goal)
            self.get_logger().info(
                'ORCH HEAD_DISPATCH | yaw=%.3f pitch=%.3f relative=%s'
                % (yaw, pitch, relative)
            )
            return True

        if self.fallback_to_joint_angles_topic and self._joint_angles_pub is not None:
            msg = JointAnglesWithSpeed()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['HeadYaw', 'HeadPitch']
            msg.joint_angles = [yaw, pitch]
            msg.speed = float(self.head_motion_speed)
            msg.relative = 1 if relative else 0
            self._joint_angles_pub.publish(msg)
            self.get_logger().warn(
                'ORCH HEAD_TOPIC_FALLBACK | yaw=%.3f pitch=%.3f topic=%s'
                % (yaw, pitch, self.head_motion_joint_angles_topic)
            )
            return True

        self._stats.dispatch_failures += 1
        self.get_logger().warn(
            'Head motion action server unavailable and joint-topic fallback is disabled'
        )
        return False

    def _dispatch_look_at_reset(self) -> bool:
        """Prefer the upstream look-at action and fall back to a head reset."""
        if (
            self._look_at_client is not None
            and self._look_at_client.wait_for_server(timeout_sec=self.look_at_wait_sec)
        ):
            goal = LookAt.Goal()
            goal.policy = LookAt.Goal.RESET
            self._look_at_client.send_goal_async(goal)
            self.get_logger().info('ORCH LOOK_AT_DISPATCH | policy=reset')
            return True

        self.get_logger().warn('look_at action server unavailable; falling back to head reset')
        return self._dispatch_head_motion(
            {
                'yaw': 0.0,
                'pitch': 0.0,
                'relative': False,
            }
        )

    def _dispatch_look_at_target(
        self,
        *,
        frame_id: str,
        x_value,
        y_value,
        z_value,
        policy: str = '',
    ) -> bool:
        """Dispatch a target-frame gaze request through `/skill/look_at`."""
        clean_frame = str(frame_id).strip()
        if not clean_frame:
            self._stats.dispatch_failures += 1
            self.get_logger().warn('No target frame resolved for look_at dispatch')
            return False
        if (
            self._look_at_client is None
            or not self._look_at_client.wait_for_server(timeout_sec=self.look_at_wait_sec)
        ):
            self._stats.dispatch_failures += 1
            self.get_logger().warn('look_at action server unavailable for target dispatch')
            return False

        goal = LookAt.Goal()
        goal.policy = str(policy).strip().lower()
        target = PointStamped()
        target.header.frame_id = clean_frame
        target.point.x = float(x_value)
        target.point.y = float(y_value)
        target.point.z = float(z_value)
        goal.target = target
        self._look_at_client.send_goal_async(goal)
        self.get_logger().info(
            'ORCH LOOK_AT_DISPATCH | policy=%s frame=%s x=%.3f y=%.3f z=%.3f'
            % (
                goal.policy or 'track',
                clean_frame,
                float(target.point.x),
                float(target.point.y),
                float(target.point.z),
            )
        )
        return True

    def _is_duplicate(self, signature: str) -> bool:
        """Drop repeated intents that arrive inside the configured dedupe window."""
        now = time.monotonic()
        if (
            signature
            and signature == self._last_intent_signature
            and now - self._last_intent_ts <= self.dedupe_window_sec
        ):
            return True

        self._last_intent_signature = signature
        self._last_intent_ts = now
        return False

    # -------------------------------------------------------------------------
    # Diagnostics
    # -------------------------------------------------------------------------

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='/nao_orchestrator',
            message='nao_orchestrator running',
            values=[
                KeyValue(
                    key='state',
                    value='active' if self._is_active else 'inactive',
                ),
                KeyValue(key='intent_topic', value=self.intent_topic),
                KeyValue(
                    key='legacy_intent_bridge',
                    value=str(self.enable_legacy_intent_bridge),
                ),
                KeyValue(
                    key='dispatch_speech_intents',
                    value=str(self.dispatch_speech_intents),
                ),
                KeyValue(
                    key='intents_received',
                    value=str(self._stats.intents_received),
                ),
                KeyValue(
                    key='duplicates_ignored',
                    value=str(self._stats.duplicates_ignored),
                ),
                KeyValue(
                    key='dispatch_failures',
                    value=str(self._stats.dispatch_failures),
                ),
                KeyValue(key='last_intent', value=self._stats.last_intent),
                KeyValue(key='last_route', value=self._stats.last_route),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)
