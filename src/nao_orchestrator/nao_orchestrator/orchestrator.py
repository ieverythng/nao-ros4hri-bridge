#!/usr/bin/env python3
"""Lifecycle orchestrator for ROS4HRI intents.

This node is intentionally downstream-only: it receives normalized intents from
the dialogue stack, deduplicates them, and dispatches the corresponding NAO
skill endpoints without taking over prompt or dialogue ownership.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time

from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from hri_actions_msgs.msg import Intent
from interaction_skills.action import LookAt
from kb_skills.intent_labels import KB_QUERY_INTENTS
from nao_skills.action import DoHeadMotion, ReplayMotion
from planner_common import build_execution_feedback_payload
from planner_common import make_plan_id
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String

from nao_orchestrator.intent_rules import (
    classify_motion_target,
    make_intent_signature,
    normalize_incoming_intent,
    normalize_legacy_intent,
    parse_intent_data,
    posture_topic_fallback_for_motion,
    resolve_say_text,
    resolve_scan_result,
    validate_execution_plan,
)
from nao_orchestrator.planner_gate import PlannerGate

try:  # pragma: no cover - runtime dependency
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
except ImportError:  # pragma: no cover - runtime dependency
    JointAnglesWithSpeed = None

# Posture bridge JSON may report `crouch` where orchestrator expects `kneel`.
_POSTURE_BRIDGE_NAME_ALIASES = {'stand': 'stand', 'sit': 'sit', 'kneel': 'crouch'}


def _first_non_empty_text(*values) -> str:
    for value in values:
        text = str(value or '').strip()
        if text:
            return text
    return ''


def _first_non_empty_value(data: dict, *keys: str) -> str:
    if not isinstance(data, dict):
        return ''
    return _first_non_empty_text(*(data.get(key, '') for key in keys))


@dataclass(slots=True)
class _RuntimeStats:
    intents_received: int = 0
    duplicates_ignored: int = 0
    plans_started: int = 0
    plans_succeeded: int = 0
    plans_failed: int = 0
    dispatched_say: int = 0
    dispatched_replay_motion: int = 0
    dispatched_head_motion: int = 0
    dispatched_look_at: int = 0
    dispatch_failures: int = 0
    last_intent: str = ''
    last_route: str = ''
    last_plan_id: str = ''
    last_plan_status: str = ''


@dataclass(slots=True)
class _ActionExecutionResult:
    accepted: bool
    success: bool
    reason: str = ''


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
        self.declare_parameter('nao_say_result_timeout_sec', 8.0)
        self.declare_parameter('replay_motion_action', '/skill/replay_motion')
        self.declare_parameter('replay_motion_speed', 0.8)
        self.declare_parameter('replay_motion_wait_sec', 0.2)
        self.declare_parameter('replay_motion_result_timeout_sec', 20.0)
        self.declare_parameter('head_motion_action', '/skill/do_head_motion')
        self.declare_parameter('head_motion_speed', 0.25)
        self.declare_parameter('head_motion_wait_sec', 0.2)
        self.declare_parameter('head_motion_result_timeout_sec', 6.0)
        self.declare_parameter('look_at_action', '/skill/look_at')
        self.declare_parameter('look_at_wait_sec', 0.2)
        self.declare_parameter('look_at_result_timeout_sec', 8.0)
        self.declare_parameter('posture_command_topic', '/chatbot/posture_command')
        self.declare_parameter(
            'posture_command_result_topic',
            '/chatbot/posture_command_result',
        )
        self.declare_parameter('posture_command_result_timeout_sec', 20.0)
        self.declare_parameter('fallback_to_posture_topic', True)
        self.declare_parameter('head_motion_joint_angles_topic', '/joint_angles')
        self.declare_parameter('fallback_to_joint_angles_topic', True)
        self.declare_parameter('planner_feedback_topic', '/planner/execution_feedback')
        self.declare_parameter('planner_dialogue_act_topic', '/planner/dialogue_act')
        self.declare_parameter('enable_planner_gate', False)
        self.declare_parameter('planner_gate_request_topic', '/nao_orchestrator/planner_request')
        self.declare_parameter('planner_request_topic', '/planner/request')
        self.declare_parameter('dedupe_window_sec', 0.8)
        self.declare_parameter('default_greeting', 'Hello! Nice to meet you.')
        self.declare_parameter('scan_result_mode', 'success')
        self.declare_parameter('scan_summary', '')
        self.declare_parameter('scan_summary_topic', '/scene/summary')
        self.declare_parameter('scan_report_after_success', True)

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
        self.nao_say_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('nao_say_result_timeout_sec').value),
        )
        self.replay_motion_action = str(self.get_parameter('replay_motion_action').value)
        self.replay_motion_speed = float(
            self.get_parameter('replay_motion_speed').value
        )
        self.replay_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('replay_motion_wait_sec').value),
        )
        self.replay_motion_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('replay_motion_result_timeout_sec').value),
        )
        self.head_motion_action = str(self.get_parameter('head_motion_action').value)
        self.head_motion_speed = float(
            self.get_parameter('head_motion_speed').value
        )
        self.head_motion_wait_sec = max(
            0.0,
            float(self.get_parameter('head_motion_wait_sec').value),
        )
        self.head_motion_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('head_motion_result_timeout_sec').value),
        )
        self.look_at_action = str(self.get_parameter('look_at_action').value)
        self.look_at_wait_sec = max(
            0.0,
            float(self.get_parameter('look_at_wait_sec').value),
        )
        self.look_at_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('look_at_result_timeout_sec').value),
        )
        self.posture_command_topic = str(
            self.get_parameter('posture_command_topic').value
        )
        self.posture_command_result_topic = str(
            self.get_parameter('posture_command_result_topic').value
        )
        self.posture_command_result_timeout_sec = max(
            0.1,
            float(self.get_parameter('posture_command_result_timeout_sec').value),
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
        self.planner_feedback_topic = str(
            self.get_parameter('planner_feedback_topic').value
        )
        self.planner_dialogue_act_topic = str(
            self.get_parameter('planner_dialogue_act_topic').value
        )
        self.enable_planner_gate = bool(self.get_parameter('enable_planner_gate').value)
        self.planner_gate_request_topic = str(
            self.get_parameter('planner_gate_request_topic').value
        )
        self.planner_request_topic = str(self.get_parameter('planner_request_topic').value)
        self.dedupe_window_sec = max(
            0.0,
            float(self.get_parameter('dedupe_window_sec').value),
        )
        self.default_greeting = str(self.get_parameter('default_greeting').value)
        self.scan_result_mode = str(
            self.get_parameter('scan_result_mode').value
        ).strip().lower()
        self.scan_summary = str(self.get_parameter('scan_summary').value).strip()
        self.scan_summary_topic = str(self.get_parameter('scan_summary_topic').value).strip()
        self.scan_report_after_success = bool(
            self.get_parameter('scan_report_after_success').value
        )

        self._intent_sub = None
        self._legacy_intent_sub = None
        self._scan_summary_sub = None
        self._posture_result_sub = None
        self._diag_pub = None
        self._diag_timer = None
        self._posture_command_pub = None
        self._joint_angles_pub = None
        self._planner_feedback_pub = None
        self._planner_gate_sub = None
        self._planner_dialogue_act_sub = None
        self._planner_request_pub = None
        self._is_active = False
        self._stats = _RuntimeStats()
        self._last_intent_signature = ''
        self._last_intent_ts = 0.0

        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None
        self._posture_result_lock = threading.Lock()
        self._posture_result_event = threading.Event()
        self._latest_posture_result: dict | None = None
        self._latest_scan_summary = ''
        self._planner_gate = PlannerGate()

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
        self._posture_result_sub = self.create_subscription(
            String,
            self.posture_command_result_topic,
            self._on_posture_command_result,
            10,
        )
        if self.scan_summary_topic:
            self._scan_summary_sub = self.create_subscription(
                String,
                self.scan_summary_topic,
                self._on_scan_summary,
                10,
            )
        self._planner_feedback_pub = self.create_publisher(
            String,
            self.planner_feedback_topic,
            10,
        )
        if self.enable_planner_gate:
            self._planner_request_pub = self.create_publisher(
                Intent,
                self.planner_request_topic,
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
            'nao_orchestrator configured | intents:%s legacy:%s say:%s replay:%s head:%s look:%s planner_gate:%s->%s'
            % (
                self.intent_topic,
                self.legacy_intent_topic,
                self.nao_say_action,
                self.replay_motion_action,
                self.head_motion_action,
                self.look_at_action,
                self.planner_gate_request_topic if self.enable_planner_gate else 'disabled',
                self.planner_request_topic,
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
        if self.enable_planner_gate:
            self._planner_gate_sub = self.create_subscription(
                Intent,
                self.planner_gate_request_topic,
                self._on_planner_gate_request,
                10,
            )
            self._planner_dialogue_act_sub = self.create_subscription(
                String,
                self.planner_dialogue_act_topic,
                self._on_planner_dialogue_act,
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
        if self._scan_summary_sub is not None:
            self.destroy_subscription(self._scan_summary_sub)
            self._scan_summary_sub = None
        if self._planner_gate_sub is not None:
            self.destroy_subscription(self._planner_gate_sub)
            self._planner_gate_sub = None
        if self._planner_dialogue_act_sub is not None:
            self.destroy_subscription(self._planner_dialogue_act_sub)
            self._planner_dialogue_act_sub = None
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
        if self._planner_gate_sub is not None:
            self.destroy_subscription(self._planner_gate_sub)
            self._planner_gate_sub = None
        if self._planner_dialogue_act_sub is not None:
            self.destroy_subscription(self._planner_dialogue_act_sub)
            self._planner_dialogue_act_sub = None
        if self._posture_result_sub is not None:
            self.destroy_subscription(self._posture_result_sub)
            self._posture_result_sub = None
        if self._scan_summary_sub is not None:
            self.destroy_subscription(self._scan_summary_sub)
            self._scan_summary_sub = None
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
        if self._planner_feedback_pub is not None:
            self.destroy_publisher(self._planner_feedback_pub)
            self._planner_feedback_pub = None
        if self._planner_request_pub is not None:
            self.destroy_publisher(self._planner_request_pub)
            self._planner_request_pub = None
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

    def _on_scan_summary(self, msg: String) -> None:
        self._latest_scan_summary = str(msg.data or '').strip()

    def _on_planner_gate_request(self, msg: Intent) -> None:
        """Admit chatbot-originated planner requests before planner_llm sees them."""
        if not self._is_active or self._planner_request_pub is None:
            return

        decision = self._planner_gate.decide(msg.data)
        if not decision.accepted:
            self._stats.last_route = 'planner_gate:rejected'
            self.get_logger().warn(
                'Planner gate rejected request | goal_id=%s kind=%s reason=%s'
                % (
                    decision.request.goal_id,
                    decision.request.request_kind,
                    decision.reason,
                )
            )
            return

        self._planner_request_pub.publish(msg)
        self._stats.last_route = 'planner_gate:forwarded'
        self.get_logger().info(
            'Planner gate forwarded request | goal_id=%s kind=%s active_goal=%s topic=%s'
            % (
                decision.request.goal_id,
                decision.request.request_kind,
                self._planner_gate.active_goal_id or '-',
                self.planner_request_topic,
            )
        )

    def _on_planner_dialogue_act(self, msg: String) -> None:
        """Observe non-speaking planner acts so gate state clears on planner failure."""
        before = self._planner_gate.active_goal_id
        self._planner_gate.observe_dialogue_act(msg.data)
        after = self._planner_gate.active_goal_id
        if before and not after:
            self.get_logger().info(
                'Planner gate cleared by planner dialogue act | goal_id=%s' % before
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

        plan_context = self._validated_plan_context(intent_name, data)
        if plan_context and self._handle_planned_intent(
            intent_name=intent_name,
            data=data,
            plan=plan_context['steps'],
            plan_context=plan_context,
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
            self._start_direct_motion_dispatch(data)
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

    @staticmethod
    def _validated_plan_context(intent_name: str, data: dict) -> dict | None:
        if not isinstance(data, dict) or 'plan' not in data:
            return None
        return validate_execution_plan(intent_name, data)

    def _handle_planned_intent(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
        source: str,
    ) -> bool:
        plan_id = self._resolve_plan_id(plan_context)
        self._stats.plans_started += 1
        self._stats.last_plan_id = plan_id

        if plan_context.get('errors'):
            self._stats.plans_failed += 1
            self._stats.last_plan_status = 'invalid'
            self._stats.last_route = 'planned:invalid'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='invalid',
                event_type='plan_invalid',
                reason='; '.join(plan_context['errors']),
                blocking=True,
                unmet_preconditions=plan_context['errors'],
                needs_user_input=False,
                validation_errors=plan_context['errors'],
            )
            self.get_logger().warn(
                'Planned intent validation failed | intent=%s source=%s plan_id=%s errors=%s'
                % (intent_name, source, plan_id, plan_context['errors'])
            )
            return False

        self._publish_plan_feedback(
            intent_name=intent_name,
            source=source,
            plan_context=plan_context,
            status='accepted',
            event_type='plan_accepted',
        )
        self._maybe_dispatch_acknowledgement(
            intent_name=intent_name,
            data=data,
            plan=plan,
            plan_context=plan_context,
        )
        self._stats.last_route = 'planned:running'
        self._stats.last_plan_status = 'running'
        worker = threading.Thread(
            target=self._execute_planned_intent,
            kwargs={
                'intent_name': intent_name,
                'data': dict(data),
                'plan': list(plan),
                'plan_context': dict(plan_context),
                'source': source,
            },
            daemon=True,
        )
        worker.start()
        return True

    def _maybe_dispatch_acknowledgement(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
    ) -> None:
        _ = (intent_name, data, plan, plan_context)
        # Planner acknowledgement speech is realized through planner dialogue acts so
        # the executor stays focused on deterministic skill dispatch only.
        return

    def _execute_planned_intent(
        self,
        *,
        intent_name: str,
        data: dict,
        plan: list[dict],
        plan_context: dict,
        source: str,
    ) -> None:
        """Execute a validated plan in a background worker so action results can be awaited."""
        plan_id = self._resolve_plan_id(plan_context)
        executed_any = False

        for step in plan:
            step_started = False

            def _mark_step_started() -> None:
                nonlocal step_started
                if step_started:
                    return
                step_started = True
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='running',
                    event_type='step_started',
                    step=step,
                )

            step_ok, reason = self._dispatch_plan_step(
                step,
                fallback_data=data,
                on_started=_mark_step_started,
            )
            if step_ok:
                executed_any = True
                if not step_started:
                    _mark_step_started()
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='succeeded',
                    event_type='step_succeeded',
                    step=step,
                    result_summary=str(reason or '').strip(),
                )
                continue

            failure_policy = str(step.get('on_failure', 'fail')).strip().lower()
            if failure_policy == 'continue':
                self._publish_plan_feedback(
                    intent_name=intent_name,
                    source=source,
                    plan_context=plan_context,
                    status='failed',
                    event_type='step_failed',
                    reason=reason,
                    step=step,
                    blocking=False,
                    unmet_preconditions=list(step.get('requires', [])),
                    needs_user_input=False,
                )
                self.get_logger().warn(
                    'Planned intent step failed; continuing plan | intent=%s source=%s plan_id=%s step=%s reason=%s'
                    % (intent_name, source, plan_id, step, reason)
                )
                continue
            if failure_policy == 'ignore':
                self.get_logger().info(
                    'Planned intent step failed; on_failure=ignore | intent=%s plan_id=%s step=%s reason=%s'
                    % (intent_name, plan_id, step, reason)
                )
                continue

            self._stats.last_route = 'planned:failed'
            self._stats.plans_failed += 1
            self._stats.last_plan_status = 'failed'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='failed',
                event_type='step_failed',
                reason=reason,
                step=step,
                blocking=True,
                unmet_preconditions=list(step.get('requires', [])),
                needs_user_input=str(step.get('on_failure', '')).strip().lower() in (
                    'ask_user',
                    'clarify',
                ),
            )
            self.get_logger().warn(
                'Planned intent step failed | intent=%s source=%s plan_id=%s step=%s reason=%s'
                % (intent_name, source, plan_id, step, reason)
            )
            return

        if executed_any:
            self._stats.last_route = 'planned'
            self._stats.plans_succeeded += 1
            self._stats.last_plan_status = 'completed'
            self._publish_plan_feedback(
                intent_name=intent_name,
                source=source,
                plan_context=plan_context,
                status='completed',
                event_type='plan_completed',
            )
            return

        self._stats.plans_failed += 1
        self._stats.last_plan_status = 'empty'
        self._publish_plan_feedback(
            intent_name=intent_name,
            source=source,
            plan_context=plan_context,
            status='failed',
            event_type='plan_invalid',
            reason='plan contained no executable steps',
            blocking=True,
        )

    def _dispatch_plan_step(
        self,
        step: dict,
        fallback_data: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        """Execute one step from the optional structured `Intent.data.plan`."""
        step_type = str(step.get('type', '')).strip().lower()
        step_name = str(step.get('name', '')).strip().lower()
        step_args = dict(step.get('args', {}))

        if step_type == 'noop':
            if on_started is not None:
                on_started()
            return True, ''

        if step_type == 'say':
            return self._execute_say_plan_step(
                step_args,
                fallback_data,
                on_started=on_started,
            )

        if step_type == 'look_at':
            return self._dispatch_planned_look_at(
                step_name,
                step_args,
                on_started=on_started,
            )

        if step_type == 'skill':
            if step_name in ('perform_motion', 'motion', ''):
                return self._execute_motion_plan_step(
                    step_args,
                    on_started=on_started,
                )
            if step_name == 'look_at':
                return self._dispatch_planned_look_at(
                    step_name,
                    step_args,
                    on_started=on_started,
                )
            if step_name == 'scan':
                return self._execute_scan_step(
                    step_args,
                    on_started=on_started,
                )

        self._stats.dispatch_failures += 1
        self.get_logger().warn('Unsupported planned step: %s' % step)
        return False, 'unsupported planned step'

    def _dispatch_planned_look_at(
        self,
        step_name: str,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        """Map a planned look-at step onto reset or target-frame dispatch."""
        policy = str(
            step_args.get('policy', step_args.get('object', step_name))
        ).strip().lower()
        if policy in ('reset', 'look_at_reset'):
            success, reason = self._execute_look_at_reset_step(on_started=on_started)
            if success:
                self._stats.dispatched_look_at += 1
                return True, ''
            return False, reason or 'look_at reset dispatch failed'
        target_frame = str(
            step_args.get('target_frame', step_args.get('frame_id', ''))
        ).strip()
        if not target_frame:
            self._stats.dispatch_failures += 1
            self.get_logger().warn(
                'Planned look_at step is missing a target frame or reset policy: %s'
                % step_args
            )
            return False, 'look_at step missing target frame or reset policy'

        success, reason = self._execute_look_at_target_step(
            frame_id=target_frame,
            x_value=step_args.get('x', 0.0),
            y_value=step_args.get('y', 0.0),
            z_value=step_args.get('z', 0.0),
            policy=policy,
            on_started=on_started,
        )
        if success:
            self._stats.dispatched_look_at += 1
            return True, ''
        return False, reason or 'look_at target dispatch failed'

    def _execute_say_plan_step(
        self,
        step_args: dict,
        fallback_data: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        step_text = _first_non_empty_value(
            step_args,
            'text',
            'message',
            'utterance',
            'content',
            'suggested_response',
            'text_hint',
            'object',
        )
        text = resolve_say_text(
            intent_name=Intent.SAY,
            data={
                'object': step_text,
                'suggested_response': _first_non_empty_value(
                    step_args,
                    'suggested_response',
                    'text_hint',
                    'text',
                ),
                'recipient': step_args.get(
                    'recipient',
                    fallback_data.get('recipient', ''),
                ),
            },
            default_greeting=self.default_greeting,
        )
        clean_text = _first_non_empty_text(
            text,
            fallback_data.get('ack_text', ''),
            fallback_data.get('suggested_response', ''),
            fallback_data.get('goal_text', ''),
        )
        if not clean_text:
            self._stats.dispatch_failures += 1
            return False, 'say dispatch failed: empty text'

        goal = Say.Goal()
        goal.input = clean_text
        goal.person_id = str(
            step_args.get('recipient', fallback_data.get('recipient', ''))
        ).strip()
        result = self._execute_action_step(
            client=self._say_client,
            goal=goal,
            wait_sec=self.nao_say_wait_sec,
            result_timeout_sec=self.nao_say_result_timeout_sec,
            description='nao_say',
            on_started=on_started,
        )
        if result.success:
            self._stats.dispatched_say += 1
            self.get_logger().info('ORCH SAY_DISPATCH | %s' % clean_text)
            return True, ''
        return False, result.reason or 'say dispatch failed'

    def _execute_motion_plan_step(
        self,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        route, resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, step_args)
        if route == 'replay_motion':
            motion_name = resolved_payload['motion_name']
            success, reason = self._execute_replay_motion_step(
                motion_name,
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_replay_motion += 1
                return True, ''
            return False, reason or 'motion dispatch failed'

        if route == 'head_motion':
            success, reason = self._execute_head_motion_step(
                resolved_payload,
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_head_motion += 1
                return True, ''
            return False, reason or 'motion dispatch failed'

        if route == 'look_at_reset':
            success, reason = self._execute_look_at_reset_step(
                on_started=on_started,
            )
            if success:
                self._stats.dispatched_look_at += 1
                return True, ''
            return False, reason or 'look_at reset dispatch failed'

        self._stats.dispatch_failures += 1
        self.get_logger().warn('Unsupported motion payload: %s' % step_args)
        return False, 'unsupported motion payload'

    def _execute_scan_step(
        self,
        step_args: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        if on_started is not None:
            on_started()

        success, reason, metadata = resolve_scan_result(
            self._scan_args_with_summary(step_args),
            default_result_mode=self.scan_result_mode,
            default_summary=self.scan_summary,
        )

        self.get_logger().info(
            'ORCH SCAN | target=%s target_kind=%s result_mode=%s'
            % (
                metadata['target'] or metadata['target_kind'],
                metadata['target_kind'],
                metadata['result_mode'],
            )
        )
        if not success:
            self._stats.dispatch_failures += 1
            return False, reason

        if self.scan_report_after_success and reason:
            speech_ok, speech_reason = self._execute_say_plan_step(
                {'text': reason},
                {},
            )
            if not speech_ok:
                return False, speech_reason

        return True, reason

    def _scan_args_with_summary(self, step_args: dict) -> dict:
        scan_args = dict(step_args)
        if not str(scan_args.get('summary', '')).strip() and self._latest_scan_summary:
            scan_args['summary'] = self._latest_scan_summary
        return scan_args

    def _execute_replay_motion_step(
        self,
        motion_name: str,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        clean_motion = str(motion_name).strip()
        if not clean_motion:
            self._stats.dispatch_failures += 1
            return False, 'replay motion dispatch failed: empty motion name'
        goal = ReplayMotion.Goal()
        goal.motion_name = clean_motion
        goal.speed = float(self.replay_motion_speed)
        result = self._execute_action_step(
            client=self._replay_motion_client,
            goal=goal,
            wait_sec=self.replay_motion_wait_sec,
            result_timeout_sec=self.replay_motion_result_timeout_sec,
            description='replay_motion',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info('ORCH REPLAY_DISPATCH | %s' % clean_motion)
            return True, ''

        if (
            not result.accepted
            and self.fallback_to_posture_topic
            and self._posture_command_pub is not None
        ):
            fallback_ok, fallback_reason = self._execute_posture_topic_fallback(
                clean_motion,
                on_started=on_started,
            )
            if fallback_ok:
                return True, ''
            return False, fallback_reason or 'posture fallback failed'
        return False, result.reason or 'replay motion dispatch failed'

    def _execute_head_motion_step(
        self,
        payload: dict,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        yaw = float(payload.get('yaw', 0.0))
        pitch = float(payload.get('pitch', 0.0))
        relative = bool(payload.get('relative', False))
        goal = DoHeadMotion.Goal()
        goal.yaw = yaw
        goal.pitch = pitch
        goal.speed = float(self.head_motion_speed)
        goal.relative = relative
        result = self._execute_action_step(
            client=self._head_motion_client,
            goal=goal,
            wait_sec=self.head_motion_wait_sec,
            result_timeout_sec=self.head_motion_result_timeout_sec,
            description='head_motion',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info(
                'ORCH HEAD_DISPATCH | yaw=%.3f pitch=%.3f relative=%s'
                % (yaw, pitch, relative)
            )
            return True, ''

        if (
            not result.accepted
            and self.fallback_to_joint_angles_topic
            and self._joint_angles_pub is not None
        ):
            if on_started is not None:
                on_started()
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
            return True, ''
        return False, result.reason or 'head motion dispatch failed'

    def _execute_look_at_reset_step(self, *, on_started=None) -> tuple[bool, str]:
        goal = LookAt.Goal()
        goal.policy = LookAt.Goal.RESET
        result = self._execute_action_step(
            client=self._look_at_client,
            goal=goal,
            wait_sec=self.look_at_wait_sec,
            result_timeout_sec=self.look_at_result_timeout_sec,
            description='look_at_reset',
            on_started=on_started,
        )
        if result.success:
            self.get_logger().info('ORCH LOOK_AT_DISPATCH | policy=reset')
            return True, ''

        if not result.accepted:
            self.get_logger().warn('look_at action server unavailable; falling back to head reset')
            return self._execute_head_motion_step(
                {
                    'yaw': 0.0,
                    'pitch': 0.0,
                    'relative': False,
                },
                on_started=on_started,
            )
        return False, result.reason or 'look_at reset dispatch failed'

    def _execute_look_at_target_step(
        self,
        *,
        frame_id: str,
        x_value,
        y_value,
        z_value,
        policy: str = '',
        on_started=None,
    ) -> tuple[bool, str]:
        clean_frame = str(frame_id).strip()
        if not clean_frame:
            self._stats.dispatch_failures += 1
            self.get_logger().warn('No target frame resolved for look_at dispatch')
            return False, 'look_at target dispatch failed: missing target frame'

        goal = LookAt.Goal()
        goal.policy = str(policy).strip().lower()
        target = PointStamped()
        target.header.frame_id = clean_frame
        target.point.x = float(x_value)
        target.point.y = float(y_value)
        target.point.z = float(z_value)
        goal.target = target
        result = self._execute_action_step(
            client=self._look_at_client,
            goal=goal,
            wait_sec=self.look_at_wait_sec,
            result_timeout_sec=self.look_at_result_timeout_sec,
            description='look_at_target',
            on_started=on_started,
        )
        if result.success:
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
            return True, ''
        return False, result.reason or 'look_at target dispatch failed'

    def _execute_action_step(
        self,
        *,
        client,
        goal,
        wait_sec: float,
        result_timeout_sec: float,
        description: str,
        on_started=None,
    ) -> _ActionExecutionResult:
        if client is None:
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s action client unavailable' % description,
            )
        if not client.wait_for_server(timeout_sec=wait_sec):
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s action server unavailable' % description,
            )

        acceptance_event = threading.Event()
        result_event = threading.Event()
        active_goal_handle = {'value': None}
        outcome = {
            'accepted': False,
            'success': False,
            'reason': '%s result timed out' % description,
        }

        def _goal_response_callback(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as err:  # pragma: no cover - ROS action transport failure
                outcome['reason'] = '%s goal response failed: %s' % (description, err)
                acceptance_event.set()
                result_event.set()
                return

            if goal_handle is None or not goal_handle.accepted:
                outcome['reason'] = '%s goal rejected' % description
                acceptance_event.set()
                result_event.set()
                return

            active_goal_handle['value'] = goal_handle
            outcome['accepted'] = True
            acceptance_event.set()
            if on_started is not None:
                try:
                    on_started()
                except Exception as err:
                    self.get_logger().debug(
                        '%s start callback failed: %s' % (description, err)
                    )
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future) -> None:
            try:
                wrapped_result = future.result()
                action_result = getattr(wrapped_result, 'result', None)
                success, reason = self._action_result_status(action_result)
                outcome['success'] = success
                outcome['reason'] = reason
            except Exception as err:  # pragma: no cover - ROS action transport failure
                outcome['success'] = False
                outcome['reason'] = '%s result retrieval failed: %s' % (description, err)
            finally:
                result_event.set()

        goal_future = client.send_goal_async(goal)
        goal_future.add_done_callback(_goal_response_callback)

        goal_response_timeout = max(float(wait_sec), 1.0)
        if not acceptance_event.wait(timeout=goal_response_timeout):
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason='%s goal response timed out' % description,
            )
        if not outcome['accepted']:
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=False,
                success=False,
                reason=str(outcome['reason']).strip(),
            )
        if not result_event.wait(timeout=max(float(result_timeout_sec), 0.1)):
            goal_handle = active_goal_handle.get('value')
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as err:
                    self.get_logger().warn(
                        '%s timed out and cancel request failed: %s'
                        % (description, err)
                    )
            self._stats.dispatch_failures += 1
            return _ActionExecutionResult(
                accepted=True,
                success=False,
                reason='%s result timed out' % description,
            )
        if not outcome['success']:
            self._stats.dispatch_failures += 1
        return _ActionExecutionResult(
            accepted=True,
            success=bool(outcome['success']),
            reason=str(outcome['reason']).strip(),
        )

    @staticmethod
    def _action_result_status(action_result) -> tuple[bool, str]:
        if action_result is None:
            return False, 'action returned no result'
        if hasattr(action_result, 'success'):
            success = bool(getattr(action_result, 'success', False))
            message = str(getattr(action_result, 'message', '')).strip()
            if success:
                return True, message
            return False, message or 'action reported failure'
        return True, ''

    @staticmethod
    def _parse_posture_result_message(payload: str) -> dict:
        try:
            parsed = json.loads(str(payload or '').strip())
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}

    @staticmethod
    def _posture_result_matches(payload: dict, fallback_command: str) -> bool:
        if not isinstance(payload, dict):
            return False
        expected = str(fallback_command or '').strip().lower()
        if not expected:
            return False
        candidates = (
            payload.get('normalized_command', ''),
            payload.get('command', ''),
            payload.get('posture_name', ''),
        )
        alias = _POSTURE_BRIDGE_NAME_ALIASES.get(expected, expected)
        return any(
            str(candidate or '').strip().lower() == expected
            or str(candidate or '').strip().lower() == alias
            for candidate in candidates
            if str(candidate or '').strip()
        )

    def _resolve_plan_id(self, plan_context: dict) -> str:
        plan_id = str(plan_context.get('plan_id', '')).strip()
        if plan_id:
            return plan_id
        return make_plan_id()

    def _publish_plan_feedback(
        self,
        *,
        intent_name: str,
        source: str,
        plan_context: dict,
        status: str,
        event_type: str = '',
        reason: str = '',
        step: dict | None = None,
        blocking: bool = False,
        unmet_preconditions: list[str] | None = None,
        needs_user_input: bool = False,
        validation_errors: list[str] | None = None,
        result_summary: str = '',
    ) -> None:
        if self._planner_feedback_pub is None:
            return
        normalized_plan_context = dict(plan_context)
        normalized_plan_context['plan_id'] = self._resolve_plan_id(plan_context)
        payload = build_execution_feedback_payload(
            intent=str(intent_name).strip(),
            source=str(source).strip(),
            plan_context=normalized_plan_context,
            status=str(status).strip().lower(),
            event_type=event_type,
            reason=str(reason).strip(),
            step=step,
            blocking=blocking,
            unmet_preconditions=list(unmet_preconditions or []),
            needs_user_input=needs_user_input,
            validation_errors=list(validation_errors or []),
            timestamp_sec=round(time.time(), 3),
            result_summary=str(result_summary or '').strip(),
        )
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True, separators=(',', ':'))
        self._planner_feedback_pub.publish(msg)
        self._planner_gate.observe_feedback(payload)

    def _on_posture_command_result(self, msg: String) -> None:
        payload = self._parse_posture_result_message(msg.data)
        if not payload:
            return
        with self._posture_result_lock:
            self._latest_posture_result = payload
        self._posture_result_event.set()

    def _execute_posture_topic_fallback(
        self,
        motion_name: str,
        *,
        on_started=None,
    ) -> tuple[bool, str]:
        if self._posture_command_pub is None:
            self._stats.dispatch_failures += 1
            return False, 'posture fallback publisher unavailable'

        fallback_command = posture_topic_fallback_for_motion(motion_name)
        if not fallback_command:
            self._stats.dispatch_failures += 1
            return False, 'no posture fallback is defined for %s' % motion_name

        with self._posture_result_lock:
            self._latest_posture_result = None
        self._posture_result_event.clear()

        if on_started is not None:
            on_started()

        msg = String()
        msg.data = fallback_command
        self._posture_command_pub.publish(msg)
        self.get_logger().warn(
            'ORCH REPLAY_TOPIC_FALLBACK | motion=%s command=%s topic=%s result_topic=%s'
            % (
                motion_name,
                fallback_command,
                self.posture_command_topic,
                self.posture_command_result_topic,
            )
        )
        return self._wait_for_posture_fallback_result(fallback_command)

    def _wait_for_posture_fallback_result(self, fallback_command: str) -> tuple[bool, str]:
        deadline = time.monotonic() + self.posture_command_result_timeout_sec
        while time.monotonic() <= deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._posture_result_event.wait(timeout=min(0.1, remaining))
            self._posture_result_event.clear()
            with self._posture_result_lock:
                payload = dict(self._latest_posture_result or {})
            if not self._posture_result_matches(payload, fallback_command):
                continue
            success = bool(payload.get('success', False))
            reason = str(payload.get('message', '')).strip()
            if success:
                return True, reason or 'posture bridge reported success'
            self._stats.dispatch_failures += 1
            return False, reason or 'posture bridge reported failure'

        self._stats.dispatch_failures += 1
        return (
            False,
            'Timed out waiting for posture fallback result on %s'
            % self.posture_command_result_topic,
        )

    def _dispatch_motion_payload(self, payload: dict) -> tuple[bool, str]:
        route, resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, payload)
        if route == 'replay_motion':
            motion_name = resolved_payload['motion_name']
            success, reason = self._execute_replay_motion_step(motion_name)
            return success, 'replay_motion:%s' % motion_name if success else reason

        if route == 'head_motion':
            success, reason = self._execute_head_motion_step(resolved_payload)
            return success, 'head_motion' if success else reason

        if route == 'look_at_reset':
            success, reason = self._execute_look_at_reset_step()
            return success, 'look_at_reset' if success else reason

        return False, 'unsupported'

    def _start_direct_motion_dispatch(self, payload: dict) -> None:
        route, _resolved_payload = classify_motion_target(Intent.PERFORM_MOTION, payload)
        if route == 'unsupported':
            self._stats.dispatch_failures += 1
            self._stats.last_route = 'ignored:unsupported_motion'
            self.get_logger().warn('Unsupported motion payload: %s' % payload)
            return

        worker = threading.Thread(
            target=self._execute_direct_motion_dispatch,
            kwargs={'payload': dict(payload)},
            daemon=True,
        )
        worker.start()

    def _execute_direct_motion_dispatch(self, *, payload: dict) -> None:
        dispatched, route_name = self._dispatch_motion_payload(payload)
        if dispatched:
            self._stats.last_route = route_name
            return
        self._stats.last_route = 'failed:%s' % (route_name or 'perform_motion')
        self.get_logger().warn(
            'Direct motion dispatch failed | payload=%s reason=%s'
            % (payload, route_name)
        )

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
                KeyValue(key='planner_gate_enabled', value=str(self.enable_planner_gate)),
                KeyValue(key='planner_gate_active_goal', value=self._planner_gate.active_goal_id),
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
                KeyValue(key='plans_started', value=str(self._stats.plans_started)),
                KeyValue(
                    key='plans_succeeded',
                    value=str(self._stats.plans_succeeded),
                ),
                KeyValue(key='plans_failed', value=str(self._stats.plans_failed)),
                KeyValue(key='last_intent', value=self._stats.last_intent),
                KeyValue(key='last_route', value=self._stats.last_route),
                KeyValue(key='last_plan_id', value=self._stats.last_plan_id),
                KeyValue(
                    key='last_plan_status',
                    value=self._stats.last_plan_status,
                ),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)
