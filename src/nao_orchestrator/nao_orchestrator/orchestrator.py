#!/usr/bin/env python3
"""Lifecycle orchestrator for ROS4HRI intents."""

from __future__ import annotations

import json

from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hri_actions_msgs.msg import Intent
from interaction_skills.action import LookAt
from nao_skills.action import DoHeadMotion, ReplayMotion
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String

from nao_orchestrator.intent_rules import (
    classify_motion_target,
    normalize_legacy_intent,
    parse_intent_data,
)


class NaoOrchestrator(Node):
    """Dispatch ROS4HRI intents to NAO-specific skill endpoints."""

    def __init__(self) -> None:
        super().__init__("nao_orchestrator")

        self.declare_parameter("intent_topic", "/intents")
        self.declare_parameter("enable_legacy_intent_bridge", True)
        self.declare_parameter("legacy_intent_topic", "/chatbot/intent")
        self.declare_parameter("nao_say_action", "/nao/say")
        self.declare_parameter("replay_motion_action", "/skill/replay_motion")
        self.declare_parameter("head_motion_action", "/skill/do_head_motion")
        self.declare_parameter("look_at_action", "/skill/look_at")
        self.declare_parameter("default_greeting", "Hello! Nice to meet you.")

        self.intent_topic = str(self.get_parameter("intent_topic").value)
        self.enable_legacy_intent_bridge = bool(
            self.get_parameter("enable_legacy_intent_bridge").value
        )
        self.legacy_intent_topic = str(self.get_parameter("legacy_intent_topic").value)
        self.nao_say_action = str(self.get_parameter("nao_say_action").value)
        self.replay_motion_action = str(self.get_parameter("replay_motion_action").value)
        self.head_motion_action = str(self.get_parameter("head_motion_action").value)
        self.look_at_action = str(self.get_parameter("look_at_action").value)
        self.default_greeting = str(self.get_parameter("default_greeting").value)

        self._intent_sub = None
        self._legacy_intent_sub = None
        self._diag_pub = None
        self._diag_timer = None
        self._is_active = False
        self._last_route = ""

        self._say_client = None
        self._replay_motion_client = None
        self._head_motion_client = None
        self._look_at_client = None

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._say_client = ActionClient(self, Say, self.nao_say_action)
        self._replay_motion_client = ActionClient(
            self, ReplayMotion, self.replay_motion_action
        )
        self._head_motion_client = ActionClient(
            self, DoHeadMotion, self.head_motion_action
        )
        self._look_at_client = ActionClient(self, LookAt, self.look_at_action)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self._intent_sub = self.create_subscription(
            Intent, self.intent_topic, self._on_intent, 10
        )
        if self.enable_legacy_intent_bridge:
            self._legacy_intent_sub = self.create_subscription(
                String,
                self.legacy_intent_topic,
                self._on_legacy_intent,
                10,
            )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        if self._intent_sub is not None:
            self.destroy_subscription(self._intent_sub)
            self._intent_sub = None
        if self._legacy_intent_sub is not None:
            self.destroy_subscription(self._legacy_intent_sub)
            self._legacy_intent_sub = None
        return super().on_deactivate(state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        for client in (
            self._say_client,
            self._replay_motion_client,
            self._head_motion_client,
            self._look_at_client,
        ):
            if client is not None:
                client.destroy()
        return TransitionCallbackReturn.SUCCESS

    def _on_legacy_intent(self, msg: String) -> None:
        intent_name, data = normalize_legacy_intent(msg.data)
        bridged = Intent()
        bridged.intent = intent_name
        bridged.data = json.dumps(data, separators=(",", ":"))
        bridged.source = Intent.ROBOT_ITSELF
        bridged.modality = Intent.MODALITY_INTERNAL
        bridged.priority = 128
        bridged.confidence = 1.0
        self._on_intent(bridged)

    def _on_intent(self, msg: Intent) -> None:
        if not self._is_active:
            return

        data = parse_intent_data(msg.data)
        if msg.intent == Intent.GREET:
            self._send_say_goal(data.get("object", "") or self.default_greeting)
            self._last_route = "say:greet"
            return

        if msg.intent == Intent.SAY:
            text = str(data.get("object", "")).strip()
            if text:
                self._send_say_goal(text)
                self._last_route = "say:text"
            return

        if msg.intent == Intent.PERFORM_MOTION:
            route, payload = classify_motion_target(msg.intent, data)
            self._last_route = route
            if route == "replay_motion":
                self._send_replay_motion_goal(payload["motion_name"])
            elif route == "head_motion":
                self._send_head_motion_goal(payload)
            elif route == "look_at":
                self._send_look_at_reset()
            else:
                self.get_logger().warn("Unsupported motion payload: %s" % payload)
            return

        self._last_route = "ignored"
        self.get_logger().warn("Unhandled intent: %s" % msg.intent)

    def _send_say_goal(self, text: str) -> None:
        if not text or self._say_client is None:
            return
        if not self._say_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("nao_say action server unavailable")
            return
        goal = Say.Goal()
        goal.input = text
        self._say_client.send_goal_async(goal)

    def _send_replay_motion_goal(self, motion_name: str) -> None:
        if self._replay_motion_client is None:
            return
        if not self._replay_motion_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("replay_motion action server unavailable")
            return
        goal = ReplayMotion.Goal()
        goal.motion_name = motion_name
        goal.speed = 0.8
        self._replay_motion_client.send_goal_async(goal)

    def _send_head_motion_goal(self, payload: dict) -> None:
        if self._head_motion_client is None:
            return
        if not self._head_motion_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("head_motion action server unavailable")
            return
        goal = DoHeadMotion.Goal()
        goal.yaw = float(payload.get("yaw", 0.0))
        goal.pitch = float(payload.get("pitch", 0.0))
        goal.speed = 0.25
        goal.relative = bool(payload.get("relative", False))
        self._head_motion_client.send_goal_async(goal)

    def _send_look_at_reset(self) -> None:
        if self._look_at_client is None:
            return
        if not self._look_at_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("look_at action server unavailable")
            return
        goal = LookAt.Goal()
        goal.policy = LookAt.Goal.RESET
        self._look_at_client.send_goal_async(goal)

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/nao_orchestrator",
            message="nao_orchestrator running",
            values=[
                KeyValue(
                    key="state",
                    value="active" if self._is_active else "inactive",
                ),
                KeyValue(key="intent_topic", value=self.intent_topic),
                KeyValue(
                    key="legacy_intent_bridge",
                    value=str(self.enable_legacy_intent_bridge),
                ),
                KeyValue(key="last_route", value=self._last_route),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)
