#!/usr/bin/env python3
"""Print a filtered demo view of ROS logs from /rosout."""

from __future__ import annotations

import argparse
from datetime import datetime
import json

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

try:  # pragma: no cover - import-light unit tests outside ROS workspaces
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover
    Intent = None


DEFAULT_NODE_ALLOWLIST = (
    "chatbot_llm",
    "planner_llm",
    "nao_orchestrator",
    "dialogue_manager",
    "nao_say_skill",
    "head_motion_skill_server",
    "replay_motion_skill_server",
    "nao_look_at",
    "robot_speech_debug",
)

_LEVELS = {
    "debug": Log.DEBUG,
    "info": Log.INFO,
    "warn": Log.WARN,
    "warning": Log.WARN,
    "error": Log.ERROR,
    "fatal": Log.FATAL,
}

_LEVEL_NAMES = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


class DemoRosoutFilter(Node):
    """Print a compact operator trace for demo-relevant runtime events."""

    def __init__(
        self,
        *,
        nodes: tuple[str, ...],
        min_level: int,
        include_topics: bool,
    ) -> None:
        super().__init__("demo_rosout_filter")
        self._allowed_nodes = {_normalize_node_name(node) for node in nodes}
        self._min_level = int(min_level)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        if include_topics:
            self._create_topic_trace_subscriptions()
        print(
            "Demo log filter active | nodes=%s min_level=%s topics=%s"
            % (
                ",".join(sorted(self._allowed_nodes)),
                _LEVEL_NAMES.get(self._min_level, self._min_level),
                include_topics,
            ),
            flush=True,
        )

    def _on_log(self, msg: Log) -> None:
        node_name = _normalize_node_name(msg.name)
        if node_name not in self._allowed_nodes or int(msg.level) < self._min_level:
            return
        stamp = _format_stamp(msg)
        level = _LEVEL_NAMES.get(int(msg.level), str(int(msg.level)))
        print("%s %-5s %-28s %s" % (stamp, level, node_name, msg.msg), flush=True)

    def _create_topic_trace_subscriptions(self) -> None:
        if Intent is not None:
            self.create_subscription(
                Intent,
                "/intents",
                lambda msg: self._print_intent("INTENT", msg),
                10,
            )
            self.create_subscription(
                Intent,
                "/planner/request",
                lambda msg: self._print_intent("PLANNER_REQUEST", msg),
                10,
            )
        self.create_subscription(
            String,
            "/planner/execution_feedback",
            lambda msg: self._print_json_string("PLANNER_FEEDBACK", msg.data),
            10,
        )
        self.create_subscription(
            String,
            "/planner/dialogue_act",
            lambda msg: self._print_json_string("PLANNER_ACT", msg.data),
            10,
        )
        self.create_subscription(
            String,
            "/dialogue_manager/closed_captions",
            lambda msg: self._print_plain("CAPTION", msg.data),
            10,
        )

    def _print_intent(self, label: str, msg) -> None:
        print(_format_intent_event(label, msg), flush=True)

    def _print_json_string(self, label: str, payload: str) -> None:
        print(_format_json_event(label, payload), flush=True)

    def _print_plain(self, label: str, text: str) -> None:
        clean_text = _compact_text(text, 220)
        if clean_text:
            print("%-16s %s" % (label, clean_text), flush=True)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Filtered /rosout demo log window")
    parser.add_argument(
        "--nodes",
        default=",".join(DEFAULT_NODE_ALLOWLIST),
        help="Comma-separated node allowlist. Leading slashes are ignored.",
    )
    parser.add_argument(
        "--min-level",
        default="info",
        choices=sorted(_LEVELS),
        help="Minimum log severity to print.",
    )
    parser.add_argument(
        "--no-topics",
        action="store_true",
        help="Only print filtered /rosout logs; skip structured topic summaries.",
    )
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = DemoRosoutFilter(
        nodes=_parse_nodes(parsed.nodes),
        min_level=_LEVELS[str(parsed.min_level).lower()],
        include_topics=not parsed.no_topics,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _parse_nodes(value: str) -> tuple[str, ...]:
    nodes = tuple(
        node
        for node in (_normalize_node_name(item) for item in str(value or "").split(","))
        if node
    )
    return nodes or DEFAULT_NODE_ALLOWLIST


def _normalize_node_name(value: str) -> str:
    return str(value or "").strip().strip("/")


def _format_stamp(msg: Log) -> str:
    seconds = int(msg.stamp.sec)
    nanoseconds = int(msg.stamp.nanosec)
    if seconds <= 0:
        return "--:--:--.---"
    timestamp = datetime.fromtimestamp(seconds).strftime("%H:%M:%S")
    return "%s.%03d" % (timestamp, nanoseconds // 1_000_000)


def _format_intent_event(label: str, msg) -> str:
    data = _parse_json(str(getattr(msg, "data", "") or ""))
    plan = data.get("plan") if isinstance(data.get("plan"), dict) else {}
    steps = plan.get("steps") if isinstance(plan.get("steps"), list) else []
    detail_parts = [
        "intent=%s" % _compact_text(getattr(msg, "intent", ""), 80),
        "source=%s" % _compact_text(getattr(msg, "source", ""), 60),
        "modality=%s" % _compact_text(getattr(msg, "modality", ""), 40),
    ]
    if plan:
        step_names = _format_step_names(steps)
        detail_parts.extend(
            [
                "goal_id=%s" % _compact_text(plan.get("goal_id", ""), 60),
                "plan_id=%s" % _compact_text(plan.get("plan_id", ""), 60),
                "steps=%d" % len(steps),
                "step_names=%s" % _compact_text(step_names, 120),
                "validation=%s" % _compact_text(plan.get("validation_status", ""), 40),
            ]
        )
    else:
        request_id = data.get("request_id") or data.get("goal_id")
        if request_id:
            detail_parts.append("request_id=%s" % _compact_text(request_id, 60))
        goal_text = data.get("goal_text", "")
        if goal_text:
            detail_parts.append("goal=%s" % _compact_text(goal_text, 120))
        intents = data.get("normalized_intents", data.get("intents"))
        if isinstance(intents, list):
            detail_parts.append("intents=%s" % _compact_text(",".join(map(str, intents)), 100))
        requested_plan = data.get("requested_plan")
        if isinstance(requested_plan, list):
            detail_parts.append("requested_steps=%d" % len(requested_plan))
    return "%-16s %s" % (label, " ".join(part for part in detail_parts if part))


def _format_json_event(label: str, payload: str) -> str:
    data = _parse_json(payload)
    if not data:
        return "%-16s %s" % (label, _compact_text(payload, 220))

    if label == "PLANNER_FEEDBACK":
        step = data.get("step")
        step_name = ""
        if isinstance(step, dict):
            step_name = "/".join(
                part for part in (str(step.get("type", "")), str(step.get("name", ""))) if part
            )
        parts = [
            "event=%s" % _compact_text(data.get("event") or data.get("event_type", ""), 50),
            "status=%s" % _compact_text(data.get("status", ""), 40),
            "goal_id=%s" % _compact_text(data.get("goal_id", ""), 60),
            "plan_id=%s" % _compact_text(data.get("plan_id", ""), 60),
            "step=%s" % _compact_text(step_name, 80),
            "reason=%s" % _compact_text(data.get("reason", ""), 120),
        ]
        return "%-16s %s" % (label, " ".join(part for part in parts if not part.endswith("=")))

    if label == "PLANNER_ACT":
        parts = [
            "act=%s" % _compact_text(data.get("act", ""), 50),
            "goal_id=%s" % _compact_text(data.get("goal_id", ""), 60),
            "plan_id=%s" % _compact_text(data.get("plan_id", ""), 60),
            "await_user=%s" % _compact_text(data.get("await_user_response", ""), 20),
            "hint=%s" % _compact_text(data.get("text_hint", ""), 120),
            "reason=%s" % _compact_text(data.get("reason", ""), 120),
        ]
        return "%-16s %s" % (label, " ".join(part for part in parts if not part.endswith("=")))

    return "%-16s %s" % (label, _compact_text(json.dumps(data, sort_keys=True), 220))


def _parse_json(payload: str) -> dict:
    try:
        parsed = json.loads(str(payload or "").strip())
    except json.JSONDecodeError:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _format_step_names(steps: list) -> str:
    names: list[str] = []
    for step in steps:
        if not isinstance(step, dict):
            continue
        step_type = str(step.get("type", "")).strip()
        step_name = str(step.get("name", "")).strip()
        names.append("/".join(part for part in (step_type, step_name) if part))
    return ",".join(name for name in names if name)


def _compact_text(value, max_chars: int) -> str:
    text = " ".join(str(value or "").split())
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 1)] + "…"


if __name__ == "__main__":
    main()
