"""ROS node that records planner/orchestrator interaction traces."""

from __future__ import annotations

from dataclasses import replace
import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from interaction_trace_viewer.payload_normalizer import classify_speech_topic
from interaction_trace_viewer.payload_normalizer import normalize_intent_message
from interaction_trace_viewer.payload_normalizer import normalize_rosout_message
from interaction_trace_viewer.payload_normalizer import normalize_string_message
from interaction_trace_viewer.render_html import render_events_html
from interaction_trace_viewer.render_tui import format_event_line
from interaction_trace_viewer.trace_model import JsonlTraceWriter
from interaction_trace_viewer.trace_model import TraceRecorder

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except Exception:  # pragma: no cover
    Intent = None


class InteractionTraceNode(Node):
    """Subscribe to runtime topics and emit normalized trace events."""

    def __init__(self) -> None:
        super().__init__('interaction_trace_viewer')

        self.declare_parameter('trace_viewer_enabled', True)
        self.declare_parameter('compact_mode', True)
        self.declare_parameter('write_jsonl', True)
        self.declare_parameter('jsonl_output_dir', '~/.ros/nao_ros4hri_traces')
        self.declare_parameter('write_html_on_shutdown', True)
        self.declare_parameter('html_output_dir', '~/.ros/nao_ros4hri_trace_reports')
        self.declare_parameter('include_raw_payloads', True)
        self.declare_parameter('max_payload_chars', 4000)
        self.declare_parameter('discovery_period_sec', 2.0)

        self.enabled = bool(self.get_parameter('trace_viewer_enabled').value)
        self.compact_mode = bool(self.get_parameter('compact_mode').value)
        self.write_jsonl = bool(self.get_parameter('write_jsonl').value)
        self.write_html_on_shutdown = bool(self.get_parameter('write_html_on_shutdown').value)
        self.include_raw_payloads = bool(self.get_parameter('include_raw_payloads').value)
        self.max_payload_chars = max(0, int(self.get_parameter('max_payload_chars').value))
        self.discovery_period_sec = max(0.5, float(self.get_parameter('discovery_period_sec').value))

        self.jsonl_output_dir = str(self.get_parameter('jsonl_output_dir').value).strip() or '~/.ros/nao_ros4hri_traces'
        self.html_output_dir = str(self.get_parameter('html_output_dir').value).strip() or '~/.ros/nao_ros4hri_trace_reports'

        self._topic_subscriptions: dict[str, object] = {}
        self._trace_recorder = TraceRecorder()
        self._writer = JsonlTraceWriter(self.jsonl_output_dir) if self.write_jsonl else None

        self._known_topic_specs = {
            '/planner/request': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/intents': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/planner/execution_feedback': ('std_msgs/msg/String', self._subscribe_string),
            '/planner/dialogue_act': ('std_msgs/msg/String', self._subscribe_string),
            '/scene/summary': ('std_msgs/msg/String', self._subscribe_string),
            '/rosout': ('rcl_interfaces/msg/Log', self._subscribe_rosout),
        }

        if not self.enabled:
            self.get_logger().warn('interaction_trace_viewer disabled via parameter trace_viewer_enabled=false')
            return

        self._discovery_timer = self.create_timer(self.discovery_period_sec, self._discover_topics)
        self._discover_topics()
        self.get_logger().info('interaction_trace_viewer started')

    def _discover_topics(self) -> None:
        topics = dict(self.get_topic_names_and_types())

        for topic_name, (expected_type, subscribe_fn) in self._known_topic_specs.items():
            if topic_name in self._topic_subscriptions:
                continue
            topic_types = topics.get(topic_name, [])
            if expected_type not in topic_types:
                continue
            subscribe_fn(topic_name)

        # dynamic speech topic support
        for topic_name, topic_types in topics.items():
            if topic_name in self._topic_subscriptions:
                continue
            if not str(topic_name).endswith('/speech'):
                continue
            if 'std_msgs/msg/String' not in topic_types:
                continue
            self._subscribe_speech(topic_name)

    def _subscribe_intent(self, topic_name: str) -> None:
        if Intent is None:
            self.get_logger().warn(
                'trace skipped %s because hri_actions_msgs/Intent is unavailable in this environment'
                % topic_name
            )
            return
        self._topic_subscriptions[topic_name] = self.create_subscription(
            Intent,
            topic_name,
            lambda msg, channel=topic_name: self._on_intent(channel, msg),
            10,
        )
        self.get_logger().info('trace subscribed: %s' % topic_name)

    def _subscribe_string(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            String,
            topic_name,
            lambda msg, channel=topic_name: self._on_string(channel, msg),
            10,
        )
        self.get_logger().info('trace subscribed: %s' % topic_name)

    def _subscribe_speech(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            String,
            topic_name,
            lambda msg, channel=topic_name: self._on_speech(channel, msg),
            10,
        )
        self.get_logger().info('trace subscribed (speech): %s' % topic_name)

    def _subscribe_rosout(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            Log,
            topic_name,
            lambda msg, channel=topic_name: self._on_rosout(channel, msg),
            100,
        )
        self.get_logger().info('trace subscribed: %s' % topic_name)

    def _on_intent(self, channel: str, msg) -> None:
        event = normalize_intent_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        self._emit_event(event)

    def _on_string(self, channel: str, msg: String) -> None:
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        self._emit_event(event)

    def _on_speech(self, channel: str, msg: String) -> None:
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        event = replace(event, event_type=classify_speech_topic(channel))
        self._emit_event(event)

    def _on_rosout(self, channel: str, msg: Log) -> None:
        event = normalize_rosout_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        if str(event.payload.get('name', '')).strip() == self.get_name():
            return
        self._emit_event(event)

    def _emit_event(self, event) -> None:
        timestamp = event.timestamp if event.timestamp > 0.0 else time.time()
        source_node = str(event.payload.get('name', '')).strip() if event.event_type == 'rosout' else ''
        raw_payload = event.raw if self.include_raw_payloads else None

        normalized = replace(
            event,
            timestamp=timestamp,
            source_node=source_node,
            raw=raw_payload,
        )
        traced = self._trace_recorder.add(normalized)

        if self._writer is not None:
            self._writer.write(traced)

        print(format_event_line(traced, verbose=not self.compact_mode), flush=True)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()

        if not self.write_html_on_shutdown:
            return

        events = list(self._trace_recorder.events())
        if not events:
            return

        out_dir = os.path.expanduser(self.html_output_dir)
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        output_path = os.path.join(out_dir, 'interaction_trace_%s.html' % stamp)

        html_text = render_events_html(events, title='Interaction Trace Report %s' % stamp)
        with open(output_path, 'w', encoding='utf-8') as handle:
            handle.write(html_text)
        self.get_logger().info('trace HTML report written: %s' % output_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractionTraceNode()

    try:
        if node.enabled:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
