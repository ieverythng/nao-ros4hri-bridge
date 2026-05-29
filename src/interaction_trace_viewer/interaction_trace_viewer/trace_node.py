"""ROS node that records planner/orchestrator interaction traces."""

from __future__ import annotations

from dataclasses import replace
import json
import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

from interaction_trace_viewer.payload_normalizer import classify_speech_topic
from interaction_trace_viewer.payload_normalizer import normalize_include_event_types
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
        self.declare_parameter('include_raw_payloads', False)
        self.declare_parameter('max_payload_chars', 4000)
        self.declare_parameter('include_channels_csv', '')
        self.declare_parameter('exclude_channels_csv', '')
        self.declare_parameter('include_event_types_csv', '')
        self.declare_parameter('exclude_event_types_csv', '')
        self.declare_parameter('discovery_period_sec', 2.0)
        self.declare_parameter('kb_snapshot_emit_period_sec', 0.8)
        self.declare_parameter('enable_scene_summary_channel', False)
        self.declare_parameter('scene_summary_emit_on_change_only', True)
        self.declare_parameter('scene_summary_min_interval_sec', 1.0)
        self.declare_parameter(
            'rosout_node_allowlist_csv',
            (
                'chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,'
                'report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,'
                'head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug'
            ),
        )
        self.declare_parameter('rosout_min_level', 'warn')

        self.enabled = bool(self.get_parameter('trace_viewer_enabled').value)
        self.compact_mode = bool(self.get_parameter('compact_mode').value)
        self.write_jsonl = bool(self.get_parameter('write_jsonl').value)
        self.write_html_on_shutdown = bool(self.get_parameter('write_html_on_shutdown').value)
        self.include_raw_payloads = bool(self.get_parameter('include_raw_payloads').value)
        self.max_payload_chars = max(0, int(self.get_parameter('max_payload_chars').value))
        self._include_channels = _parse_csv_set(self.get_parameter('include_channels_csv').value)
        self._exclude_channels = _parse_csv_set(self.get_parameter('exclude_channels_csv').value)
        self._include_event_types = _parse_csv_set(self.get_parameter('include_event_types_csv').value)
        self._exclude_event_types = _parse_csv_set(self.get_parameter('exclude_event_types_csv').value)
        self._include_event_types = normalize_include_event_types(
            include_channels=self._include_channels,
            include_event_types=self._include_event_types,
            exclude_event_types=self._exclude_event_types,
        )
        if (
            self._include_event_types
            and 'kb_snapshot' in self._include_event_types
            and self._include_channels & {'world_model/enriched_snapshot', 'world_model/enriched_text'}
        ):
            self.get_logger().info(
                'interaction_trace_viewer include_event_types normalized with kb_snapshot for world_model channels'
            )
        self.discovery_period_sec = max(0.5, float(self.get_parameter('discovery_period_sec').value))
        self.kb_snapshot_emit_period_sec = max(
            0.0,
            float(self.get_parameter('kb_snapshot_emit_period_sec').value),
        )
        self.enable_scene_summary_channel = bool(self.get_parameter('enable_scene_summary_channel').value)
        self.scene_summary_emit_on_change_only = bool(
            self.get_parameter('scene_summary_emit_on_change_only').value
        )
        self.scene_summary_min_interval_sec = max(
            0.0,
            float(self.get_parameter('scene_summary_min_interval_sec').value),
        )
        self._rosout_node_allowlist = _parse_csv_set(
            self.get_parameter('rosout_node_allowlist_csv').value
        )
        self._rosout_min_level = _coerce_rosout_level(
            self.get_parameter('rosout_min_level').value
        )

        self.jsonl_output_dir = str(self.get_parameter('jsonl_output_dir').value).strip() or '~/.ros/nao_ros4hri_traces'
        self.html_output_dir = str(self.get_parameter('html_output_dir').value).strip() or '~/.ros/nao_ros4hri_trace_reports'

        self._topic_subscriptions: dict[str, object] = {}
        self._trace_recorder = TraceRecorder()
        self._writer = JsonlTraceWriter(self.jsonl_output_dir) if self.write_jsonl else None
        self._last_kb_snapshot_hash = ''
        self._last_kb_snapshot_emit_sec = 0.0
        self._last_scene_summary_key = ''
        self._last_scene_summary_ts = 0.0

        self._known_topic_specs = {
            '/planner/request': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/intents': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/planner/execution_feedback': ('std_msgs/msg/String', self._subscribe_string),
            '/planner/dialogue_act': ('std_msgs/msg/String', self._subscribe_string),
            '/nao_orchestrator/planner_dialogue_act': ('std_msgs/msg/String', self._subscribe_string),
            '/chatbot_llm/turn_trace': ('std_msgs/msg/String', self._subscribe_string),
            '/fake_skills/events': ('std_msgs/msg/String', self._subscribe_string),
            '/world_model/enriched_snapshot': ('std_msgs/msg/String', self._subscribe_string),
            '/world_model/enriched_text': ('std_msgs/msg/String', self._subscribe_string),
            '/rosout': ('rcl_interfaces/msg/Log', self._subscribe_rosout),
        }
        if self.enable_scene_summary_channel:
            self._known_topic_specs['/scene/summary'] = ('std_msgs/msg/String', self._subscribe_string)

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
        if channel == '/scene/summary' and not self._should_emit_scene_summary(msg.data):
            return
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        if event.event_type == 'kb_snapshot' and not self._should_emit_kb_snapshot_event(event.payload):
            return
        self._emit_event(event)

    def _on_speech(self, channel: str, msg: String) -> None:
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        event = replace(event, event_type=classify_speech_topic(channel))
        self._emit_event(event)

    def _on_rosout(self, channel: str, msg: Log) -> None:
        event = normalize_rosout_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        if str(event.payload.get('name', '')).strip() == self.get_name():
            return
        source_node = _normalize_node_name(event.payload.get('name', ''))
        if self._rosout_node_allowlist and source_node not in self._rosout_node_allowlist:
            return
        if int(event.payload.get('level', 0) or 0) < self._rosout_min_level:
            return
        self._emit_event(event)

    def _should_emit_scene_summary(self, raw_payload: str) -> bool:
        now = time.time()
        if self.scene_summary_min_interval_sec > 0.0:
            elapsed = now - self._last_scene_summary_ts
            if elapsed < self.scene_summary_min_interval_sec:
                return False

        parsed = {}
        try:
            candidate = json.loads(str(raw_payload or '').strip())
            if isinstance(candidate, dict):
                parsed = candidate
        except Exception:
            parsed = {}

        if self.scene_summary_emit_on_change_only:
            objects = parsed.get('objects', []) if isinstance(parsed, dict) else []
            labels = [
                str(item.get('label', '')).strip().lower()
                for item in objects
                if isinstance(item, dict)
            ]
            labels = [label for label in labels if label]
            summary_key = ','.join(sorted(labels))
            if summary_key == self._last_scene_summary_key:
                return False
            self._last_scene_summary_key = summary_key

        self._last_scene_summary_ts = now
        return True

    def _emit_event(self, event) -> None:
        if not self._event_allowed(event):
            return
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

    def _should_emit_kb_snapshot_event(self, payload: dict) -> bool:
        current_hash = str(hash(json.dumps(payload, sort_keys=True, separators=(',', ':'))))
        now = time.time()
        if (
            current_hash == self._last_kb_snapshot_hash
            and self.kb_snapshot_emit_period_sec > 0.0
            and (now - self._last_kb_snapshot_emit_sec) < self.kb_snapshot_emit_period_sec
        ):
            return False
        self._last_kb_snapshot_hash = current_hash
        self._last_kb_snapshot_emit_sec = now
        return True

    def _event_allowed(self, event) -> bool:
        channel = str(event.channel or '').strip().lower()
        event_type = str(event.event_type or '').strip().lower()
        clean_channel = channel.strip('/')
        clean_event_type = event_type.strip('/')
        if self._include_channels and clean_channel not in self._include_channels:
            return False
        if self._exclude_channels and clean_channel in self._exclude_channels:
            return False
        if self._include_event_types and clean_event_type not in self._include_event_types:
            return False
        if self._exclude_event_types and clean_event_type in self._exclude_event_types:
            return False
        return True

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


def _parse_csv_set(raw_value) -> set[str]:
    return {
        _normalize_node_name(token).lower()
        for token in str(raw_value or '').split(',')
        if _normalize_node_name(token)
    }


def _normalize_node_name(value) -> str:
    return str(value or '').strip().strip('/')


def _coerce_rosout_level(raw_level) -> int:
    level_map = {
        'debug': 10,
        'info': 20,
        'warn': 30,
        'warning': 30,
        'error': 40,
        'fatal': 50,
    }
    clean_level = str(raw_level or '').strip().lower()
    if clean_level in level_map:
        return level_map[clean_level]
    try:
        return int(clean_level)
    except (TypeError, ValueError):
        return 30
