"""Dashboard backend node: event taps + ROS graph + AB registry + HTTP API."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import threading
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

from interaction_trace_viewer.payload_normalizer import classify_speech_topic
from interaction_trace_viewer.payload_normalizer import normalize_intent_message
from interaction_trace_viewer.payload_normalizer import normalize_rosout_message
from interaction_trace_viewer.payload_normalizer import normalize_string_message
from interaction_trace_viewer.trace_model import TraceRecorder

from nao_dashboard.models import DashboardEvent
from nao_dashboard.models import make_run_id
from nao_dashboard.node_health import action_health
from nao_dashboard.registry_adapter import build_registry_snapshot
from nao_dashboard.ros_graph_introspector import build_ros_graph_snapshot
from nao_dashboard.ros_graph_introspector import discovered_action_names
from nao_dashboard.websocket_server import DashboardHttpServer

try:  # pragma: no cover - runtime dependency
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except Exception:  # pragma: no cover
    Intent = None


class NaoDashboardNode(Node):
    """Serve one compact observability hub for the NAO stack."""

    def __init__(self) -> None:
        super().__init__('nao_dashboard')

        self.declare_parameter('dashboard_enabled', True)
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('discovery_period_sec', 1.0)
        self.declare_parameter('graph_refresh_period_sec', 1.5)
        self.declare_parameter('registry_refresh_period_sec', 3.0)
        self.declare_parameter('max_events', 400)
        self.declare_parameter('max_payload_chars', 4000)
        self.declare_parameter(
            'expected_actions',
            [
                '/skill/scan',
                '/skill/report_result',
                '/skill/say',
                '/skill/look_at',
                '/skill/do_head_motion',
                '/skill/replay_motion',
                '/skill/fake/navigate_to',
                '/skill/fake/find_object',
                '/skill/fake/wave_greet',
                '/skill/fake/inspect_area',
                '/skill/fake/walk_to',
            ],
        )

        self.enabled = bool(self.get_parameter('dashboard_enabled').value)
        self.max_events = max(50, int(self.get_parameter('max_events').value))
        self.max_payload_chars = max(0, int(self.get_parameter('max_payload_chars').value))
        self._discovery_period_sec = max(0.5, float(self.get_parameter('discovery_period_sec').value))
        self._graph_refresh_period_sec = max(0.5, float(self.get_parameter('graph_refresh_period_sec').value))
        self._registry_refresh_period_sec = max(1.0, float(self.get_parameter('registry_refresh_period_sec').value))
        self._expected_actions = [str(item).strip() for item in self.get_parameter('expected_actions').value if str(item).strip()]

        self._topic_subscriptions: dict[str, object] = {}
        self._trace_recorder = TraceRecorder()
        self._run_id = make_run_id()
        self._event_seq = 0
        self._events: list[DashboardEvent] = []
        self._lock = threading.Lock()

        self._ros_graph_snapshot = build_ros_graph_snapshot(self)
        self._registry_snapshot = build_registry_snapshot()
        self._action_health = action_health(
            expected_actions=self._expected_actions,
            discovered_actions=discovered_action_names(self),
        )

        self._known_topic_specs = {
            '/planner/request': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/intents': ('hri_actions_msgs/msg/Intent', self._subscribe_intent),
            '/planner/execution_feedback': ('std_msgs/msg/String', self._subscribe_string),
            '/planner/dialogue_act': ('std_msgs/msg/String', self._subscribe_string),
            '/scene/summary': ('std_msgs/msg/String', self._subscribe_string),
            '/fake_skills/events': ('std_msgs/msg/String', self._subscribe_string),
            '/stack_observer/events': ('std_msgs/msg/String', self._subscribe_string),
            '/rosout': ('rcl_interfaces/msg/Log', self._subscribe_rosout),
        }

        if not self.enabled:
            self.get_logger().warn('nao_dashboard disabled via parameter dashboard_enabled=false')
            return

        self._discover_timer = self.create_timer(self._discovery_period_sec, self._discover_topics)
        self._graph_timer = self.create_timer(self._graph_refresh_period_sec, self._refresh_graph)
        self._registry_timer = self.create_timer(self._registry_refresh_period_sec, self._refresh_registry)

        web_dir = self._resolve_web_dir()
        self._http_server = DashboardHttpServer(
            host=str(self.get_parameter('http_host').value).strip(),
            port=int(self.get_parameter('http_port').value),
            web_dir=str(web_dir),
            state_provider=self._state_payload,
        )
        self._http_server.start()

        self._discover_topics()
        self.get_logger().info('nao_dashboard ready at %s' % self._http_server.base_url)

    # ------------------------------------------------------------------
    # Discovery and subscriptions
    # ------------------------------------------------------------------

    def _discover_topics(self) -> None:
        topics = dict(self.get_topic_names_and_types())

        for topic_name, (expected_type, subscribe_fn) in self._known_topic_specs.items():
            if topic_name in self._topic_subscriptions:
                continue
            topic_types = topics.get(topic_name, [])
            if expected_type not in topic_types:
                continue
            subscribe_fn(topic_name)

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
            self.get_logger().warn('dashboard skipped %s: hri_actions_msgs/Intent unavailable' % topic_name)
            return
        self._topic_subscriptions[topic_name] = self.create_subscription(
            Intent,
            topic_name,
            lambda msg, channel=topic_name: self._on_intent(channel, msg),
            10,
        )

    def _subscribe_string(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            String,
            topic_name,
            lambda msg, channel=topic_name: self._on_string(channel, msg),
            10,
        )

    def _subscribe_speech(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            String,
            topic_name,
            lambda msg, channel=topic_name: self._on_speech(channel, msg),
            10,
        )

    def _subscribe_rosout(self, topic_name: str) -> None:
        self._topic_subscriptions[topic_name] = self.create_subscription(
            Log,
            topic_name,
            lambda msg, channel=topic_name: self._on_rosout(channel, msg),
            100,
        )

    # ------------------------------------------------------------------
    # Event callbacks
    # ------------------------------------------------------------------

    def _on_intent(self, channel: str, msg) -> None:
        event = normalize_intent_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        self._record_event(event)

    def _on_string(self, channel: str, msg: String) -> None:
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        self._record_event(event)

    def _on_speech(self, channel: str, msg: String) -> None:
        event = normalize_string_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        event = replace(event, event_type=classify_speech_topic(channel))
        self._record_event(event)

    def _on_rosout(self, channel: str, msg: Log) -> None:
        event = normalize_rosout_message(channel=channel, msg=msg, max_payload_chars=self.max_payload_chars)
        if str(event.payload.get('name', '')).strip() == self.get_name():
            return
        self._record_event(event)

    def _record_event(self, event) -> None:
        timestamp = event.timestamp if event.timestamp > 0.0 else time.time()
        source_node = str(event.payload.get('name', '')).strip() if event.event_type == 'rosout' else ''
        normalized = replace(event, timestamp=timestamp, source_node=source_node)
        traced = self._trace_recorder.add(normalized)

        with self._lock:
            self._event_seq += 1
            dashboard_event = DashboardEvent(
                timestamp=traced.timestamp,
                event_id='event_%06d' % self._event_seq,
                run_id=self._run_id,
                trace_id=traced.trace_id,
                source=traced.source_node,
                event_type=traced.event_type,
                channel=traced.channel,
                ab_object_id=traced.ab_object_id,
                ab_level=traced.ab_level,
                payload_summary=traced.summary,
                payload=traced.payload,
            )
            self._events.append(dashboard_event)
            if len(self._events) > self.max_events:
                self._events = self._events[-self.max_events :]

    # ------------------------------------------------------------------
    # Snapshot refresh
    # ------------------------------------------------------------------

    def _refresh_graph(self) -> None:
        graph = build_ros_graph_snapshot(self)
        health = action_health(
            expected_actions=self._expected_actions,
            discovered_actions=discovered_action_names(self),
        )
        with self._lock:
            self._ros_graph_snapshot = graph
            self._action_health = health

    def _refresh_registry(self) -> None:
        snapshot = build_registry_snapshot()
        with self._lock:
            self._registry_snapshot = snapshot

    # ------------------------------------------------------------------
    # HTTP state provider
    # ------------------------------------------------------------------

    def _state_payload(self) -> dict:
        with self._lock:
            events = [item.to_dict() for item in self._events]
            ros_graph = self._ros_graph_snapshot.to_dict()
            ab_registry = self._registry_snapshot.to_dict()
            action_rows = list(self._action_health)

        return {
            'updated_at': time.time(),
            'run_id': self._run_id,
            'events': events,
            'ros_graph': ros_graph,
            'ab_registry': ab_registry,
            'action_health': action_rows,
            'stats': {
                'event_count': len(events),
                'node_count': len(ros_graph.get('nodes', [])),
                'topic_count': len(ros_graph.get('topics', [])),
                'service_count': len(ros_graph.get('services', [])),
                'action_count': len(ros_graph.get('actions', [])),
                'ab_object_count': len(ab_registry.get('objects', [])),
            },
        }

    def _resolve_web_dir(self) -> Path:
        if get_package_share_directory is not None:
            try:
                return Path(get_package_share_directory('nao_dashboard')) / 'web'
            except Exception:
                pass
        return Path(__file__).resolve().parent.parent / 'web'

    def close(self) -> None:
        if hasattr(self, '_http_server') and self._http_server is not None:
            self._http_server.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaoDashboardNode()
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


if __name__ == '__main__':
    main()
