"""ROS node that publishes planner-facing enriched world snapshots."""

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from std_msgs.msg import String

from kb_skills.query_client import KnowledgeCoreQueryClient
from planner_common import ExecutionFeedback
from planner_common import SceneSummary
from planner_common import coerce_str_list
from nao_world_model_enricher.world_state import WorldModelState


class NaoWorldModelEnricher(Node):
    """Fuse scene summaries, KB rows, and executor feedback into WME outputs."""

    def __init__(self) -> None:
        super().__init__('nao_world_model_enricher')

        self.declare_parameter('scene_summary_topic', '/scene/summary')
        self.declare_parameter('planner_feedback_topic', '/planner/execution_feedback')
        self.declare_parameter('enriched_snapshot_topic', '/world_model/enriched_snapshot')
        self.declare_parameter('enriched_text_topic', '/world_model/enriched_text')
        self.declare_parameter('observer_name', 'myself')
        self.declare_parameter('publish_interval_sec', 0.5)
        self.declare_parameter('recent_after_sec', 4.0)
        self.declare_parameter('occluded_after_sec', 10.0)
        self.declare_parameter('stale_after_sec', 45.0)
        self.declare_parameter('max_kb_rows', 12)
        self.declare_parameter('summary_max_chars', 2400)
        self.declare_parameter('knowledge_enabled', True)
        self.declare_parameter('knowledge_query_service_name', '/kb/query')
        self.declare_parameter('knowledge_models', ['default'])
        self.declare_parameter('knowledge_query_interval_sec', 1.5)
        self.declare_parameter(
            'knowledge_query_patterns',
            ['{observer} sees ?entity', '?entity rdf:type ?type'],
        )
        self.declare_parameter('knowledge_query_vars', ['entity', 'type'])
        self.declare_parameter('mirror_runtime_facts', False)

        self._scene_summary_topic = str(self.get_parameter('scene_summary_topic').value).strip()
        self._planner_feedback_topic = str(
            self.get_parameter('planner_feedback_topic').value
        ).strip()
        self._enriched_snapshot_topic = str(
            self.get_parameter('enriched_snapshot_topic').value
        ).strip()
        self._enriched_text_topic = str(self.get_parameter('enriched_text_topic').value).strip()
        self._summary_max_chars = max(160, int(self.get_parameter('summary_max_chars').value))
        self._knowledge_enabled = bool(self.get_parameter('knowledge_enabled').value)
        self._knowledge_query_service_name = str(
            self.get_parameter('knowledge_query_service_name').value
        ).strip() or '/kb/query'
        self._knowledge_query_interval_sec = max(
            0.5,
            float(self.get_parameter('knowledge_query_interval_sec').value),
        )
        self._mirror_runtime_facts = bool(self.get_parameter('mirror_runtime_facts').value)

        try:
            raw_knowledge_models = self.get_parameter('knowledge_models').value
        except ParameterUninitializedException:
            raw_knowledge_models = ['default']
        try:
            raw_query_patterns = self.get_parameter('knowledge_query_patterns').value
        except ParameterUninitializedException:
            raw_query_patterns = ['{observer} sees ?entity', '?entity rdf:type ?type']
        try:
            raw_query_vars = self.get_parameter('knowledge_query_vars').value
        except ParameterUninitializedException:
            raw_query_vars = ['entity', 'type']

        self._knowledge_models = coerce_str_list(raw_knowledge_models) or ['default']
        self._knowledge_query_patterns = coerce_str_list(raw_query_patterns)
        self._knowledge_query_vars = coerce_str_list(raw_query_vars)

        self._state = WorldModelState(
            observer_name=str(self.get_parameter('observer_name').value).strip() or 'myself',
            recent_after_sec=float(self.get_parameter('recent_after_sec').value),
            occluded_after_sec=float(self.get_parameter('occluded_after_sec').value),
            stale_after_sec=float(self.get_parameter('stale_after_sec').value),
            max_kb_rows=int(self.get_parameter('max_kb_rows').value),
        )

        self._snapshot_pub = self.create_publisher(String, self._enriched_snapshot_topic, 10)
        self._text_pub = self.create_publisher(String, self._enriched_text_topic, 10)
        self._query_client = None
        if self._knowledge_enabled:
            self._query_client = KnowledgeCoreQueryClient(
                node=self,
                callback_group=None,
                service_name=self._knowledge_query_service_name,
                timeout_sec=0.4,
            )

        self._last_snapshot_payload = ''
        self._last_text_payload = ''

        self.create_subscription(
            String,
            self._scene_summary_topic,
            self._on_scene_summary,
            10,
        )
        self.create_subscription(
            String,
            self._planner_feedback_topic,
            self._on_execution_feedback,
            10,
        )
        self.create_timer(
            max(0.2, float(self.get_parameter('publish_interval_sec').value)),
            self._publish_if_changed,
        )
        if self._knowledge_enabled and self._query_client is not None:
            self.create_timer(self._knowledge_query_interval_sec, self._refresh_kb_rows)

        self.get_logger().info(
            'nao_world_model_enricher ready | scene=%s feedback=%s snapshot=%s text=%s kb=%s mirror=%s'
            % (
                self._scene_summary_topic,
                self._planner_feedback_topic,
                self._enriched_snapshot_topic,
                self._enriched_text_topic,
                self._knowledge_query_service_name if self._knowledge_enabled else 'disabled',
                self._mirror_runtime_facts,
            )
        )

    def _on_scene_summary(self, msg: String) -> None:
        summary = SceneSummary.from_payload(msg.data)
        self._state.apply_scene_summary(summary, now_sec=self._clock_now_sec())
        self._publish_if_changed()

    def _on_execution_feedback(self, msg: String) -> None:
        feedback = ExecutionFeedback.from_payload(msg.data)
        self._state.apply_execution_feedback(feedback, now_sec=self._clock_now_sec())
        self._publish_if_changed()

    def _refresh_kb_rows(self) -> None:
        if self._query_client is None:
            return
        patterns = self._state.build_kb_query_patterns(self._knowledge_query_patterns)
        if not patterns or not self._knowledge_query_vars:
            return
        rows = self._query_client.query_rows(
            patterns=patterns,
            query_vars=self._knowledge_query_vars,
            models=list(self._knowledge_models),
        )
        self._state.refresh_kb_rows(rows, now_sec=self._clock_now_sec())
        self._publish_if_changed()

    def _publish_if_changed(self) -> None:
        now_sec = self._clock_now_sec()
        snapshot_payload = self._state.build_snapshot_payload(now_sec=now_sec)
        snapshot_json = json.dumps(snapshot_payload, sort_keys=True, separators=(',', ':'))
        if snapshot_json != self._last_snapshot_payload:
            snapshot_msg = String()
            snapshot_msg.data = snapshot_json
            self._snapshot_pub.publish(snapshot_msg)
            self._last_snapshot_payload = snapshot_json

        text_payload = self._state.build_text(now_sec=now_sec, max_chars=self._summary_max_chars)
        if text_payload != self._last_text_payload:
            text_msg = String()
            text_msg.data = text_payload
            self._text_pub.publish(text_msg)
            self._last_text_payload = text_payload

    def _clock_now_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + float(now_msg.nanosec) / 1_000_000_000.0


def main(args=None) -> None:
    """Run the world-model enricher as a standard rclpy node."""
    rclpy.init(args=args)
    node = NaoWorldModelEnricher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # pragma: no cover - manual shutdown
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
