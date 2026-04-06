"""ROS node for planner_llm."""

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from planner_common import DEFAULT_PLANNER_REQUEST_INTENT
from planner_common import ExecutionFeedback
from planner_common import PlannerRequest
from planner_common import parse_json_object
from planner_llm.planner_engine import PlannerDecision
from planner_llm.planner_engine import PlannerEngine
from planner_llm.providers import PlannerProviderConfig
from planner_llm.providers import build_provider

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover - runtime dependency
    Intent = None


class PlannerNode(Node):
    """Subscribe to planner requests and emit executable plan intents."""

    def __init__(self) -> None:
        super().__init__('planner_llm')
        if Intent is None:
            raise RuntimeError('hri_actions_msgs is required to run planner_llm')

        self.declare_parameter('planner_request_topic', '/planner/request')
        self.declare_parameter('intent_topic', '/intents')
        self.declare_parameter('planner_feedback_topic', '/planner/execution_feedback')
        self.declare_parameter('enriched_snapshot_topic', '/world_model/enriched_snapshot')
        self.declare_parameter('enriched_text_topic', '/world_model/enriched_text')
        self.declare_parameter('planner_request_intent', DEFAULT_PLANNER_REQUEST_INTENT)
        self.declare_parameter('default_intent_name', Intent.RAW_USER_INPUT)
        self.declare_parameter('provider', 'ollama')
        self.declare_parameter('model', 'gpt-oss:120b-cloud')
        self.declare_parameter('base_url', 'http://127.0.0.1:11434')
        self.declare_parameter('api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_tokens', 800)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('default_retry_budget', 1)
        self.declare_parameter('auto_replan', True)

        self._planner_request_topic = str(self.get_parameter('planner_request_topic').value).strip()
        self._intent_topic = str(self.get_parameter('intent_topic').value).strip()
        self._planner_feedback_topic = str(
            self.get_parameter('planner_feedback_topic').value
        ).strip()
        self._enriched_snapshot_topic = str(
            self.get_parameter('enriched_snapshot_topic').value
        ).strip()
        self._enriched_text_topic = str(self.get_parameter('enriched_text_topic').value).strip()
        self._planner_request_intent = str(
            self.get_parameter('planner_request_intent').value
        ).strip() or DEFAULT_PLANNER_REQUEST_INTENT
        self._default_intent_name = str(
            self.get_parameter('default_intent_name').value
        ).strip() or Intent.RAW_USER_INPUT
        self._auto_replan = bool(self.get_parameter('auto_replan').value)

        provider_config = PlannerProviderConfig(
            provider=str(self.get_parameter('provider').value).strip() or 'ollama',
            model=str(self.get_parameter('model').value).strip() or 'gpt-oss:120b-cloud',
            base_url=str(self.get_parameter('base_url').value).strip() or 'http://127.0.0.1:11434',
            api_key_env=str(self.get_parameter('api_key_env').value).strip() or 'OPENAI_API_KEY',
            temperature=float(self.get_parameter('temperature').value),
            max_tokens=int(self.get_parameter('max_tokens').value),
            timeout_sec=float(self.get_parameter('timeout_sec').value),
        )
        provider = build_provider(provider_config)
        self._engine = PlannerEngine(
            provider,
            default_intent_name=self._default_intent_name,
            default_retry_budget=int(self.get_parameter('default_retry_budget').value),
        )

        self._intent_pub = self.create_publisher(Intent, self._intent_topic, 10)
        self.create_subscription(Intent, self._planner_request_topic, self._on_planner_request, 10)
        self.create_subscription(String, self._planner_feedback_topic, self._on_feedback, 10)
        self.create_subscription(String, self._enriched_snapshot_topic, self._on_world_snapshot, 10)
        self.create_subscription(String, self._enriched_text_topic, self._on_world_text, 10)

        self._world_snapshot_payload: dict = {}
        self._world_text = ''
        self._active_requests: dict[str, PlannerRequest] = {}

        self.get_logger().info(
            'planner_llm ready | request=%s intents=%s feedback=%s snapshot=%s text=%s provider=%s model=%s auto_replan=%s'
            % (
                self._planner_request_topic,
                self._intent_topic,
                self._planner_feedback_topic,
                self._enriched_snapshot_topic,
                self._enriched_text_topic,
                provider_config.provider,
                provider_config.model,
                self._auto_replan,
            )
        )

    def _on_world_snapshot(self, msg: String) -> None:
        self._world_snapshot_payload = parse_json_object(msg.data)

    def _on_world_text(self, msg: String) -> None:
        self._world_text = str(msg.data or '').strip()

    def _on_planner_request(self, msg: Intent) -> None:
        if msg.intent and str(msg.intent).strip() != self._planner_request_intent:
            self.get_logger().warn(
                'planner_llm received unexpected request intent=%s on %s; continuing anyway'
                % (msg.intent, self._planner_request_topic)
            )
        planner_request = PlannerRequest.from_payload(msg.data)
        decision = self._engine.plan_request(
            planner_request,
            world_model_text=self._world_text,
            world_model_snapshot=self._world_snapshot_payload,
            feedback=None,
        )
        self._publish_decision(decision, modality=getattr(msg, 'modality', ''), source='planner_llm')
        self._active_requests[decision.plan_id] = planner_request

    def _on_feedback(self, msg: String) -> None:
        feedback = ExecutionFeedback.from_payload(msg.data)

        if feedback.status == 'completed':
            self._active_requests.pop(feedback.plan_id, None)
            return
        if not self._auto_replan:
            return
        if feedback.status not in ('failed', 'invalid'):
            return

        planner_request = self._active_requests.pop(feedback.plan_id, None)
        if planner_request is None:
            return

        decision = self._engine.plan_request(
            planner_request,
            world_model_text=self._world_text,
            world_model_snapshot=self._world_snapshot_payload,
            feedback=feedback,
        )
        self._publish_decision(decision, modality='planner_replan', source='planner_llm')
        self._active_requests[decision.plan_id] = planner_request

    def _publish_decision(self, decision: PlannerDecision, *, modality: str, source: str) -> None:
        msg = Intent()
        msg.intent = decision.intent_name
        msg.modality = str(modality or '')
        msg.source = str(source or 'planner_llm')
        msg.data = json.dumps(decision.payload, sort_keys=True, separators=(',', ':'))
        self._intent_pub.publish(msg)


def main(args=None) -> None:
    """Run planner_llm as a standard rclpy node."""
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # pragma: no cover - manual shutdown
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
