"""ROS node for planner_llm."""

from __future__ import annotations

import json
from dataclasses import replace

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
from planner_llm.skill_registry import SkillRegistry
from planner_llm.supervisor import PlannerSupervisor
from planner_llm.supervisor import SupervisorOutcome

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover - runtime dependency
    Intent = None


class PlannerNode(Node):
    """Subscribe to planner requests and publish supervisor outcomes."""

    def __init__(self) -> None:
        super().__init__('planner_llm')
        if Intent is None:
            raise RuntimeError('hri_actions_msgs is required to run planner_llm')

        self.declare_parameter('planner_request_topic', '/planner/request')
        self.declare_parameter('intent_topic', '/intents')
        self.declare_parameter('planner_feedback_topic', '/planner/execution_feedback')
        self.declare_parameter('planner_dialogue_act_topic', '/planner/dialogue_act')
        self.declare_parameter('enriched_snapshot_topic', '/world_model/enriched_snapshot')
        self.declare_parameter('enriched_text_topic', '/world_model/enriched_text')
        self.declare_parameter('planner_request_intent', DEFAULT_PLANNER_REQUEST_INTENT)
        self.declare_parameter('default_intent_name', Intent.RAW_USER_INPUT)
        self.declare_parameter('skill_registry_path', '')
        self.declare_parameter('provider', 'ollama')
        self.declare_parameter('model', 'gemma4:31b-cloud')
        self.declare_parameter('base_url', 'http://127.0.0.1:11434')
        self.declare_parameter('api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_tokens', 800)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('think', False)
        self.declare_parameter('preflight_enabled', True)
        self.declare_parameter('preflight_required', False)
        self.declare_parameter('preflight_timeout_sec', 45.0)
        self.declare_parameter('preflight_max_tokens', 64)
        self.declare_parameter('preflight_attempts', 1)
        self.declare_parameter('preflight_realistic_enabled', False)
        self.declare_parameter('default_retry_budget', 1)
        self.declare_parameter('auto_replan', True)

        self._planner_request_topic = self._text_parameter('planner_request_topic')
        self._intent_topic = self._text_parameter('intent_topic')
        self._planner_feedback_topic = self._text_parameter('planner_feedback_topic')
        self._planner_dialogue_act_topic = self._text_parameter(
            'planner_dialogue_act_topic',
            '/planner/dialogue_act',
        )
        self._enriched_snapshot_topic = self._text_parameter('enriched_snapshot_topic')
        self._enriched_text_topic = self._text_parameter('enriched_text_topic')
        self._planner_request_intent = self._text_parameter(
            'planner_request_intent',
            DEFAULT_PLANNER_REQUEST_INTENT,
        )
        self._default_intent_name = self._text_parameter(
            'default_intent_name',
            Intent.RAW_USER_INPUT,
        )
        self._auto_replan = bool(self.get_parameter('auto_replan').value)
        default_retry_budget = int(self.get_parameter('default_retry_budget').value)

        provider_config = self._provider_config()
        prov_lc = str(provider_config.provider or '').strip().lower()
        if provider_config.think and prov_lc in ('openai', 'openai_compatible', 'watsonow'):
            self.get_logger().warn(
                'think=True has no effect for OpenAI-compatible planner providers '
                '(only the Ollama adapter sends a think flag).'
            )
        if prov_lc in ('openai', 'openai_compatible', 'watsonow') and not str(
            provider_config.model or ''
        ).strip():
            self.get_logger().warn(
                'planner_llm model parameter is empty; set model to a concrete '
                'OpenAI-compatible model id before relying on planning output.'
            )
        if not self._run_provider_preflight(provider_config):
            raise RuntimeError('planner_llm provider preflight failed')

        provider = build_provider(provider_config)
        skill_registry = SkillRegistry.load(
            self._text_parameter('skill_registry_path'),
            logger=self.get_logger(),
        )
        engine = PlannerEngine(
            provider,
            skill_registry,
            default_intent_name=self._default_intent_name,
            default_retry_budget=default_retry_budget,
        )
        self._supervisor = PlannerSupervisor(engine, auto_replan=self._auto_replan)

        self._intent_pub = self.create_publisher(Intent, self._intent_topic, 10)
        self._dialogue_act_pub = self.create_publisher(
            String,
            self._planner_dialogue_act_topic,
            10,
        )
        self.create_subscription(Intent, self._planner_request_topic, self._on_planner_request, 10)
        self.create_subscription(String, self._planner_feedback_topic, self._on_feedback, 10)
        self.create_subscription(String, self._enriched_snapshot_topic, self._on_world_snapshot, 10)
        self.create_subscription(String, self._enriched_text_topic, self._on_world_text, 10)

        self._world_snapshot_payload: dict = {}
        self._world_text = ''

        self.get_logger().info(
            '[STACK READY] planner_llm ready | request=%s intents=%s feedback=%s dialogue_act=%s snapshot=%s text=%s provider=%s model=%s auto_replan=%s'
            % (
                self._planner_request_topic,
                self._intent_topic,
                self._planner_feedback_topic,
                self._planner_dialogue_act_topic,
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

    def _text_parameter(self, name: str, fallback: str = '') -> str:
        return str(self.get_parameter(name).value).strip() or str(fallback or '')

    def _provider_config(self) -> PlannerProviderConfig:
        return PlannerProviderConfig(
            provider=self._text_parameter('provider', 'ollama'),
            model=self._text_parameter('model', 'gemma4:31b-cloud'),
            base_url=self._text_parameter('base_url', 'http://127.0.0.1:11434'),
            api_key_env=self._text_parameter('api_key_env', 'OPENAI_API_KEY'),
            temperature=float(self.get_parameter('temperature').value),
            max_tokens=int(self.get_parameter('max_tokens').value),
            timeout_sec=float(self.get_parameter('timeout_sec').value),
            think=bool(self.get_parameter('think').value),
        )

    def _run_provider_preflight(self, provider_config: PlannerProviderConfig) -> bool:
        enabled = bool(self.get_parameter('preflight_enabled').value)
        required = bool(self.get_parameter('preflight_required').value)
        timeout_sec = max(0.5, float(self.get_parameter('preflight_timeout_sec').value))
        max_tokens = max(1, int(self.get_parameter('preflight_max_tokens').value))
        attempts = max(1, int(self.get_parameter('preflight_attempts').value))
        realistic_enabled = bool(self.get_parameter('preflight_realistic_enabled').value)
        if not enabled:
            return True

        self.get_logger().info(
            '[LLM PREFLIGHT] planner starting | provider=%s model=%s timeout=%.1fs '
            'required=%s attempts=%d realistic=%s'
            % (
                provider_config.provider,
                provider_config.model,
                timeout_sec,
                required,
                attempts,
                realistic_enabled,
            )
        )
        preflight_config = replace(
            provider_config,
            timeout_sec=timeout_sec,
            max_tokens=max_tokens,
            temperature=0.0,
        )
        provider = build_provider(preflight_config)
        for attempt in range(1, attempts + 1):
            if self._run_planner_readiness_attempt(
                provider,
                attempt=attempt,
                attempts=attempts,
                realistic_enabled=realistic_enabled,
            ):
                self.get_logger().info(
                    '[LLM PREFLIGHT] planner model ready | provider=%s model=%s attempt=%d/%d'
                    % (provider_config.provider, provider_config.model, attempt, attempts)
                )
                return True
        return not required

    def _run_planner_readiness_attempt(
        self,
        provider,
        *,
        attempt: int,
        attempts: int,
        realistic_enabled: bool,
    ) -> bool:
        try:
            text = provider.generate(
                [
                    {'role': 'system', 'content': 'Reply only with JSON. No prose.'},
                    {'role': 'user', 'content': 'Return {"ready":true}.'},
                ]
            )
        except Exception as err:
            self.get_logger().warn(
                '[LLM PREFLIGHT] planner tiny probe failed | attempt=%d/%d error=%s'
                % (attempt, attempts, err)
            )
            return False

        if _preflight_ready(text):
            return self._run_planner_realistic_probe(
                provider,
                attempt=attempt,
                attempts=attempts,
                enabled=realistic_enabled,
            )
        self.get_logger().error(
            '[LLM PREFLIGHT] planner returned invalid readiness payload | payload=%s'
            % _preview_text(text)
        )
        return False

    def _run_planner_realistic_probe(
        self,
        provider,
        *,
        attempt: int,
        attempts: int,
        enabled: bool,
    ) -> bool:
        if not enabled:
            return True
        try:
            text = provider.generate(
                [
                    {
                        'role': 'system',
                        'content': (
                            'You are planner_llm for a ROS4HRI robot. Reply with one '
                            'compact JSON plan object only.'
                        ),
                    },
                    {
                        'role': 'user',
                        'content': (
                            'Plan a demo request: move the head right, then report completion. '
                            'Use fields goal_id, plan_id, mode, steps.'
                        ),
                    },
                ]
            )
        except Exception as err:
            self.get_logger().warn(
                '[LLM PREFLIGHT] planner realistic probe failed | attempt=%d/%d error=%s'
                % (attempt, attempts, err)
            )
            return False
        if str(text or '').strip():
            return True
        self.get_logger().warn(
            '[LLM PREFLIGHT] planner realistic probe returned empty text | attempt=%d/%d'
            % (attempt, attempts)
        )
        return False

    def _on_planner_request(self, msg: Intent) -> None:
        if msg.intent and str(msg.intent).strip() != self._planner_request_intent:
            self.get_logger().warn(
                'planner_llm received unexpected request intent=%s on %s; continuing anyway'
                % (msg.intent, self._planner_request_topic)
            )

        planner_request = PlannerRequest.from_payload(msg.data)
        self.get_logger().info(
            'planner_llm request received | goal_id=%s request_id=%s kind=%s intents=%s scene_targets=%s source=%s'
            % (
                planner_request.goal_id,
                planner_request.request_id,
                planner_request.request_kind,
                list(planner_request.normalized_intents),
                list(planner_request.scene_targets),
                str(getattr(msg, 'source', '') or 'unknown'),
            )
        )

        outcome = self._supervisor.handle_request(
            planner_request,
            world_model_text=self._world_text,
            world_model_snapshot=self._world_snapshot_payload,
        )
        self._publish_outcome(
            outcome,
            modality=getattr(msg, 'modality', ''),
            source='planner_llm',
        )

    def _on_feedback(self, msg: String) -> None:
        feedback = ExecutionFeedback.from_payload(msg.data)
        self.get_logger().info(
            'planner_llm feedback received | goal_id=%s plan_id=%s version=%s event=%s status=%s step=%s/%s retry_budget=%s reason=%s'
            % (
                feedback.goal_id,
                feedback.plan_id,
                feedback.plan_version,
                feedback.event_type,
                feedback.status,
                feedback.step_type,
                feedback.step_name,
                feedback.retry_budget,
                feedback.reason,
            )
        )

        outcome = self._supervisor.handle_feedback(
            feedback,
            world_model_text=self._world_text,
            world_model_snapshot=self._world_snapshot_payload,
        )
        self._publish_outcome(outcome, modality='planner_feedback', source='planner_llm')

    def _publish_outcome(self, outcome: SupervisorOutcome, *, modality: str, source: str) -> None:
        for dialogue_act in outcome.dialogue_acts:
            msg = String()
            msg.data = json.dumps(
                {
                    'goal_id': dialogue_act.goal_id,
                    'plan_id': dialogue_act.plan_id,
                    'plan_version': dialogue_act.plan_version,
                    'act': dialogue_act.act,
                    'priority': dialogue_act.priority,
                    'await_user_response': dialogue_act.await_user_response,
                    'reason': dialogue_act.reason,
                    'text_hint': dialogue_act.text_hint,
                    'slots_needed': list(dialogue_act.slots_needed),
                    'context': dialogue_act.context,
                },
                sort_keys=True,
                separators=(',', ':'),
            )
            self._dialogue_act_pub.publish(msg)
            self.get_logger().info(
                'planner_llm published dialogue_act | goal_id=%s plan_id=%s version=%s act=%s await_user_response=%s'
                % (
                    dialogue_act.goal_id,
                    dialogue_act.plan_id,
                    dialogue_act.plan_version,
                    dialogue_act.act,
                    dialogue_act.await_user_response,
                )
            )

        if outcome.decision is None:
            return
        self._publish_decision(outcome.decision, modality=modality, source=source)

    def _publish_decision(self, decision: PlannerDecision, *, modality: str, source: str) -> None:
        msg = Intent()
        msg.intent = decision.intent_name
        msg.modality = str(modality or '')
        msg.source = str(source or 'planner_llm')
        msg.data = json.dumps(decision.payload, sort_keys=True, separators=(',', ':'))
        self._intent_pub.publish(msg)

        plan_payload = decision.payload.get('plan', {})
        steps = plan_payload.get('steps', [])
        self.get_logger().info(
            'planner_llm published decision | goal_id=%s plan_id=%s version=%s mode=%s steps=%s validation=%s'
            % (
                plan_payload.get('goal_id', ''),
                decision.plan_id,
                plan_payload.get('plan_version', 0),
                decision.mode,
                len(steps) if isinstance(steps, list) else 0,
                plan_payload.get('validation_status', ''),
            )
        )


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


def _preflight_ready(text: str) -> bool:
    clean_text = str(text or '').strip()
    if not clean_text:
        return False
    try:
        parsed = json.loads(clean_text)
    except json.JSONDecodeError:
        return '"ready"' in clean_text.lower() and 'true' in clean_text.lower()
    return isinstance(parsed, dict) and parsed.get('ready') is True


def _preview_text(text: str, max_len: int = 160) -> str:
    clean_text = ' '.join(str(text or '').split())
    if len(clean_text) <= max_len:
        return clean_text
    return clean_text[: max_len - 3] + '...'
