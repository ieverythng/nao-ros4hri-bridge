"""Small local publisher for planner and WME fixture messages."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:  # pragma: no cover - runtime dependency
    from hri_actions_msgs.msg import Intent
except ImportError:  # pragma: no cover - runtime dependency
    Intent = None


_SAMPLE_SCENE = {
    'observer': 'myself',
    'backend': 'fixture',
    'objects': [
        {
            'entity_id': 'cup_1',
            'label': 'cup',
            'kb_class': 'Cup',
            'score': 0.95,
            'source': 'fixture',
            'last_seen_sec': 0.0,
        }
    ],
}

_SAMPLE_REQUEST = {
    'request_id': 'fixture_request_1',
    'user_text': 'look at the cup',
    'normalized_intents': ['inspect_scene'],
    'ack_text': 'I will inspect the scene.',
    'ack_mode': 'auto',
    'scene_targets': ['cup'],
    'dialogue_context': ['User asked me to inspect the cup.'],
    'grounded_context': {'target': 'cup'},
    'planner_mode': 'default',
}

_SAMPLE_FEEDBACK = {
    'plan_id': 'fixture_plan_1',
    'status': 'failed',
    'reason': 'target left the field of view',
    'retry_budget': 1,
    'scene_targets': ['cup'],
    'step': {'id': 'step_1', 'type': 'look_at', 'name': 'look_at'},
    'timestamp_sec': 0.0,
}


class FixturePublisher(Node):
    """Publish one fixture message and exit."""

    def __init__(self, kind: str, topic: str, payload: dict, intent_name: str) -> None:
        super().__init__('planner_fixture_publisher')
        self._kind = kind
        self._topic = topic
        self._payload = dict(payload)
        self._intent_name = intent_name

    def publish_once(self) -> None:
        if self._kind == 'request':
            if Intent is None:
                raise RuntimeError('hri_actions_msgs is required for request fixtures')
            publisher = self.create_publisher(Intent, self._topic, 1)
            msg = Intent()
            msg.intent = self._intent_name
            msg.source = 'planner_fixture'
            msg.modality = 'fixture'
            msg.data = json.dumps(self._payload, sort_keys=True, separators=(',', ':'))
            self._publish_with_spin(publisher, msg)
            return

        publisher = self.create_publisher(String, self._topic, 1)
        msg = String()
        msg.data = json.dumps(self._payload, sort_keys=True, separators=(',', ':'))
        self._publish_with_spin(publisher, msg)

    def _publish_with_spin(self, publisher, msg) -> None:
        deadline = time.time() + 1.0
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.05)
        publisher.publish(msg)
        self.get_logger().info('Published %s fixture on %s' % (self._kind, self._topic))
        rclpy.spin_once(self, timeout_sec=0.1)


def _default_topic(kind: str) -> str:
    return {
        'scene': '/scene/summary',
        'request': '/planner/request',
        'feedback': '/planner/execution_feedback',
    }[kind]


def _default_payload(kind: str) -> dict:
    if kind == 'scene':
        return dict(_SAMPLE_SCENE)
    if kind == 'request':
        return dict(_SAMPLE_REQUEST)
    return dict(_SAMPLE_FEEDBACK)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Publish one planner/WME fixture message.')
    parser.add_argument('kind', choices=['scene', 'request', 'feedback'])
    parser.add_argument('--topic', default='')
    parser.add_argument('--json', default='')
    parser.add_argument('--intent-name', default='planner_request')
    parsed_args = parser.parse_args(args=args)

    payload = _default_payload(parsed_args.kind)
    if parsed_args.json:
        payload = json.loads(parsed_args.json)
    topic = parsed_args.topic or _default_topic(parsed_args.kind)

    rclpy.init(args=None)
    node = FixturePublisher(parsed_args.kind, topic, payload, parsed_args.intent_name)
    try:
        node.publish_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
