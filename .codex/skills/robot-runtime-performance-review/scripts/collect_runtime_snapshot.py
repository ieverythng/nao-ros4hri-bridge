#!/usr/bin/env python3
"""Collect a compact live-runtime snapshot for NAO ROS4HRI review."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path


DEFAULT_PARAM_NODES = (
    '/nao_scene_grounding',
    '/chatbot_llm',
    '/hri_person_manager',
)

DEFAULT_TOPICS = (
    '/scene/summary',
    '/humans/persons/tracked',
    '/humans/faces/tracked',
)

HEAVY_TOPICS = (
    '/detected_objects',
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--container', default='nao_ros2')
    parser.add_argument('--since', default='30m')
    parser.add_argument('--out', default='')
    parser.add_argument('--topic-timeout-sec', type=float, default=6.0)
    parser.add_argument(
        '--sample-topics',
        action='store_true',
        help='Sample contract topics with ros2 topic echo --once.',
    )
    parser.add_argument(
        '--include-heavy-topics',
        action='store_true',
        help='Also sample sparse/high-volume detector topics such as /detected_objects.',
    )
    args = parser.parse_args()

    snapshot = {
        'captured_at_unix_sec': time.time(),
        'container': args.container,
        'docker': _run(['docker', 'ps', '--format', '{{.Names}}\t{{.Status}}']),
        'logs': _run(['docker', 'logs', '--since', args.since, args.container]),
        'ros': {},
    }

    snapshot['ros']['nodes'] = _docker_ros(args.container, 'ros2 node list')
    snapshot['ros']['topics'] = _docker_ros(args.container, 'ros2 topic list')
    snapshot['ros']['param_dumps'] = {
        node: _docker_ros(args.container, f'ros2 param dump {node}', timeout=6.0)
        for node in DEFAULT_PARAM_NODES
    }
    topics = []
    if args.sample_topics:
        topics = list(DEFAULT_TOPICS)
        if args.include_heavy_topics:
            topics.extend(HEAVY_TOPICS)
    snapshot['ros']['topic_samples'] = {}
    for topic in topics:
        snapshot['ros']['topic_samples'][topic] = _docker_ros(
            args.container,
            f'timeout -k 1 {args.topic_timeout_sec:g} ros2 topic echo {topic} --once',
            timeout=max(2.0, args.topic_timeout_sec + 3.0),
        )

    _add_derived_metrics(snapshot)
    payload = json.dumps(snapshot, indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(payload + '\n', encoding='utf-8')
    else:
        print(payload)
    return 0


def _docker_ros(container: str, command: str, *, timeout: float = 12.0) -> dict:
    return _run(
        [
            'docker',
            'exec',
            container,
            'bash',
            '-lc',
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/ubuntu/ws/install/setup.bash 2>/dev/null || true; '
            + command,
        ],
        timeout=timeout,
    )


def _run(command: list[str], *, timeout: float = 20.0) -> dict:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except Exception as err:  # pragma: no cover - environment dependent
        return {'ok': False, 'returncode': None, 'stdout': '', 'stderr': str(err)}
    return {
        'ok': completed.returncode == 0,
        'returncode': completed.returncode,
        'stdout': completed.stdout,
        'stderr': completed.stderr,
    }


def _add_derived_metrics(snapshot: dict) -> None:
    logs = snapshot.get('logs', {}).get('stdout', '') + snapshot.get('logs', {}).get('stderr', '')
    lowered = logs.lower()
    snapshot['derived'] = {
        'warning_count': lowered.count('[warn') + lowered.count('[warning'),
        'error_count': lowered.count('[error'),
        'knowledge_expiry_count': lowered.count('removing expired statement'),
        'knowledge_update_count': lowered.count('updating ['),
        'knowledge_delete_count': lowered.count('deleting from ['),
        'face_skip_warning_count': lowered.count('processing too slow')
        + lowered.count('skipped 100'),
        'grounded_context_trace_count': lowered.count('grounded_context'),
        'planner_request_count': lowered.count('planner_request'),
        'report_result_count': lowered.count('report_result'),
    }


if __name__ == '__main__':
    raise SystemExit(main())
