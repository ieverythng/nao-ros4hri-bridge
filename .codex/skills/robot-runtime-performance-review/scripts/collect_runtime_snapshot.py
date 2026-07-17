#!/usr/bin/env python3
"""Collect a compact live-runtime snapshot for NAO ROS4HRI review."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path


DEFAULT_PARAM_NODES = (
    '/nao_scene_grounding',
    '/chatbot_llm',
    '/hri_person_manager',
)

REQUIRED_NODES = (
    '/chatbot_llm',
    '/planner_llm',
    '/nao_orchestrator',
    '/dialogue_manager',
    '/kb/knowledge_core',
    '/fake_skill_server',
)

LIFECYCLE_NODES = (
    '/chatbot_llm',
    '/dialogue_manager',
    '/nao_orchestrator',
)

DEFAULT_TOPICS = (
    '/scene/summary',
    '/humans/persons/tracked',
    '/humans/faces/tracked',
)

HEAVY_TOPICS = (
    '/detected_objects',
)

_ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


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
    snapshot['ros']['lifecycle'] = {
        node: _docker_ros(args.container, f'ros2 lifecycle get {node}')
        for node in LIFECYCLE_NODES
    }
    snapshot['ros']['param_dumps'] = {
        node: _docker_ros(args.container, f'ros2 param dump {node}', timeout=6.0)
        for node in DEFAULT_PARAM_NODES
    }
    snapshot['ros']['knowledge_core_query'] = _docker_ros(
        args.container,
        "timeout 8 ros2 service call /kb/query kb_msgs/srv/Query "
        "\"{patterns: ['?subject rdf:type ?type'], vars: ['?subject'], "
        "models: ['default']}\"",
        timeout=12.0,
    )
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
    snapshot['derived']['preflight'] = _preflight_status(snapshot)
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
    fallback_metrics = _fallback_metrics(logs)
    fallback_event_metrics = _fallback_event_metrics(logs)
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
        'fallback_metrics': fallback_metrics,
        'fallback_total_count': fallback_metrics.get('total', 0),
        'fallback_event_metrics': fallback_event_metrics,
        'fallback_event_total_count': fallback_event_metrics.get('total', 0),
    }


def _preflight_status(snapshot: dict) -> dict:
    """Summarize whether semantic questionnaire scoring is admissible."""
    logs = snapshot.get('logs', {}).get('stdout', '') + snapshot.get('logs', {}).get('stderr', '')
    node_result = snapshot.get('ros', {}).get('nodes', {})
    node_output = str(node_result.get('stdout', ''))
    missing_nodes = [node for node in REQUIRED_NODES if node not in node_output.splitlines()]
    lifecycle = snapshot.get('ros', {}).get('lifecycle', {})
    lifecycle_states = {
        node: _lifecycle_state(result.get('stdout', ''))
        for node, result in lifecycle.items()
    }
    non_active_lifecycle = {
        node: state
        for node, state in lifecycle_states.items()
        if state and not state.startswith('active ')
    }
    lifecycle_probe_failures = [
        node for node, state in lifecycle_states.items() if not state
    ]
    llm_preflight_failures = {
        'chatbot_llm': len(
            re.findall(
                r'\[LLM PREFLIGHT\].*chatbot required preflight failed',
                logs,
                flags=re.IGNORECASE,
            )
        ),
        'planner_llm': len(
            re.findall(
                r'\[LLM PREFLIGHT\].*planner tiny probe failed|'
                r'\[LLM PREFLIGHT\].*planner provider preflight failed',
                logs,
                flags=re.IGNORECASE,
            )
        ),
    }
    kb_query = snapshot.get('ros', {}).get('knowledge_core_query', {})
    knowledge_core_ready = bool(
        kb_query.get('ok', False)
        and re.search(r'\bsuccess\s*[=:]\s*true\b', str(kb_query.get('stdout', '')), re.I)
    ) or bool(re.search('KnowledgeCore ready', logs, flags=re.IGNORECASE))
    status = 'ready_for_semantic_scoring'
    if (
        not node_result.get('ok', False)
        or missing_nodes
        or non_active_lifecycle
        or lifecycle_probe_failures
        or any(llm_preflight_failures.values())
        or not knowledge_core_ready
    ):
        status = 'preflight_not_scored'
    return {
        'status': status,
        'required_nodes': list(REQUIRED_NODES),
        'missing_nodes': missing_nodes,
        'lifecycle_states': lifecycle_states,
        'non_active_lifecycle': non_active_lifecycle,
        'lifecycle_probe_failures': lifecycle_probe_failures,
        'llm_preflight_failures': llm_preflight_failures,
        'knowledge_core_ready': knowledge_core_ready,
    }


def _lifecycle_state(value: str) -> str:
    """Extract the lifecycle state from output that may include ANSI warnings."""
    clean = _ANSI_ESCAPE.sub('', str(value or ''))
    matches = re.findall(r'\b(?:unknown|unconfigured|inactive|active|finalized|error)\s+\[\d+\]', clean)
    return matches[-1] if matches else clean.strip()


def _fallback_metrics(logs: str) -> dict[str, int]:
    """Count observable fallback and recovery markers without scoring behavior."""
    text = str(logs or '')
    patterns = {
        'chatbot_llm_response_failed': r'llm response failed fallback',
        'chatbot_llm_disabled': r'llm disabled fallback response',
        'chatbot_rules_response_fallback': r'llm response fallback -> rules',
        'chatbot_rules_intent_fallback': r'rules_llm_intent_fallback',
        'chatbot_generic_fallback_intent': r"intent[=:]fallback|\"intent\"\\s*:\\s*\"fallback\"",
        'planner_invalid_json': r'model output did not contain a JSON object',
        'planner_invalid_executable_plan': r'valid executable plan|model output did not contain executable steps',
        'planner_gate_rejected': r'planner_gate_rejected',
        'planner_duplicate_active_goal': r'duplicate active planner goal',
        'planner_rule_fallback': r'rule_fallback',
        'planner_target_selection_recovery': r'validated_target_selection_recovery',
        'execution_report_fallback': r'fallback_execution_report|execution report.*fallback',
        'route_repair': r'llm_response_route_repair|route_conflict',
        'language_model_unreachable_speech': r'having trouble reaching my language model',
    }
    counts = {
        name: len(re.findall(pattern, text, flags=re.IGNORECASE))
        for name, pattern in patterns.items()
    }
    counts['total'] = sum(counts.values())
    return counts


def _fallback_event_metrics(logs: str) -> dict[str, int]:
    """Deduplicate fallback/recovery markers that are mirrored across logs."""
    text = str(logs or '')
    patterns = {
        'chatbot_llm_response_failed': r'llm response failed fallback',
        'chatbot_llm_disabled': r'llm disabled fallback response',
        'chatbot_rules_response_fallback': r'llm response fallback -> rules',
        'chatbot_rules_intent_fallback': r'rules_llm_intent_fallback',
        'chatbot_generic_fallback_intent': r"intent[=:]fallback|\"intent\"\\s*:\\s*\"fallback\"",
        'planner_invalid_json': r'model output did not contain a JSON object',
        'planner_invalid_executable_plan': r'valid executable plan|model output did not contain executable steps',
        'planner_gate_rejected': r'planner_gate_rejected',
        'planner_duplicate_active_goal': r'duplicate active planner goal',
        'planner_rule_fallback': r'rule_fallback',
        'planner_target_selection_recovery': r'validated_target_selection_recovery',
        'execution_report_fallback': r'fallback_execution_report|execution report.*fallback',
        'route_repair': r'llm_response_route_repair|route_conflict',
        'language_model_unreachable_speech': r'having trouble reaching my language model',
    }
    events_by_name = {name: set() for name in patterns}
    for index, line in enumerate(text.splitlines()):
        for name, pattern in patterns.items():
            if not re.search(pattern, line, flags=re.IGNORECASE):
                continue
            events_by_name[name].add(_fallback_event_key(line, index))
    counts = {name: len(events) for name, events in events_by_name.items()}
    counts['total'] = sum(counts.values())
    return counts


def _fallback_event_key(line: str, index: int) -> str:
    """Return a stable event key for one mirrored log line."""
    clean = str(line or '')
    for pattern in (
        r'turn[:=]([A-Za-z0-9_:.+-]+)',
        r'"turn_id"\\s*:\\s*"([^"]+)"',
        r'goal_id=([A-Za-z0-9_:.+-]+)',
        r'"goal_id"\\s*:\\s*"([^"]+)"',
        r'plan_id=([A-Za-z0-9_:.+-]+)',
        r'"plan_id"\\s*:\\s*"([^"]+)"',
    ):
        match = re.search(pattern, clean)
        if match:
            return match.group(1)
    stamp = re.search(r'\[(\d{10}(?:\.\d+)?)\]', clean)
    if stamp:
        return stamp.group(1)
    return 'line_%d' % index


if __name__ == '__main__':
    raise SystemExit(main())
