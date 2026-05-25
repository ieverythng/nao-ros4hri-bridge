"""ROS-less mock server for quick dashboard demos."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from nao_dashboard.websocket_server import DashboardHttpServer


def _build_state_provider():
    event_seq = {'value': 0}

    def _state() -> dict:
        now = time.time()
        event_seq['value'] += 1
        idx = event_seq['value']

        events = [
            {
                'timestamp': now - 2.0,
                'event_id': 'event_%06d' % max(1, idx - 2),
                'run_id': 'mock_run',
                'trace_id': 'trace_mock_%03d' % max(1, idx - 2),
                'source': 'chatbot_llm',
                'event_type': 'planner_request',
                'channel': '/planner/request',
                'ab_object_id': 'scan',
                'ab_level': 1,
                'payload_summary': 'goal=find the cup',
                'payload': {'goal_text': 'find the cup'},
            },
            {
                'timestamp': now - 1.0,
                'event_id': 'event_%06d' % max(1, idx - 1),
                'run_id': 'mock_run',
                'trace_id': 'trace_mock_%03d' % max(1, idx - 1),
                'source': 'nao_orchestrator',
                'event_type': 'execution_feedback',
                'channel': '/planner/execution_feedback',
                'ab_object_id': 'navigate_to',
                'ab_level': 1,
                'payload_summary': 'navigate_to | started',
                'payload': {'skill': 'navigate_to', 'status': 'started'},
            },
            {
                'timestamp': now,
                'event_id': 'event_%06d' % idx,
                'run_id': 'mock_run',
                'trace_id': 'trace_mock_%03d' % idx,
                'source': 'fake_skill_server',
                'event_type': 'skill_result',
                'channel': '/fake_skills/events',
                'ab_object_id': 'report_result',
                'ab_level': 1,
                'payload_summary': 'report_result | completed',
                'payload': {'skill': 'report_result', 'status': 'completed'},
            },
        ]

        ros_graph = {
            'timestamp': now,
            'nodes': [
                {'name': 'chatbot_llm', 'namespace': '/'},
                {'name': 'planner_llm', 'namespace': '/'},
                {'name': 'nao_orchestrator', 'namespace': '/'},
                {'name': 'fake_skill_server', 'namespace': '/'},
            ],
            'topics': [
                {'name': '/planner/request', 'types': ['hri_actions_msgs/msg/Intent'], 'publisher_count': 1, 'subscriber_count': 1},
                {'name': '/fake_skills/events', 'types': ['std_msgs/msg/String'], 'publisher_count': 1, 'subscriber_count': 1},
            ],
            'services': [],
            'actions': [
                {'name': '/skill/fake/navigate_to', 'types': ['nao_skills/action/ScanScene']},
                {'name': '/skill/report_result', 'types': ['nao_msgs/action/Say']},
            ],
            'edges': [],
        }

        action_health = [
            {'action_name': '/skill/fake/navigate_to', 'available': True, 'status': 'online'},
            {'action_name': '/skill/fake/find_object', 'available': True, 'status': 'online'},
            {'action_name': '/skill/report_result', 'available': True, 'status': 'online'},
            {'action_name': '/skill/scan', 'available': False, 'status': 'missing'},
        ]

        ab_registry = {
            'timestamp': now,
            'objects': [
                {'object_id': 'scan', 'ab_level': 1, 'robot_adapter_mapping': 'nao_orchestrator.scan'},
                {'object_id': 'navigate_to', 'ab_level': 1, 'robot_adapter_mapping': 'fake_skills.navigate_to'},
                {'object_id': 'report_result', 'ab_level': 1, 'robot_adapter_mapping': 'nao_orchestrator.report_result'},
            ],
            'edges': [
                {'from': 'scan_and_report', 'to': 'scan', 'kind': 'decomposes_to'},
                {'from': 'scan_and_report', 'to': 'report_result', 'kind': 'decomposes_to'},
            ],
            'validation_errors': [],
        }

        return {
            'updated_at': now,
            'run_id': 'mock_run',
            'events': events,
            'ros_graph': ros_graph,
            'ab_registry': ab_registry,
            'action_health': action_health,
            'stats': {
                'event_count': len(events),
                'node_count': len(ros_graph['nodes']),
                'topic_count': len(ros_graph['topics']),
                'service_count': len(ros_graph['services']),
                'action_count': len(ros_graph['actions']),
                'ab_object_count': len(ab_registry['objects']),
            },
        }

    return _state


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Run nao_dashboard mock server without ROS.')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--web-dir', default='')
    args = parser.parse_args(argv)

    if args.web_dir:
        web_dir = args.web_dir
    else:
        web_dir = str(Path(__file__).resolve().parent.parent / 'web')

    server = DashboardHttpServer(
        host=str(args.host).strip() or '127.0.0.1',
        port=int(args.port),
        web_dir=web_dir,
        state_provider=_build_state_provider(),
    )
    server.start()
    print('nao_dashboard mock server running at http://%s:%d' % (args.host, args.port), flush=True)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
