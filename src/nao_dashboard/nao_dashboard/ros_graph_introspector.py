"""ROS graph introspection helpers for dashboard snapshots."""

from __future__ import annotations

import time

from nao_dashboard.models import RosGraphSnapshot


def build_ros_graph_snapshot(node) -> RosGraphSnapshot:
    node_rows = _node_rows(node)
    topic_rows = _topic_rows(node)
    service_rows = _service_rows(node)
    action_rows = _action_rows(node)

    return RosGraphSnapshot(
        timestamp=time.time(),
        nodes=node_rows,
        topics=topic_rows,
        services=service_rows,
        actions=action_rows,
        edges=[],
    )


def discovered_action_names(node) -> set[str]:
    action_rows = _action_rows(node)
    return {str(item.get('name', '')).strip() for item in action_rows if str(item.get('name', '')).strip()}


def _node_rows(node) -> list[dict]:
    rows: list[dict] = []
    names_and_ns = node.get_node_names_and_namespaces()
    for name, namespace in sorted(names_and_ns):
        rows.append(
            {
                'name': str(name).strip(),
                'namespace': str(namespace).strip(),
            }
        )
    return rows


def _topic_rows(node) -> list[dict]:
    rows: list[dict] = []
    topics = node.get_topic_names_and_types(no_demangle=False)
    for name, types in sorted(topics):
        topic_name = str(name).strip()
        type_list = [str(item).strip() for item in types if str(item).strip()]
        rows.append(
            {
                'name': topic_name,
                'types': type_list,
                'publisher_count': _safe_len(node.get_publishers_info_by_topic(topic_name)),
                'subscriber_count': _safe_len(node.get_subscriptions_info_by_topic(topic_name)),
            }
        )
    return rows


def _service_rows(node) -> list[dict]:
    rows: list[dict] = []
    services = node.get_service_names_and_types()
    for name, types in sorted(services):
        rows.append(
            {
                'name': str(name).strip(),
                'types': [str(item).strip() for item in types if str(item).strip()],
            }
        )
    return rows


def _action_rows(node) -> list[dict]:
    if not hasattr(node, 'get_action_names_and_types'):
        return []
    rows: list[dict] = []
    try:
        actions = node.get_action_names_and_types()
    except Exception:
        return []
    for name, types in sorted(actions):
        rows.append(
            {
                'name': str(name).strip(),
                'types': [str(item).strip() for item in types if str(item).strip()],
            }
        )
    return rows


def _safe_len(value) -> int:
    try:
        return len(value)
    except Exception:
        return 0
