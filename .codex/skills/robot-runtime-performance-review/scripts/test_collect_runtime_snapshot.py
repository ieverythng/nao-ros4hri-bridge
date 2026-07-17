import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name('collect_runtime_snapshot.py')
SPEC = importlib.util.spec_from_file_location('collect_runtime_snapshot', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _snapshot(
    *,
    logs: str,
    nodes: str,
    lifecycle: dict[str, str],
    kb_query: dict | None = None,
) -> dict:
    return {
        'logs': {'stdout': logs, 'stderr': ''},
        'ros': {
            'nodes': {'ok': True, 'stdout': nodes},
            'lifecycle': {
                node: {'stdout': state}
                for node, state in lifecycle.items()
            },
            'knowledge_core_query': kb_query or {
                'ok': False,
                'stdout': '',
                'stderr': '',
            },
        },
    }


def test_preflight_rejects_missing_planner_and_llm_failure() -> None:
    result = MODULE._preflight_status(
        _snapshot(
            logs='[LLM PREFLIGHT] chatbot required preflight failed\n',
            nodes='/chatbot_llm\n/nao_orchestrator\n',
            lifecycle={
                '/chatbot_llm': 'unconfigured [1]',
                '/dialogue_manager': 'unconfigured [1]',
                '/nao_orchestrator': 'active [3]',
            },
        )
    )

    assert result['status'] == 'preflight_not_scored'
    assert '/planner_llm' in result['missing_nodes']
    assert result['llm_preflight_failures']['chatbot_llm'] == 1


def test_preflight_accepts_complete_active_graph() -> None:
    result = MODULE._preflight_status(
        _snapshot(
            logs='KnowledgeCore ready\n',
            nodes='\n'.join(MODULE.REQUIRED_NODES),
            lifecycle={
                '/chatbot_llm': '\x1b[33mtransport warning\x1b[0m\nactive [3]',
                '/dialogue_manager': 'active [3]',
                '/nao_orchestrator': 'active [3]',
            },
            kb_query={'ok': True, 'stdout': "success=True, json='[]'", 'stderr': ''},
        )
    )

    assert result['status'] == 'ready_for_semantic_scoring'
    assert result['missing_nodes'] == []
    assert result['non_active_lifecycle'] == {}


def test_preflight_uses_live_kb_query_when_ready_log_is_outside_window() -> None:
    result = MODULE._preflight_status(
        _snapshot(
            logs='',
            nodes='\n'.join(MODULE.REQUIRED_NODES),
            lifecycle={node: 'active [3]' for node in MODULE.LIFECYCLE_NODES},
            kb_query={'ok': True, 'stdout': "success=True, json='[]'", 'stderr': ''},
        )
    )

    assert result['knowledge_core_ready'] is True
    assert result['status'] == 'ready_for_semantic_scoring'


def test_fallback_metrics_count_validated_target_selection_recovery() -> None:
    logs = (
        'planner_llm published decision | '
        'mode=validated_target_selection_recovery steps=3\n'
    )

    raw = MODULE._fallback_metrics(logs)
    events = MODULE._fallback_event_metrics(logs)

    assert raw['planner_target_selection_recovery'] == 1
    assert events['planner_target_selection_recovery'] == 1
