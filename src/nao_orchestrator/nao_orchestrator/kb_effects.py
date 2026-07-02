"""KnowledgeCore mutation helpers for nao_orchestrator.

This module isolates explicit KB effect execution from the main orchestrator so
planner-side knowledge mutations can evolve without growing the core node even
further.
"""

from __future__ import annotations

from typing import Any

from kb_skills.mutation_client import KnowledgeCoreMutationClient

KB_MUTATION_OPERATIONS = {
    'kb_add': 'add',
    'kb_remove': 'remove',
    'kb_revise': 'update',
}


def _statement_parts(statement: str) -> tuple[str, str, str]:
    tokens = str(statement or '').strip().split()
    if len(tokens) < 3:
        return '', '', ''
    return tokens[0], tokens[1], ' '.join(tokens[2:])


def _single_entity_statement(statement: str) -> str:
    tokens = str(statement or '').strip().split()
    if len(tokens) != 1:
        return ''
    token = tokens[0]
    if any(char in token for char in ('"', "'", '{', '}', '[', ']')):
        return ''
    return token


def _binding_value(row: dict, key: str) -> str:
    if not isinstance(row, dict):
        return ''
    return str(row.get(key, row.get('?%s' % key, ''))).strip()


def _statement_from_binding(subject: str, predicate: str, row: dict) -> str:
    clean_subject = str(subject or '').strip()
    clean_predicate = str(predicate or '').strip()
    clean_object = _binding_value(row, 'object')
    if not clean_subject or not clean_predicate or not clean_object:
        return ''
    return '%s %s %s' % (clean_subject, clean_predicate, clean_object)


def _dedupe_statements(statements: list[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for statement in statements:
        clean = str(statement or '').strip()
        if not clean or clean in seen:
            continue
        seen.add(clean)
        deduped.append(clean)
    return deduped


def remove_previous_kb_values(
    statements: list[str],
    models: list[str],
    *,
    query_client,
    mutation_client,
) -> tuple[list[str], str]:
    """Retract existing subject/predicate values before a KB revise update."""
    if query_client is None or mutation_client is None:
        return [], ''
    removals: list[str] = []
    query_models = models if isinstance(models, list) and models else ['default']
    for statement in statements:
        subject, predicate, new_object = _statement_parts(statement)
        if not subject or not predicate or not new_object:
            continue
        rows = query_client.query_rows(
            patterns=['%s %s ?object' % (subject, predicate)],
            query_vars=['?object'],
            models=query_models,
        )
        removals.extend(
            _statement_from_binding(subject, predicate, row)
            for row in rows
            if _binding_value(row, 'object') != new_object
        )
    removals = _dedupe_statements(removals)
    if not removals:
        return [], ''
    result = mutation_client.mutate(
        operation='remove',
        statements=removals,
        models=models if isinstance(models, list) else [],
        wait_for_result=True,
    )
    if result.success:
        return removals, ''
    return removals, result.error_msg or 'KnowledgeCore previous-value removal failed'


def expand_kb_remove_statements(
    statements: list[str],
    models: list[str],
    *,
    query_client,
) -> list[str]:
    """Expand entity-only remove requests into current concrete KB facts."""
    if query_client is None:
        return statements
    expanded: list[str] = []
    query_models = models if isinstance(models, list) and models else ['default']
    for statement in statements:
        subject, predicate, obj = _statement_parts(statement)
        if subject and predicate and obj:
            expanded.append(statement)
            continue
        subject = _single_entity_statement(statement)
        if not subject:
            expanded.append(statement)
            continue
        rows = query_client.query_rows(
            patterns=['%s ?predicate ?object' % subject],
            query_vars=['?predicate', '?object'],
            models=query_models,
        )
        expanded.extend(
            _statement_from_binding(subject, _binding_value(row, 'predicate'), row)
            for row in rows
        )
    return _dedupe_statements(expanded)


def execute_kb_mutation_step(
    step_name: str,
    step_args: dict,
    *,
    mutation_client,
    query_client,
    stats: Any,
    on_started=None,
) -> tuple[bool, str, dict]:
    """Delegate one explicit planner mutation to the KnowledgeCore seam."""
    statements = step_args.get('statements', step_args.get('statement', []))
    statements = KnowledgeCoreMutationClient.coerce_statements(statements)
    models = step_args.get('models', [])
    if isinstance(models, str):
        models = [models]
    if on_started is not None:
        on_started()
    if mutation_client is None:
        stats.dispatch_failures += 1
        return False, 'KnowledgeCore mutation client is unavailable', {}

    if step_name == 'kb_revise':
        removed, reason = remove_previous_kb_values(
            statements,
            models,
            query_client=query_client,
            mutation_client=mutation_client,
        )
        if reason:
            stats.dispatch_failures += 1
            return False, reason, {
                'skill': step_name,
                'operation': 'remove_previous_values',
                'statement_count': len(removed),
                'dispatched': bool(removed),
                'success': False,
            }
    elif step_name == 'kb_remove':
        statements = expand_kb_remove_statements(
            statements,
            models,
            query_client=query_client,
        )

    result = mutation_client.mutate(
        operation=KB_MUTATION_OPERATIONS[step_name],
        statements=statements,
        models=models if isinstance(models, list) else [],
        lifespan_sec=step_args.get('lifespan_sec', 0.0),
        wait_for_result=True,
    )
    payload = {
        'skill': step_name,
        'operation': result.operation,
        'statement_count': result.statement_count,
        'dispatched': result.dispatched,
        'success': result.success,
    }
    if result.success:
        stats.dispatched_kb_mutation += 1
        return True, 'KnowledgeCore mutation completed', payload
    stats.dispatch_failures += 1
    payload['error_msg'] = result.error_msg
    return False, result.error_msg or 'KnowledgeCore mutation failed', payload
