"""KnowledgeCore post-effect helpers for orchestrated skills."""

from __future__ import annotations


SPATIAL_EFFECT_PREDICATES = ('oro:isOn', 'oro:isAt', 'oro:isIn')
SPATIAL_EFFECT_CLEANUP_PREDICATES = (
    'oro:isOn',
    'oro:isAt',
    'oro:isIn',
    'oro:contains',
    'oro:placeOf',
    'oro:isUnder',
)
SPATIAL_EFFECT_SKILLS = frozenset(
    {'bring_object', 'deliver_object', 'place_object', 'pick_object'}
)
_PERSON_TYPE_MARKERS = ('human', 'person')
_PLACE_TYPE_MARKERS = ('area', 'container', 'location', 'place', 'room', 'station')
_SUPPORT_TYPE_MARKERS = ('bench', 'counter', 'desk', 'shelf', 'surface', 'table')
_SPATIAL_PREDICATE_BY_LOCAL_NAME = {
    predicate.split(':', 1)[-1].lower(): predicate
    for predicate in SPATIAL_EFFECT_CLEANUP_PREDICATES
}


def validate_spatial_effect_statements(
    *,
    statements: list[str],
    models: list[str],
    query_client,
) -> str:
    """Reject spatial effects whose destination is not explicitly typed."""
    if query_client is None:
        return 'spatial effect validation requires KnowledgeCore query access'
    query_models = models if isinstance(models, list) and models else ['default']
    destination_markers = {
        'oro:isAt': _PERSON_TYPE_MARKERS + _PLACE_TYPE_MARKERS + _SUPPORT_TYPE_MARKERS,
        'oro:isIn': _PLACE_TYPE_MARKERS,
        'oro:isOn': _SUPPORT_TYPE_MARKERS,
    }
    for statement in statements:
        subject, predicate, obj = statement_parts(statement)
        canonical_predicate = canonical_spatial_predicate(predicate)
        if canonical_predicate not in SPATIAL_EFFECT_PREDICATES:
            continue
        if not subject or not obj or subject == obj:
            return 'spatial effect requires distinct subject and destination: %s' % statement
        if not _entity_has_type(
            obj,
            destination_markers[canonical_predicate],
            query_models,
            query_client,
        ):
            return 'spatial effect destination is not typed for %s: %s' % (
                canonical_predicate,
                obj,
            )
    return ''


def _entity_has_type(
    entity_id: str,
    markers: tuple[str, ...],
    query_models: list[str],
    query_client,
) -> bool:
    rows = query_client.query_rows(
        patterns=['%s rdf:type ?object' % entity_id],
        query_vars=['?object'],
        models=query_models,
    )
    return any(
        any(marker in binding_value(row, 'object').lower() for marker in markers)
        for row in rows
    )


def remove_stale_spatial_effect_values(
    *,
    statements: list[str],
    models: list[str],
    query_client,
    mutation_client,
) -> tuple[list[str], str]:
    """Remove stale spatial facts superseded by successful skill effects."""
    if query_client is None or mutation_client is None:
        return [], ''
    query_models = models if isinstance(models, list) and models else ['default']
    removals: list[str] = []
    for statement in statements:
        subject, predicate, new_object = statement_parts(statement)
        canonical_predicate = canonical_spatial_predicate(predicate)
        if canonical_predicate not in SPATIAL_EFFECT_PREDICATES or not subject or not new_object:
            continue
        forward_rows = query_client.query_rows(
            patterns=['%s ?predicate ?object' % subject],
            query_vars=['?predicate', '?object'],
            models=query_models,
        )
        for stale_predicate in SPATIAL_EFFECT_CLEANUP_PREDICATES:
            removals.extend(
                statement_from_values(
                    subject,
                    binding_value(row, 'predicate'),
                    binding_value(row, 'object'),
                )
                for row in forward_rows
                if canonical_spatial_predicate(
                    binding_value(row, 'predicate')
                ) == stale_predicate
                and (
                    stale_predicate != canonical_predicate
                    or binding_value(row, 'object') != new_object
                )
            )
        reverse_rows = query_client.query_rows(
            patterns=['?subject ?predicate %s' % subject],
            query_vars=['?subject', '?predicate'],
            models=query_models,
        )
        for stale_predicate in SPATIAL_EFFECT_CLEANUP_PREDICATES:
            removals.extend(
                statement_from_values(
                    binding_value(row, 'subject'),
                    binding_value(row, 'predicate'),
                    subject,
                )
                for row in reverse_rows
                if canonical_spatial_predicate(
                    binding_value(row, 'predicate')
                ) == stale_predicate
                and binding_value(row, 'subject') != new_object
            )
    removals = dedupe_statements(removals)
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
    return removals, result.error_msg or 'KnowledgeCore stale spatial removal failed'


def canonical_spatial_predicate(value: str) -> str:
    """Normalize prefixed and KnowledgeCore-local spatial predicate names."""
    clean = str(value or '').strip()
    if not clean:
        return ''
    local_name = clean.rsplit('#', 1)[-1].rsplit('/', 1)[-1].split(':')[-1]
    return _SPATIAL_PREDICATE_BY_LOCAL_NAME.get(local_name.lower(), clean)


def statement_parts(statement: str) -> tuple[str, str, str]:
    tokens = str(statement or '').strip().split()
    if len(tokens) < 3:
        return '', '', ''
    return tokens[0], tokens[1], ' '.join(tokens[2:])


def statement_from_values(subject: str, predicate: str, obj: str) -> str:
    if not subject or not predicate or not obj:
        return ''
    return '%s %s %s' % (subject, predicate, obj)


def binding_value(row: dict, key: str) -> str:
    if not isinstance(row, dict):
        return ''
    return str(
        row.get(key)
        or row.get('?%s' % key)
        or row.get(key.lstrip('?'))
        or ''
    ).strip()


def dedupe_statements(statements: list[str]) -> list[str]:
    result: list[str] = []
    for statement in statements:
        clean = str(statement or '').strip()
        if clean and clean not in result:
            result.append(clean)
    return result
