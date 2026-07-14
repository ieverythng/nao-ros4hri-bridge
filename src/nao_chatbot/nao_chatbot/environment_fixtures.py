"""Pure validation helpers for semantic environment fixtures."""

from __future__ import annotations


_LOCATION_PREDICATES = frozenset({'oro:isAt', 'oro:isIn'})
_PERSON_TYPES = frozenset({'Human', 'Person', 'foaf:Person'})


def validate_environment_fixture(_fixture_id: str, fixture: dict) -> list[str]:
    """Return semantic contract errors for one preloaded environment."""
    statements = _parse_statements(fixture.get('statements', []))
    errors: list[str] = []

    robot_locations = {
        obj
        for subject, predicate, obj in statements
        if subject in {'myself', 'nao_robot'} and predicate in _LOCATION_PREDICATES
    }
    if not robot_locations:
        errors.append('fixture has no explicit robot location')

    person_ids = {
        subject
        for subject, predicate, obj in statements
        if predicate == 'rdf:type' and obj in _PERSON_TYPES
    }
    for person_id in sorted(person_ids):
        locations = {
            obj
            for subject, predicate, obj in statements
            if subject == person_id and predicate in _LOCATION_PREDICATES
        }
        if not locations:
            errors.append('person %s has no explicit location' % person_id)
            continue
        if len(locations) > 1:
            errors.append('person %s has multiple explicit locations' % person_id)
        for location_id in sorted(locations):
            if (location_id, 'oro:contains', person_id) not in statements:
                errors.append(
                    'person %s location %s lacks reciprocal contains'
                    % (person_id, location_id)
                )

    return errors


def _parse_statements(values) -> set[tuple[str, str, str]]:
    parsed: set[tuple[str, str, str]] = set()
    if not isinstance(values, list):
        return parsed
    for value in values:
        parts = str(value or '').strip().split(maxsplit=2)
        if len(parts) == 3:
            parsed.add((parts[0], parts[1], parts[2]))
    return parsed
