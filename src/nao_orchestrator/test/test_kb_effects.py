from nao_orchestrator.kb_effects import remove_stale_spatial_effect_values
from nao_orchestrator.kb_effects import validate_spatial_effect_statements


class _Query:
    def __init__(self, facts: list[str]) -> None:
        self.facts = facts

    def query_rows(self, *, patterns, query_vars, models):
        del models
        subject, predicate, obj = patterns[0].split()
        rows = []
        for fact in self.facts:
            fact_subject, fact_predicate, fact_object = fact.split(maxsplit=2)
            if subject != fact_subject or predicate != fact_predicate:
                continue
            if obj != '?object' and obj != fact_object:
                continue
            rows.append({'?object': fact_object} if '?object' in query_vars else {})
        return rows


def test_validate_spatial_effect_accepts_explicit_typed_place() -> None:
    error = validate_spatial_effect_statements(
        statements=['cup_1 oro:isAt book_1'],
        models=['default'],
        query_client=_Query(
            [
                'book_1 rdf:type Room',
            ]
        ),
    )

    assert error == ''


def test_validate_spatial_effect_rejects_object_destination_without_rewriting() -> None:
    error = validate_spatial_effect_statements(
        statements=['cup_1 oro:isAt book_1'],
        models=['default'],
        query_client=_Query(
            [
                'book_1 rdf:type Book',
                'book_1 oro:isAt kitchen_1',
                'kitchen_1 rdf:type Room',
            ]
        ),
    )

    assert error == 'spatial effect destination is not typed for oro:isAt: book_1'


def test_validate_spatial_effect_does_not_infer_type_from_identifier() -> None:
    error = validate_spatial_effect_statements(
        statements=['cup_1 oro:isAt Kitchen'],
        models=['default'],
        query_client=_Query([]),
    )

    assert error == 'spatial effect destination is not typed for oro:isAt: Kitchen'


class _LocalNameQuery:
    def query_rows(self, *, patterns, query_vars, models):
        del query_vars, models
        if patterns == ['cup_1 ?predicate ?object']:
            return [
                {'predicate': 'isAt', 'object': 'lab_table'},
                {'predicate': 'isOn', 'object': 'lab_table'},
                {'predicate': 'placeOf', 'object': 'lab_table'},
            ]
        if patterns == ['?subject ?predicate cup_1']:
            return [{'subject': 'lab_table', 'predicate': 'contains'}]
        return []


class _Mutation:
    def __init__(self) -> None:
        self.statements = []

    def mutate(self, *, operation, statements, models, wait_for_result):
        del models, wait_for_result
        assert operation == 'remove'
        self.statements = list(statements)
        return type('Result', (), {'success': True, 'error_msg': ''})()


def test_stale_spatial_cleanup_accepts_knowledgecore_local_predicate_names() -> None:
    mutation = _Mutation()

    removals, error = remove_stale_spatial_effect_values(
        statements=['cup_1 oro:isAt person_1'],
        models=['default'],
        query_client=_LocalNameQuery(),
        mutation_client=mutation,
    )

    assert error == ''
    assert removals == [
        'cup_1 isOn lab_table',
        'cup_1 isAt lab_table',
        'cup_1 placeOf lab_table',
        'lab_table contains cup_1',
    ]
    assert mutation.statements == removals
