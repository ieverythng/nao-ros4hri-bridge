from kb_skills.mutation_client import MutationResult

from nao_orchestrator.orchestrator import NaoOrchestrator
from nao_orchestrator.orchestrator import _RuntimeStats
from nao_orchestrator.orchestrator import _group_kb_effect_statements
from nao_orchestrator.orchestrator import _kb_effects_from_result_payload


class FakeKnowledgeQuery:
    def __init__(self, facts: list[str]) -> None:
        self.facts = list(facts)
        self.calls: list[dict] = []

    def query_rows(self, *, patterns, query_vars, models):
        self.calls.append(
            {
                'patterns': list(patterns),
                'query_vars': list(query_vars),
                'models': list(models),
            }
        )
        pattern = patterns[0]
        tokens = pattern.split()
        if len(tokens) < 3:
            return []
        subject, predicate, obj = tokens[0], tokens[1], tokens[2]
        rows = []
        for fact in self.facts:
            fact_subject, fact_predicate, fact_object = fact.split(maxsplit=2)
            if subject != fact_subject:
                continue
            if predicate not in {'?predicate', fact_predicate}:
                continue
            if obj not in {'?object', fact_object}:
                continue
            row = {}
            if '?predicate' in query_vars:
                row['?predicate'] = fact_predicate
            if '?object' in query_vars:
                row['?object'] = fact_object
            rows.append(row)
        return rows


class FakeKnowledgeMutation:
    def __init__(self, query: FakeKnowledgeQuery, *, apply_remove: bool = True) -> None:
        self.query = query
        self.apply_remove = apply_remove
        self.calls: list[dict] = []

    def mutate(
        self,
        *,
        operation,
        statements,
        models,
        lifespan_sec=0.0,
        wait_for_result=True,
    ):
        clean_statements = list(statements)
        self.calls.append(
            {
                'operation': operation,
                'statements': clean_statements,
                'models': list(models),
                'lifespan_sec': lifespan_sec,
                'wait_for_result': wait_for_result,
            }
        )
        if operation == 'remove' and self.apply_remove:
            self.query.facts = [
                fact for fact in self.query.facts if fact not in clean_statements
            ]
        elif operation in {'add', 'update'}:
            for statement in clean_statements:
                if statement not in self.query.facts:
                    self.query.facts.append(statement)
        return MutationResult(
            success=True,
            operation=operation,
            dispatched=True,
            statement_count=len(clean_statements),
        )


class FakeLogger:
    def info(self, *_args, **_kwargs) -> None:
        pass

    def warn(self, *_args, **_kwargs) -> None:
        pass


def _orchestrator_with_kb(
    facts: list[str],
    *,
    apply_remove: bool = True,
) -> tuple[NaoOrchestrator, FakeKnowledgeQuery, FakeKnowledgeMutation]:
    node = NaoOrchestrator.__new__(NaoOrchestrator)
    query = FakeKnowledgeQuery(facts)
    mutation = FakeKnowledgeMutation(query, apply_remove=apply_remove)
    node._kb_query_client = query
    node._kb_mutation_client = mutation
    node._stats = _RuntimeStats()
    node.apply_success_kb_effects = True
    node._logger = FakeLogger()
    return node, query, mutation


def test_kb_remove_expands_subject_only_request_and_verifies_empty_postcondition():
    node, query, mutation = _orchestrator_with_kb(
        [
            'codex_marker rdf:type Cup',
            'codex_marker dbp:color blue',
            'other_marker dbp:color red',
        ]
    )

    success, reason, payload = node._execute_kb_mutation_step(
        'kb_remove',
        {'statements': ['codex_marker']},
    )

    assert success is True
    assert reason == 'KnowledgeCore mutation completed'
    assert mutation.calls[0]['operation'] == 'remove'
    assert mutation.calls[0]['statements'] == [
        'codex_marker rdf:type Cup',
        'codex_marker dbp:color blue',
    ]
    assert payload['statement_count'] == 2
    assert query.facts == ['other_marker dbp:color red']
    assert node._stats.dispatched_kb_mutation == 1
    assert node._stats.dispatch_failures == 0


def test_success_kb_effect_helpers_group_valid_statements():
    payload = {
        'evidence': {
            'kb_effects': [
                {'action': 'remove', 'statement': 'cup_1 oro:isOn table_1'},
                {'action': 'add', 'statement': 'robot oro:holds cup_1'},
                {'action': 'revise', 'statements': ['cup_1 dbp:color blue']},
                {'action': 'ignored', 'statement': 'cup_1 noise value'},
                {'action': 'add', 'statement': 'robot oro:holds cup_1'},
            ]
        }
    }

    effects = _kb_effects_from_result_payload(payload)

    assert _group_kb_effect_statements(effects) == {
        'remove': ['cup_1 oro:isOn table_1'],
        'add': ['robot oro:holds cup_1'],
        'update': ['cup_1 dbp:color blue'],
    }


def test_successful_skill_kb_effects_are_applied_through_kb_boundary():
    node, _query, mutation = _orchestrator_with_kb(['cup_1 oro:isOn table_1'])

    summary = node._apply_success_kb_effects(
        {
            'skill': 'pick_object',
            'status': 'succeeded',
            'evidence': {
                'kb_effects': [
                    {'action': 'remove', 'statement': 'cup_1 oro:isOn table_1'},
                    {'action': 'add', 'statement': 'robot oro:holds cup_1'},
                ]
            },
        }
    )

    assert summary['applied'] is True
    assert summary['verified'] is True
    assert mutation.calls == [
        {
            'operation': 'remove',
            'statements': ['cup_1 oro:isOn table_1'],
            'models': ['default'],
            'lifespan_sec': 0.0,
            'wait_for_result': True,
        },
        {
            'operation': 'add',
            'statements': ['robot oro:holds cup_1'],
            'models': ['default'],
            'lifespan_sec': 0.0,
            'wait_for_result': True,
        },
    ]
    assert _query.facts == ['robot oro:holds cup_1']
    assert node._stats.dispatched_kb_mutation == 2


def test_successful_skill_kb_effects_fail_when_remove_postcondition_remains():
    node, _query, mutation = _orchestrator_with_kb(
        ['cup_1 oro:isOn table_1'],
        apply_remove=False,
    )

    summary = node._apply_success_kb_effects(
        {
            'skill': 'pick_object',
            'status': 'succeeded',
            'evidence': {
                'kb_effects': [
                    {'action': 'remove', 'statement': 'cup_1 oro:isOn table_1'},
                    {'action': 'add', 'statement': 'robot oro:holds cup_1'},
                ]
            },
        }
    )

    assert summary['applied'] is False
    assert summary['verified'] is True
    assert summary['remaining_statements'] == ['cup_1 oro:isOn table_1']
    assert mutation.calls[0]['operation'] == 'remove'
    assert node._stats.dispatched_kb_mutation == 2
    assert node._stats.dispatch_failures == 1


def test_successful_skill_kb_effects_can_be_disabled():
    node, _query, mutation = _orchestrator_with_kb(['cup_1 oro:isOn table_1'])
    node.apply_success_kb_effects = False

    summary = node._apply_success_kb_effects(
        {
            'evidence': {
                'kb_effects': [{'action': 'remove', 'statement': 'cup_1 oro:isOn table_1'}]
            }
        }
    )

    assert summary == {}
    assert mutation.calls == []


def test_kb_remove_expands_generic_type_statement_as_subject_remove():
    node, _query, mutation = _orchestrator_with_kb(
        [
            'codex_marker rdf:type Cube',
            'codex_marker rdf:type owl:Thing',
            'codex_marker dbp:name NOVA',
            'codex_marker dbp:color blue',
            'other_marker dbp:color red',
        ]
    )

    success, reason, payload = node._execute_kb_mutation_step(
        'kb_remove',
        {'statements': ['codex_marker rdf:type owl:Thing']},
    )

    assert success is True
    assert reason == 'KnowledgeCore mutation completed'
    assert mutation.calls[0]['statements'] == [
        'codex_marker rdf:type Cube',
        'codex_marker rdf:type owl:Thing',
        'codex_marker dbp:name NOVA',
        'codex_marker dbp:color blue',
    ]
    assert payload['statement_count'] == 4
    assert _query.facts == ['other_marker dbp:color red']


def test_kb_remove_expands_natural_language_subject_alias():
    node, _query, mutation = _orchestrator_with_kb(
        [
            'codex_marker rdf:type Cube',
            'codex_marker dbp:name NOVA',
            'codex_marker dbp:color blue',
        ]
    )

    success, reason, payload = node._execute_kb_mutation_step(
        'kb_remove',
        {'statements': ['codex_marker is in knowledge base']},
    )

    assert success is True
    assert reason == 'KnowledgeCore mutation completed'
    assert mutation.calls[0]['statements'] == [
        'codex_marker rdf:type Cube',
        'codex_marker dbp:name NOVA',
        'codex_marker dbp:color blue',
    ]
    assert payload['statement_count'] == 3
    assert _query.facts == []


def test_kb_remove_rejects_subject_only_request_when_no_facts_match():
    node, _query, mutation = _orchestrator_with_kb(['other_marker dbp:color red'])

    success, reason, payload = node._execute_kb_mutation_step(
        'kb_remove',
        {'statements': ['missing_marker']},
    )

    assert success is False
    assert reason == 'KnowledgeCore remove found no matching facts'
    assert payload == {
        'skill': 'kb_remove',
        'operation': 'remove',
        'statement_count': 0,
        'dispatched': False,
        'success': False,
    }
    assert mutation.calls == []
    assert node._stats.dispatched_kb_mutation == 0
    assert node._stats.dispatch_failures == 1


def test_kb_remove_fails_when_postcondition_still_returns_removed_fact():
    node, _query, mutation = _orchestrator_with_kb(
        ['codex_marker dbp:color blue'],
        apply_remove=False,
    )

    success, reason, payload = node._execute_kb_mutation_step(
        'kb_remove',
        {'statements': ['codex_marker dbp:color blue']},
    )

    assert success is False
    assert reason == 'KnowledgeCore remove post-condition failed'
    assert mutation.calls[0]['statements'] == ['codex_marker dbp:color blue']
    assert payload['dispatched'] is True
    assert payload['success'] is False
    assert payload['remaining_statements'] == ['codex_marker dbp:color blue']
    assert node._stats.dispatched_kb_mutation == 0
    assert node._stats.dispatch_failures == 1
