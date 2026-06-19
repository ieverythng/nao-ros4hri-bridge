from fake_skills.kb_guard import real_kb_required
from fake_skills.kb_guard import validate_skill_target


def test_kb_guard_passes_grounded_subject_with_rows() -> None:
    calls = []

    def _query_rows(**kwargs):
        calls.append(kwargs)
        return [{'?predicate': 'rdf:type', '?object': 'Cup'}]

    outcome = validate_skill_target(
        skill='pick_object',
        args={'target': 'codex_probe_cup'},
        query_rows=_query_rows,
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is True
    assert calls[0]['patterns'] == ['codex_probe_cup ?predicate ?object']


def test_kb_guard_rejects_noun_phrase_target() -> None:
    outcome = validate_skill_target(
        skill='find_object',
        args={'target': 'red cup'},
        query_rows=lambda **_kwargs: [],
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is False
    assert outcome.payload['failure']['code'] == 'kb_target_not_canonical'


def test_kb_guard_rejects_absent_grounded_subject() -> None:
    outcome = validate_skill_target(
        skill='look_at',
        args={'target': 'missing_probe_cup'},
        query_rows=lambda **_kwargs: [],
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is False
    assert outcome.payload['failure']['code'] == 'kb_target_unavailable'


def test_kb_guard_can_be_disabled_per_request() -> None:
    assert real_kb_required({'use_real_KB': False}, default_enabled=True) is False
    outcome = validate_skill_target(
        skill='bring_object',
        args={'target': 'red cup', 'use_real_kb': 'false'},
        query_rows=lambda **_kwargs: [],
        default_enabled=True,
        models=[],
    )

    assert outcome is None


def test_bring_object_validates_grounded_recipient() -> None:
    calls = []

    def _query_rows(**kwargs):
        calls.append(kwargs)
        subject = kwargs['patterns'][0].split()[0]
        if subject in {'codex_probe_cup', 'person_1'}:
            return [{'?predicate': 'rdf:type', '?object': 'Thing'}]
        return []

    outcome = validate_skill_target(
        skill='bring_object',
        args={'target': 'codex_probe_cup', 'recipient_id': 'person_1'},
        query_rows=_query_rows,
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is True
    assert calls[0]['patterns'] == ['codex_probe_cup ?predicate ?object']
    assert calls[1]['patterns'] == ['person_1 ?predicate ?object']


def test_bring_object_accepts_object_id_as_primary_target() -> None:
    calls = []

    def _query_rows(**kwargs):
        calls.append(kwargs)
        subject = kwargs['patterns'][0].split()[0]
        if subject in {'codex_kitchen_cup', 'codex_recipient_person'}:
            return [{'?predicate': 'rdf:type', '?object': 'Thing'}]
        return []

    outcome = validate_skill_target(
        skill='bring_object',
        args={'object_id': 'codex_kitchen_cup', 'recipient': 'codex_recipient_person'},
        query_rows=_query_rows,
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is True
    assert calls[0]['patterns'] == ['codex_kitchen_cup ?predicate ?object']
    assert calls[1]['patterns'] == ['codex_recipient_person ?predicate ?object']


def test_bring_object_rejects_noun_phrase_recipient() -> None:
    outcome = validate_skill_target(
        skill='bring_object',
        args={'target': 'codex_probe_cup', 'recipient': 'the person'},
        query_rows=lambda **_kwargs: [{'?predicate': 'rdf:type', '?object': 'Cup'}],
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is False
    assert outcome.payload['failure']['code'] == 'kb_recipient_not_canonical'


def test_place_object_validates_destination() -> None:
    def _query_rows(**kwargs):
        subject = kwargs['patterns'][0].split()[0]
        if subject in {'codex_probe_cup', 'table_1'}:
            return [{'?predicate': 'rdf:type', '?object': 'Thing'}]
        return []

    outcome = validate_skill_target(
        skill='place_object',
        args={'target': 'codex_probe_cup', 'destination_id': 'table_1'},
        query_rows=_query_rows,
        default_enabled=True,
        models=[],
    )

    assert outcome is not None
    assert outcome.ok is True
