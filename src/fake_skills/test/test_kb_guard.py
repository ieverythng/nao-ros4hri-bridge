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
