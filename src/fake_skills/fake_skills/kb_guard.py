"""Skill-scoped KnowledgeCore validation for fake runtime skills."""

from __future__ import annotations

from dataclasses import dataclass

from fake_skills.result_builders import build_skill_result
from fake_skills.result_builders import failure_block

KB_GUARDED_SKILLS = {
    'find_object',
    'look_at',
    'pick_object',
    'place_object',
    'bring_object',
}

_TRUE_VALUES = {'1', 'true', 'yes', 'on', 'enabled'}
_FALSE_VALUES = {'0', 'false', 'no', 'off', 'disabled'}
_REAL_KB_KEYS = (
    'use_real_kb',
    'use_real_KB',
    'use_real_kb_validation',
    'kb_required',
)


@dataclass(frozen=True)
class KbGuardOutcome:
    """Result of a fake-skill KB precondition check."""

    ok: bool
    payload: dict


def real_kb_required(args: dict, *, default_enabled: bool) -> bool:
    """Return whether this request should validate against KnowledgeCore."""
    for key in _REAL_KB_KEYS:
        if key not in args:
            continue
        return _coerce_bool(args.get(key), default=default_enabled)
    return bool(default_enabled)


def validate_skill_target(
    *,
    skill: str,
    args: dict,
    query_rows,
    default_enabled: bool,
    models: list[str],
) -> KbGuardOutcome | None:
    """Validate the target required by a fake skill against KnowledgeCore.

    The guard is intentionally narrow: it queries only the entity required by
    this skill, rather than building or consuming a global KB dump.
    """
    clean_skill = str(skill or '').strip().lower()
    if clean_skill not in KB_GUARDED_SKILLS:
        return None
    if not real_kb_required(args, default_enabled=default_enabled):
        return None

    target = _skill_target(clean_skill, args)
    if not target:
        return KbGuardOutcome(
            ok=False,
            payload=_kb_failure_payload(
                skill=clean_skill,
                target='',
                code='kb_target_missing',
                message='Fake %s requires a grounded KB target.' % clean_skill,
                recoverable=True,
                suggested_recovery='ask_user_to_identify_target',
                rows=[],
            ),
        )

    if not _looks_like_kb_entity(target):
        return KbGuardOutcome(
            ok=False,
            payload=_kb_failure_payload(
                skill=clean_skill,
                target=target,
                code='kb_target_not_canonical',
                message=(
                    'Target "%s" is not a canonical KB entity id; planner args should '
                    'pass the grounded RDF subject instead of a noun phrase.'
                )
                % target,
                recoverable=True,
                suggested_recovery='resolve_target_from_kb',
                rows=[],
            ),
        )

    rows = query_rows(
        patterns=['%s ?predicate ?object' % target],
        query_vars=['?predicate', '?object'],
        models=models,
    )
    if rows:
        return KbGuardOutcome(ok=True, payload={})

    return KbGuardOutcome(
        ok=False,
        payload=_kb_failure_payload(
            skill=clean_skill,
            target=target,
            code='kb_target_unavailable',
            message='KnowledgeCore has no current facts for target "%s".' % target,
            recoverable=True,
            suggested_recovery='scan_or_replan_with_grounded_target',
            rows=[],
        ),
    )


def _skill_target(skill: str, args: dict) -> str:
    if skill == 'look_at':
        keys = ('target', 'object', 'entity', 'target_frame', 'policy')
    else:
        keys = ('target', 'object', 'entity')
    for key in keys:
        value = str(args.get(key, '')).strip()
        if value:
            return value
    return ''


def _looks_like_kb_entity(value: str) -> bool:
    clean = str(value or '').strip()
    if not clean:
        return False
    if any(char.isspace() for char in clean):
        return False
    if any(char in clean for char in ('"', "'", '{', '}', '[', ']')):
        return False
    return True


def _kb_failure_payload(
    *,
    skill: str,
    target: str,
    code: str,
    message: str,
    recoverable: bool,
    suggested_recovery: str,
    rows: list[dict],
) -> dict:
    return build_skill_result(
        skill=skill,
        status='failed',
        target=target,
        target_kind='object',
        target_found=False,
        summary_text='I could not confirm %s in the knowledge base.' % (target or 'the target'),
        evidence={
            'kb_validation': {
                'required': True,
                'query_scope': 'skill_target',
                'rows': list(rows or []),
            }
        },
        failure=failure_block(
            code=code,
            message=message,
            recoverable=recoverable,
            suggested_recovery=suggested_recovery,
        ),
        metadata={
            'fake': True,
            'kb_validation': 'failed',
            'kb_required': True,
        },
    ).to_dict()


def _coerce_bool(value, *, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    clean = str(value).strip().lower()
    if clean in _TRUE_VALUES:
        return True
    if clean in _FALSE_VALUES:
        return False
    return bool(default)
