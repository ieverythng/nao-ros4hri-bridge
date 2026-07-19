"""Semantic report-result contract helpers.

The helpers in this module convert execution evidence into a compact contract
for report wording. They do not produce user-facing sentences.
"""

from __future__ import annotations

from planner_common.target_selection import validate_target_selection


_HANDOFF_SKILLS = frozenset({'bring_object', 'deliver_object'})
_PLACEMENT_SKILLS = frozenset({'place_object'})
_DELIVERY_SKILLS = _HANDOFF_SKILLS | _PLACEMENT_SKILLS
_OBJECT_REPORT_SKILLS = _DELIVERY_SKILLS | frozenset({'pick_object'})
_NAVIGATION_SKILLS = frozenset({'navigate_to', 'walk_to'})
_OBSERVATION_SKILLS = frozenset({'look_at', 'scan', 'inspect_scene', 'find_object'})
_SUPPORT_LABELS = frozenset(
    {
        'table',
        'desk',
        'counter',
        'shelf',
        'surface',
        'work_table',
        'kitchen_surface',
    }
)
_LOCATION_KIND_MARKERS = ('location', 'room', 'place', 'area', 'station')
_PERSON_MARKERS = ('person', 'human')


def build_report_outcome(
    *,
    plan_steps: list[dict] | tuple[dict, ...] | None = None,
    execution_results: list[dict] | tuple[dict, ...] | None = None,
    plan_outcome_summary: dict | None = None,
    grounded_context: dict | None = None,
    scene_targets: list[str] | tuple[str, ...] | None = None,
    target_selection: dict | None = None,
    report_role: str = '',
) -> dict:
    """Build a normalized semantic contract for execution-report wording."""
    steps = [dict(item) for item in plan_steps or [] if isinstance(item, dict)]
    results = [dict(item) for item in execution_results or [] if isinstance(item, dict)]
    context = grounded_context if isinstance(grounded_context, dict) else {}
    entity_index = _entity_index(context)
    location_index = _location_index(context)
    step_index = {
        str(step.get('id', '')).strip(): step
        for step in steps
        if str(step.get('id', '')).strip()
    }

    reportable_objects: list[dict] = []
    recipients: list[dict] = []
    anchors: list[dict] = []
    excluded_targets: list[dict] = []
    events: list[dict] = []
    failures: list[dict] = []

    for result in results:
        step_id = str(result.get('id', '')).strip()
        step = _merge_step_record(step_index.get(step_id, {}), result)
        skill = _step_name(step)
        status = str(result.get('status', step.get('status', ''))).strip().lower()
        target = _step_target(step)
        summary = str(result.get('result_summary', '') or step.get('result_summary', '')).strip()
        semantic_failure = ''
        if skill in _PLACEMENT_SKILLS:
            placement_support = _step_recipient_id(step)
            if placement_support and _target_is_person(placement_support, entity_index):
                status = 'failed'
                semantic_failure = 'a person cannot be a placement support'
        events.append(
            {
                'step_id': step_id,
                'skill': skill,
                'status': status,
                'target': target,
                'summary': summary,
            }
        )
        if status == 'failed':
            failures.append(
                {
                    'step_id': step_id,
                    'skill': skill,
                    'target': target,
                    'reason': semantic_failure
                    or str(result.get('reason', '') or step.get('reason', '')).strip(),
                }
            )
            continue
        if status != 'succeeded':
            continue

        if skill in _OBJECT_REPORT_SKILLS:
            object_id = _step_object_id(step)
            if object_id and _target_can_be_report_object(object_id, entity_index, location_index):
                _append_unique(
                    reportable_objects,
                    _entity_contract(
                        object_id,
                        entity_index=entity_index,
                        status='completed',
                    ),
                )
            elif object_id:
                _append_unique(
                    excluded_targets,
                    _excluded_contract(
                        object_id,
                        entity_index=entity_index,
                        location_index=location_index,
                        reason=_excluded_reason(object_id, entity_index, location_index),
                    ),
                )

        if skill in _HANDOFF_SKILLS:
            recipient = _step_recipient_id(step)
            if recipient:
                if _target_is_person(recipient, entity_index):
                    _append_unique(
                        recipients,
                        _entity_contract(
                            recipient,
                            entity_index=entity_index,
                            kind='person',
                        ),
                    )
                    _append_unique(
                        excluded_targets,
                        _excluded_contract(
                            recipient,
                            entity_index=entity_index,
                            location_index=location_index,
                            reason='recipient',
                        ),
                    )
                else:
                    _append_unique(
                        anchors,
                        _anchor_contract(
                            recipient,
                            entity_index=entity_index,
                            location_index=location_index,
                        ),
                    )
                    _append_unique(
                        excluded_targets,
                        _excluded_contract(
                            recipient,
                            entity_index=entity_index,
                            location_index=location_index,
                            reason=_excluded_reason(recipient, entity_index, location_index),
                        ),
                    )

        if skill in _NAVIGATION_SKILLS and target:
            _append_unique(
                excluded_targets,
                _excluded_contract(
                    target,
                    entity_index=entity_index,
                    location_index=location_index,
                    reason='navigation_only',
                ),
            )

    for target in scene_targets or []:
        target_id = str(target or '').strip()
        if not target_id:
            continue
        if _target_is_person(target_id, entity_index):
            _append_unique(
                excluded_targets,
                _excluded_contract(
                    target_id,
                    entity_index=entity_index,
                    location_index=location_index,
                    reason='recipient',
                ),
            )
        elif _target_is_location_or_support(target_id, entity_index, location_index):
            _append_unique(
                anchors,
                _anchor_contract(
                    target_id,
                    entity_index=entity_index,
                    location_index=location_index,
                ),
            )

    selection = target_selection if isinstance(target_selection, dict) else {}
    if str(report_role or '').strip().lower() == 'final' and selection:
        operation = str(selection.get('operation', '')).strip().lower()
        if operation == 'visit':
            completed_ids = {
                str(item.get('target', '')).strip()
                for item in events
                if str(item.get('status', '')).strip().lower() == 'succeeded'
                and str(item.get('skill', '')).strip().lower() in _NAVIGATION_SKILLS
                and str(item.get('target', '')).strip()
            }
        else:
            completed_ids = {
                str(item.get('id', '')).strip()
                for item in reportable_objects
                if str(item.get('id', '')).strip()
            }
        failed_ids = {
            str(item.get('target', '')).strip()
            for item in failures
            if str(item.get('target', '')).strip()
        }
        skill = 'bring_object' if operation == 'deliver' else 'navigate_to'
        for member_id in selection.get('member_ids', []):
            member_id = str(member_id).strip()
            if not member_id or member_id in completed_ids or member_id in failed_ids:
                continue
            failures.append(
                {
                    'step_id': '',
                    'skill': skill,
                    'target': member_id,
                    'reason': 'selected target has no successful execution evidence',
                }
            )

    mode = _report_mode(
        reportable_objects=reportable_objects,
        recipients=recipients,
        anchors=anchors,
        events=events,
        failures=failures,
    )
    return {
        'mode': mode,
        'reportable_objects': reportable_objects,
        'recipients': recipients,
        'anchors': anchors,
        'excluded_targets': excluded_targets,
        'events': events,
        'failures': failures,
    }


def plan_semantic_errors(
    steps,
    grounded_context: dict | None = None,
    target_selection: dict | None = None,
) -> list[str]:
    """Return grounded semantic errors that must block dispatch."""
    context = grounded_context if isinstance(grounded_context, dict) else {}
    entity_index = _entity_index(context)
    location_index = _location_index(context)
    plan_steps = [
        step
        for step in steps if isinstance(steps, (list, tuple))
        if isinstance(step, dict)
    ]
    errors = []
    for step in plan_steps:
        if _step_name(step) not in _PLACEMENT_SKILLS:
            continue
        support = _step_recipient_id(step)
        if support and _target_is_person(support, entity_index):
            errors.append(
                '%s: place_object destination %r is a person, not a placement support'
                % (str(step.get('id', '')).strip() or 'step', support)
            )
    selection = target_selection if isinstance(target_selection, dict) else {}
    if selection:
        selection_validation = validate_target_selection(selection, context)
        errors.extend(selection_validation.errors)
        selection = selection_validation.selection
    operation = str(selection.get('operation', '')).strip().lower()
    member_ids = {
        str(item).strip()
        for item in selection.get('member_ids', [])
        if str(item).strip()
    }
    if operation == 'deliver':
        recipient_id = str(selection.get('recipient_id', '')).strip()
        delivered_ids = set()
        wrong_recipients = set()
        for step in plan_steps:
            if _step_name(step) not in _HANDOFF_SKILLS:
                continue
            step_recipient = _step_recipient_id(step)
            if recipient_id and step_recipient != recipient_id:
                if step_recipient:
                    wrong_recipients.add(step_recipient)
                continue
            object_id = _step_object_id(step)
            if object_id:
                delivered_ids.add(object_id)
        missing_ids = sorted(member_ids - delivered_ids)
        if missing_ids:
            errors.append(
                'delivery plan does not hand off %s to recipient %r'
                % (', '.join(missing_ids), recipient_id)
            )
        extra_ids = sorted(delivered_ids - member_ids)
        if extra_ids:
            errors.append('delivery plan contains unselected objects: %s' % ', '.join(extra_ids))
        if wrong_recipients:
            errors.append(
                'delivery plan changes recipient %r to: %s'
                % (recipient_id, ', '.join(sorted(wrong_recipients)))
            )

    if operation == 'visit':
        visit_targets = {
            _step_target(step)
            for step in plan_steps
            if _step_name(step) in _NAVIGATION_SKILLS
            and _step_target(step)
        }
        if visit_targets != member_ids:
            errors.append(
                'visit targets must exactly match selected members; expected=%s actual=%s'
                % (sorted(member_ids), sorted(visit_targets))
            )

    report_policy = str(selection.get('report_policy', '')).strip().lower()
    report_steps = [
        step
        for step in plan_steps
        if _step_name(step) == 'report_result'
    ]
    if report_policy == 'final' and not report_steps:
        errors.append('final report policy requires a report_result step')
    if report_policy == 'per_target':
        if len(report_steps) != len(member_ids):
            errors.append(
                'per-target report policy requires %d report_result steps, got %d'
                % (len(member_ids), len(report_steps))
            )
        navigation_ids = {
            str(step.get('id', '')).strip()
            for step in plan_steps
            if _step_name(step) in _NAVIGATION_SKILLS
        }
        reported_dependencies = {
            str(dependency).strip()
            for step in report_steps
            for dependency in step.get('requires', [])
            if str(dependency).strip()
        }
        missing_dependencies = sorted(navigation_ids - reported_dependencies)
        if missing_dependencies:
            errors.append(
                'per-target reports do not depend on navigation steps: %s'
                % ', '.join(missing_dependencies)
            )
    return errors


def _merge_step_record(plan_step: dict, result_step: dict) -> dict:
    merged = dict(plan_step or {})
    for key, value in result_step.items():
        if key == 'args' and isinstance(value, dict):
            args = dict(merged.get('args', {}) if isinstance(merged.get('args', {}), dict) else {})
            args.update(value)
            merged['args'] = args
        else:
            merged[key] = value
    return merged


def _step_name(step: dict) -> str:
    return str(step.get('name', '') or step.get('type', '')).strip().lower()


def _step_target(step: dict) -> str:
    for source in _step_sources(step):
        for key in ('target', 'target_frame', 'object_id', 'object', 'location'):
            value = str(source.get(key, '')).strip()
            if value:
                return value
    return ''


def _step_object_id(step: dict) -> str:
    for source in _step_sources(step):
        for key in ('object_id', 'object', 'target_object', 'target'):
            value = str(source.get(key, '')).strip()
            if value:
                return value
    return ''


def _step_recipient_id(step: dict) -> str:
    for source in _step_sources(step):
        for key in ('recipient', 'recipient_id', 'destination', 'destination_id'):
            value = str(source.get(key, '')).strip()
            if value:
                return value
    return ''


def _step_sources(step: dict) -> tuple[dict, ...]:
    sources = []
    for key in ('result_payload', 'args'):
        value = step.get(key, {})
        if isinstance(value, dict):
            sources.append(value)
    sources.append(step)
    return tuple(sources)


def _entity_index(grounded_context: dict) -> dict[str, dict]:
    indexed: dict[str, dict] = {}
    for entity in grounded_context.get('entities', []):
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get('id', '')).strip()
        if entity_id and entity_id not in indexed:
            indexed[entity_id] = entity
    return indexed


def _location_index(grounded_context: dict) -> dict[str, dict]:
    indexed: dict[str, dict] = {}
    for location in grounded_context.get('locations', []):
        if not isinstance(location, dict):
            continue
        location_id = str(location.get('id', '')).strip()
        if location_id and location_id not in indexed:
            indexed[location_id] = location
    return indexed


def _entity_contract(
    entity_id: str,
    *,
    entity_index: dict[str, dict],
    kind: str = '',
    status: str = '',
) -> dict:
    entity = entity_index.get(entity_id, {})
    contract = {
        'id': entity_id,
        'label': _entity_label(entity_id, entity),
        'class': str(entity.get('class', '')).strip(),
    }
    if kind:
        contract['kind'] = kind
    elif str(entity.get('kind', '')).strip():
        contract['kind'] = str(entity.get('kind', '')).strip()
    if status:
        contract['status'] = status
    return contract


def _anchor_contract(
    target_id: str,
    *,
    entity_index: dict[str, dict],
    location_index: dict[str, dict],
) -> dict:
    location = location_index.get(target_id, {})
    entity = entity_index.get(target_id, {})
    role = 'location' if target_id in location_index else 'support'
    return {
        'id': target_id,
        'label': _entity_label(target_id, location or entity),
        'role': role,
    }


def _excluded_contract(
    target_id: str,
    *,
    entity_index: dict[str, dict],
    location_index: dict[str, dict],
    reason: str,
) -> dict:
    source = entity_index.get(target_id, location_index.get(target_id, {}))
    return {
        'id': target_id,
        'label': _entity_label(target_id, source),
        'reason': reason,
    }


def _entity_label(entity_id: str, entity: dict) -> str:
    label = str(entity.get('label', '')).strip() if isinstance(entity, dict) else ''
    if label:
        return label
    return str(entity_id or '').strip()


def _target_can_be_report_object(
    target_id: str,
    entity_index: dict[str, dict],
    location_index: dict[str, dict],
) -> bool:
    return bool(target_id) and not _target_is_person(target_id, entity_index) and not _target_is_location_or_support(
        target_id,
        entity_index,
        location_index,
    )


def _target_is_person(target_id: str, entity_index: dict[str, dict]) -> bool:
    lowered = str(target_id or '').strip().lower()
    entity = entity_index.get(target_id, {})
    kind = str(entity.get('kind', '')).strip().lower()
    entity_class = str(entity.get('class', '')).strip().lower()
    return (
        any(marker in lowered for marker in _PERSON_MARKERS)
        or kind in {'person', 'human'}
        or any(marker in entity_class for marker in _PERSON_MARKERS)
    )


def _target_is_location_or_support(
    target_id: str,
    entity_index: dict[str, dict],
    location_index: dict[str, dict],
) -> bool:
    lowered = str(target_id or '').strip().lower()
    if target_id in location_index:
        return True
    entity = entity_index.get(target_id, {})
    label = str(entity.get('label', '')).strip().lower()
    entity_class = str(entity.get('class', '')).strip().lower()
    kind = str(entity.get('kind', '')).strip().lower()
    candidates = {lowered, label, entity_class, kind}
    if candidates.intersection(_SUPPORT_LABELS):
        return True
    return any(marker in item for item in candidates for marker in _LOCATION_KIND_MARKERS)


def _excluded_reason(
    target_id: str,
    entity_index: dict[str, dict],
    location_index: dict[str, dict],
) -> str:
    if _target_is_person(target_id, entity_index):
        return 'recipient'
    if target_id in location_index:
        return 'room'
    return 'support'


def _append_unique(items: list[dict], item: dict) -> None:
    item_id = str(item.get('id', '')).strip()
    if not item_id:
        return
    if any(str(existing.get('id', '')).strip() == item_id for existing in items):
        return
    items.append(item)


def _report_mode(
    *,
    reportable_objects: list[dict],
    recipients: list[dict],
    anchors: list[dict],
    events: list[dict],
    failures: list[dict],
) -> str:
    if failures and not reportable_objects:
        return 'failure'
    if failures and reportable_objects:
        return 'mixed'
    successful_skills = {
        str(event.get('skill', '')).strip().lower()
        for event in events
        if str(event.get('status', '')).strip().lower() == 'succeeded'
    }
    if reportable_objects and (
        recipients
        or (anchors and successful_skills.intersection(_DELIVERY_SKILLS))
    ):
        return 'delivery'
    if successful_skills and successful_skills.issubset(_NAVIGATION_SKILLS | {'report_result'}):
        return 'ordered_navigation'
    if successful_skills and successful_skills.intersection(_OBSERVATION_SKILLS):
        return 'observation'
    if reportable_objects or anchors:
        return 'mixed'
    return 'failure' if failures else 'mixed'
