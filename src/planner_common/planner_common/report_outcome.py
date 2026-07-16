"""Semantic report-result contract helpers.

The helpers in this module convert execution evidence into a compact contract
for report wording. They do not produce user-facing sentences.
"""

from __future__ import annotations


_DELIVERY_SKILLS = frozenset({'bring_object', 'deliver_object', 'place_object'})
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


def plan_semantic_errors(
    plan_steps: list[dict] | tuple[dict, ...] | None,
    grounded_context: dict | None,
    target_selection: dict | None,
) -> list[str]:
    """Validate selected members against grounding and executable plan targets."""
    selection = target_selection if isinstance(target_selection, dict) else {}
    member_ids = _unique_strings(selection.get('member_ids', []))
    if not selection or not member_ids:
        return []

    context = grounded_context if isinstance(grounded_context, dict) else {}
    entities = _entity_index(context)
    locations = _location_index(context)
    errors: list[str] = []

    invalid_members = [
        member_id
        for member_id in member_ids
        if member_id not in entities
        or _target_is_person(member_id, entities)
        or _target_is_location_or_support(member_id, entities, locations)
    ]
    if invalid_members:
        errors.append(
            'target_selection members must be grounded objects: %s'
            % ', '.join(invalid_members)
        )

    operation = str(selection.get('operation', '')).strip().lower()
    steps = [dict(step) for step in plan_steps or [] if isinstance(step, dict)]
    if operation == 'visit':
        visited = _ordered_step_targets(
            steps,
            skills={'navigate_to', 'walk_to', 'move_to_location'},
            keys=('target', 'target_frame', 'object_id', 'object', 'location'),
        )
        unexpected = [target for target in visited if target not in member_ids]
        missing = [member_id for member_id in member_ids if member_id not in visited]
        if unexpected or missing:
            details = []
            if missing:
                details.append('missing %s' % ', '.join(missing))
            if unexpected:
                details.append('unexpected %s' % ', '.join(unexpected))
            errors.append('visit targets do not match target_selection (%s)' % '; '.join(details))

    if operation == 'deliver':
        deliveries = _delivery_step_targets(steps)
        delivered_members = [object_id for object_id, _ in deliveries]
        missing = [member_id for member_id in member_ids if member_id not in delivered_members]
        unexpected = [object_id for object_id in delivered_members if object_id not in member_ids]
        if missing or unexpected:
            details = []
            if missing:
                details.append('missing %s' % ', '.join(missing))
            if unexpected:
                details.append('unexpected %s' % ', '.join(unexpected))
            errors.append('delivery targets do not match target_selection (%s)' % '; '.join(details))

        recipient_id = str(selection.get('recipient_id', '')).strip()
        if recipient_id:
            if recipient_id not in entities or not _target_is_person(recipient_id, entities):
                errors.append('delivery recipient is not a grounded person: %s' % recipient_id)
            mismatched = [
                recipient
                for _, recipient in deliveries
                if recipient and recipient != recipient_id
            ]
            if mismatched:
                errors.append(
                    'delivery recipient does not match target_selection: %s'
                    % ', '.join(_unique_strings(mismatched))
                )
    return errors


def _unique_strings(values) -> list[str]:
    if not isinstance(values, (list, tuple, set)):
        return []
    result = []
    seen = set()
    for value in values:
        normalized = str(value or '').strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        result.append(normalized)
    return result


def _ordered_step_targets(
    steps: list[dict],
    *,
    skills: set[str],
    keys: tuple[str, ...],
) -> list[str]:
    targets = []
    for step in steps:
        if _step_name(step) not in skills:
            continue
        for source in _step_sources(step):
            target = next(
                (str(source.get(key, '')).strip() for key in keys if str(source.get(key, '')).strip()),
                '',
            )
            if target:
                targets.append(target)
                break
    return targets


def _delivery_step_targets(steps: list[dict]) -> list[tuple[str, str]]:
    deliveries = []
    for step in steps:
        if _step_name(step) not in _DELIVERY_SKILLS:
            continue
        deliveries.append((_step_object_id(step), _step_recipient_id(step)))
    return [(object_id, recipient) for object_id, recipient in deliveries if object_id]


def build_report_outcome(
    *,
    plan_steps: list[dict] | tuple[dict, ...] | None = None,
    execution_results: list[dict] | tuple[dict, ...] | None = None,
    plan_outcome_summary: dict | None = None,
    grounded_context: dict | None = None,
    scene_targets: list[str] | tuple[str, ...] | None = None,
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
                    'reason': str(result.get('reason', '') or step.get('reason', '')).strip(),
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

        if skill in _DELIVERY_SKILLS:
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
