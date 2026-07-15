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
