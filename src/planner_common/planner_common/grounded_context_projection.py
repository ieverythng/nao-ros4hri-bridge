"""Compact LLM-facing world-view projection from planner grounding seams.

Extracted from contracts.py to keep the public contract surface lean.
All helpers here are private to this module and not part of the public API.
"""

from __future__ import annotations

from planner_common.contracts import (
    _COMPACT_GROUNDED_CONTEXT_KEYS,
    _LLM_RELATION_PREDICATE_ALIASES,
    _LLM_RELATION_PREDICATE_PRIORITY,
    _MAX_LLM_RELATIONS_PER_ENTITY,
    _first_non_empty,
    coerce_bool,
    _coerce_nonnegative_int,
    _coerce_float,
)


# ---------------------------------------------------------------------------
# Entity normalization
# ---------------------------------------------------------------------------

def _normalize_grounded_entity(
    item: dict,
    *,
    include_raw_relations: bool = False,
) -> dict:
    entity_id = str(item.get('id', item.get('entity_id', ''))).strip()
    label_value = item.get('label', None)
    label = None if label_value is None else str(label_value).strip()
    kind = _normalized_kind(item.get('kind', ''), item.get('class', item.get('type', '')))
    entity = {
        'id': entity_id,
        'label': label or None,
        'kind': kind,
        'class': str(item.get('class', item.get('type', ''))).strip(),
        'visible': coerce_bool(item.get('visible', True)),
        'relations': _normalize_relations(item.get('relations', [])),
    }
    raw_relations = item.get('raw_relations', [])
    if include_raw_relations and isinstance(raw_relations, list) and raw_relations:
        entity['raw_relations'] = _normalize_raw_relations(raw_relations)
    state_t0 = item.get('state_t0', None)
    if isinstance(state_t0, dict) and state_t0:
        entity['state_t0'] = dict(state_t0)
    return {
        key: value
        for key, value in entity.items()
        if key == 'label' or value not in ('', [], {})
    }


def _normalize_grounded_counts(counts, entities: list) -> dict:
    if isinstance(counts, dict):
        return {
            'entities': _coerce_nonnegative_int(counts.get('entities', len(entities))),
            'objects': _coerce_nonnegative_int(counts.get('objects', 0)),
            'people': _coerce_nonnegative_int(counts.get('people', 0)),
        }
    return _counts_from_entities(entities)


# ---------------------------------------------------------------------------
# Relation normalization
# ---------------------------------------------------------------------------

def _normalize_relations(value, *, max_relations: int = _MAX_LLM_RELATIONS_PER_ENTITY) -> list[dict]:
    if not isinstance(value, list):
        return []
    relations_by_key = {}
    seen = set()
    seen_rdf_type = False
    for item in value:
        if not isinstance(item, dict):
            continue
        predicate = _normalize_relation_predicate(
            item.get('predicate', item.get('p', ''))
        )
        obj = str(item.get('object', item.get('o', ''))).strip()
        if not predicate or not obj:
            continue
        if predicate not in _LLM_RELATION_PREDICATE_PRIORITY:
            continue
        if predicate == 'rdf:type':
            if seen_rdf_type:
                continue
            seen_rdf_type = True
        key = (predicate, obj)
        if key in seen:
            continue
        seen.add(key)
        relations_by_key[key] = {'predicate': predicate, 'object': obj}
    relations = list(relations_by_key.values())
    relations.sort(
        key=lambda item: (
            _relation_priority(item.get('predicate', '')),
            item.get('object', ''),
        )
    )
    return relations[:max(0, int(max_relations))]


def _normalize_raw_relations(value) -> list[dict]:
    if not isinstance(value, list):
        return []
    relations = []
    seen = set()
    for item in value:
        if not isinstance(item, dict):
            continue
        predicate = str(item.get('predicate', item.get('p', ''))).strip()
        obj = str(item.get('object', item.get('o', ''))).strip()
        if not predicate or not obj:
            continue
        key = (predicate, obj)
        if key in seen:
            continue
        seen.add(key)
        relations.append({'predicate': predicate, 'object': obj})
    return relations


def _normalize_relation_predicate(value) -> str:
    text = _compact_term(value)
    if not text:
        return ''
    if text in _LLM_RELATION_PREDICATE_PRIORITY:
        return text
    lower = text.strip().lower()
    return _LLM_RELATION_PREDICATE_ALIASES.get(lower, text)


def _relation_priority(predicate: str) -> int:
    try:
        return _LLM_RELATION_PREDICATE_PRIORITY.index(str(predicate or '').strip())
    except ValueError:
        return len(_LLM_RELATION_PREDICATE_PRIORITY)


def _compact_term(value) -> str:
    text = str(value or '').strip()
    if not text:
        return ''
    if text.startswith('dbr:'):
        return text.split(':', 1)[1]
    for separator in ('#', '/'):
        if separator in text:
            text = text.rsplit(separator, 1)[-1]
    return text


# ---------------------------------------------------------------------------
# Scene / state / KB iterators
# ---------------------------------------------------------------------------

def _scene_items(scene_summary: dict, key: str) -> list[dict]:
    if not isinstance(scene_summary, dict):
        return []
    items = scene_summary.get(key, [])
    if not isinstance(items, list):
        return []
    return [item for item in items if isinstance(item, dict)]


def _state_entities(state_t0: dict) -> list[dict]:
    if not isinstance(state_t0, dict):
        return []
    entities = state_t0.get('entities', [])
    if isinstance(entities, list) and entities:
        return [item for item in entities if isinstance(item, dict)]
    result = []
    for key in ('objects', 'people'):
        items = state_t0.get(key, [])
        if isinstance(items, list):
            result.extend(item for item in items if isinstance(item, dict))
    return result


def _knowledge_references(knowledge_snapshot: dict) -> list[dict]:
    if not isinstance(knowledge_snapshot, dict):
        return []
    references = knowledge_snapshot.get('references', [])
    if isinstance(references, list):
        return [item for item in references if isinstance(item, dict)]
    return []


# ---------------------------------------------------------------------------
# Compact entity builders
# ---------------------------------------------------------------------------

def _ensure_compact_entity(
    entities_by_id: dict[str, dict],
    entity_id: str,
    *,
    label: str | None,
    kind: str,
    entity_class,
) -> dict:
    clean_id = str(entity_id or '').strip()
    entity = entities_by_id.setdefault(
        clean_id,
        {
            'id': clean_id,
            'label': label,
            'kind': kind or 'object',
            'class': str(entity_class or '').strip(),
            'visible': True,
            'relations': [],
        },
    )
    if not entity.get('label') and label:
        entity['label'] = label
    if not entity.get('class') and str(entity_class or '').strip():
        entity['class'] = str(entity_class or '').strip()
    if entity.get('kind') == 'object' and kind == 'person':
        entity['kind'] = 'person'
    return entity


def _display_entity_label(value, entity_id: str) -> str:
    raw = str(value or '').strip()
    if not raw:
        raw = str(entity_id or '').strip()
    if not raw:
        return ''
    parts = raw.split('_')
    if len(parts) > 1 and _looks_generated_suffix(parts[-1]):
        return '_'.join(parts[:-1])
    return raw


def _person_label(item: dict, entity_id: str) -> str | None:
    label = str(item.get('label', '')).strip()
    if label and label != entity_id:
        return _display_entity_label(label, entity_id)
    if str(entity_id).startswith('anonymous_'):
        return None
    return _display_entity_label(label, entity_id)


def _looks_generated_suffix(value: str) -> bool:
    clean = str(value or '').strip()
    return len(clean) >= 4 and clean.isalnum() and not clean.isdigit()


def _normalized_kind(kind_value, type_value) -> str:
    kind = str(kind_value or '').strip().lower()
    if kind in ('person', 'human'):
        return 'person'
    if kind == 'object':
        return 'object'
    type_text = str(type_value or '').strip().lower()
    if any(token in type_text for token in ('person', 'human', 'face', 'speaker')):
        return 'person'
    return 'object'


def _copy_optional_planner_details(entity: dict, source: dict, keys: tuple[str, ...]) -> None:
    for key in keys:
        if key in source and source.get(key) not in (None, ''):
            entity[key] = source.get(key)


def _add_relation(entity: dict, predicate, obj) -> None:
    clean_predicate = _normalize_relation_predicate(predicate)
    clean_object = _compact_term(obj)
    if not clean_predicate or not clean_object:
        return
    relations = entity.setdefault('relations', [])
    candidate = {'predicate': clean_predicate, 'object': clean_object}
    if candidate not in relations:
        relations.append(candidate)


def _merge_knowledge_row_relation(
    entities_by_id: dict[str, dict],
    row: dict,
    *,
    include_raw_relation: bool = False,
) -> None:
    entity_id = _first_non_empty(row.get('entity', ''), row.get('s', ''))
    if not entity_id:
        return
    entity_id = _compact_term(entity_id)
    predicate = _first_non_empty(
        row.get('predicate', ''),
        row.get('p', ''),
        row.get('attribute', ''),
    )
    obj = _first_non_empty(
        row.get('object', ''),
        row.get('o', ''),
        row.get('value', ''),
        row.get('type', ''),
    )
    if row.get('type', '') and not predicate:
        predicate = 'rdf:type'
    if not predicate or not obj:
        return
    entity = _ensure_compact_entity(
        entities_by_id,
        entity_id,
        label=_display_entity_label(entity_id, entity_id),
        kind=_normalized_kind('', obj if predicate in ('rdf:type', 'type') else ''),
        entity_class=obj if predicate in ('rdf:type', 'type') else '',
    )
    normalized_predicate = _normalize_relation_predicate(predicate)
    if normalized_predicate == 'rdf:type' and not entity.get('class'):
        entity['class'] = _compact_term(obj)
    _add_relation(entity, normalized_predicate, obj)
    if include_raw_relation:
        raw_relations = entity.setdefault('raw_relations', [])
        raw_candidate = {'predicate': str(predicate).strip(), 'object': str(obj).strip()}
        if raw_candidate not in raw_relations:
            raw_relations.append(raw_candidate)


# ---------------------------------------------------------------------------
# Finalization & counts
# ---------------------------------------------------------------------------

def _finalize_compact_entity(entity: dict) -> dict:
    finalized = {
        'id': str(entity.get('id', '')).strip(),
        'label': entity.get('label') if entity.get('label') else None,
        'kind': str(entity.get('kind', 'object')).strip() or 'object',
        'class': str(entity.get('class', '')).strip(),
        'visible': coerce_bool(entity.get('visible', True)),
        'relations': _normalize_relations(entity.get('relations', [])),
    }
    raw_relations = _normalize_raw_relations(entity.get('raw_relations', []))
    if raw_relations:
        finalized['raw_relations'] = raw_relations
    for key in ('center_x', 'center_y', 'last_seen_sec', 'last_seen_age_sec'):
        if key in entity:
            finalized[key] = entity[key]
    return {
        key: value
        for key, value in finalized.items()
        if key == 'label' or value not in ('', [], {})
    }


def _counts_from_entities(entities: list) -> dict:
    clean_entities = [item for item in entities if isinstance(item, dict)]
    people = sum(
        1 for item in clean_entities if str(item.get('kind', '')).strip() == 'person'
    )
    objects = sum(
        1 for item in clean_entities if str(item.get('kind', '')).strip() == 'object'
    )
    return {
        'entities': len(clean_entities),
        'objects': objects,
        'people': people,
    }


# ---------------------------------------------------------------------------
# Public API (re-exported from contracts.py)
# ---------------------------------------------------------------------------

def project_llm_grounded_context(
    grounded_context: dict,
    *,
    knowledge_rows: list[dict] | None = None,
    include_state_t0: bool = False,
    include_planner_details: bool = False,
    include_raw_relations: bool = False,
) -> dict:
    """Project raw grounding seams into the compact LLM-facing world view."""
    from planner_common.contracts import normalize_grounded_context

    normalized = normalize_grounded_context(grounded_context)
    if 'entities' in normalized:
        compact = {
            'entities': [
                _normalize_grounded_entity(
                    item,
                    include_raw_relations=include_raw_relations,
                )
                for item in normalized.get('entities', [])
                if isinstance(item, dict)
            ],
        }
        compact['counts'] = _normalize_grounded_counts(
            normalized.get('counts', {}),
            compact['entities'],
        )
        if include_state_t0 and isinstance(normalized.get('state_t0'), dict):
            compact['state_t0'] = dict(normalized.get('state_t0', {}))
        return compact

    scene_summary = normalized.get('scene_summary', {})
    state_t0 = normalized.get('state_t0', {})
    knowledge_snapshot = normalized.get('knowledge_snapshot', {})
    entities_by_id: dict[str, dict] = {}

    for item in _scene_items(scene_summary, 'objects'):
        entity_id = _first_non_empty(item.get('entity_id', ''), item.get('id', ''))
        if not entity_id:
            continue
        entity = _ensure_compact_entity(
            entities_by_id,
            entity_id,
            label=_display_entity_label(item.get('label', ''), entity_id),
            kind='object',
            entity_class=item.get('kb_class', item.get('type', '')),
        )
        _add_relation(entity, 'rdf:type', entity.get('class', ''))
        if include_planner_details:
            _copy_optional_planner_details(
                entity,
                item,
                ('center_x', 'center_y', 'last_seen_sec', 'last_seen_age_sec'),
            )

    for item in _scene_items(scene_summary, 'people'):
        entity_id = _first_non_empty(item.get('id', ''), item.get('entity_id', ''))
        if not entity_id:
            continue
        entity = _ensure_compact_entity(
            entities_by_id,
            entity_id,
            label=_person_label(item, entity_id),
            kind='person',
            entity_class=item.get('type', item.get('kb_class', 'Human')) or 'Human',
        )
        _add_relation(entity, 'rdf:type', entity.get('class', 'Human') or 'Human')
        if include_planner_details:
            _copy_optional_planner_details(
                entity,
                item,
                ('center_x', 'center_y', 'last_seen_sec', 'last_seen_age_sec'),
            )

    for item in _state_entities(state_t0):
        entity_id = _first_non_empty(item.get('id', ''), item.get('entity_id', ''))
        if not entity_id:
            continue
        kind = _normalized_kind(item.get('kind', ''), item.get('type', ''))
        entity = _ensure_compact_entity(
            entities_by_id,
            entity_id,
            label=_display_entity_label(item.get('normalized_name', ''), entity_id),
            kind=kind,
            entity_class=item.get('type', item.get('kb_class', '')),
        )
        _add_relation(entity, 'rdf:type', entity.get('class', ''))
        if include_planner_details:
            _copy_optional_planner_details(
                entity,
                item,
                ('last_seen_sec', 'last_seen_age_sec'),
            )

    for item in _knowledge_references(knowledge_snapshot):
        entity_id = _first_non_empty(item.get('id', ''), item.get('entity_id', ''))
        if not entity_id:
            continue
        entity = _ensure_compact_entity(
            entities_by_id,
            entity_id,
            label=_display_entity_label(item.get('normalized_name', ''), entity_id),
            kind=_normalized_kind('', item.get('type', '')),
            entity_class=item.get('type', ''),
        )
        _add_relation(entity, 'rdf:type', entity.get('class', ''))

    for row in knowledge_rows or []:
        if isinstance(row, dict):
            _merge_knowledge_row_relation(
                entities_by_id,
                row,
                include_raw_relation=include_raw_relations,
            )

    entities = sorted(
        (_finalize_compact_entity(item) for item in entities_by_id.values()),
        key=lambda item: (item.get('kind', ''), item.get('id', '')),
    )
    compact = {
        'entities': entities,
        'counts': _counts_from_entities(entities),
    }
    if include_state_t0 and isinstance(state_t0, dict) and state_t0:
        compact['state_t0'] = dict(state_t0)
    return compact


def grounded_context_to_context_ref(grounded_context: dict) -> dict:
    """Project planner-ingress grounding into compact plan lineage metadata."""
    from planner_common.contracts import normalize_grounded_context

    normalized = normalize_grounded_context(grounded_context)
    state_t0 = normalized.get('state_t0', {})
    scene_summary = normalized.get('scene_summary', {})
    return {
        'captured_at_sec': _coerce_float(
            _first_non_empty(
                state_t0.get('captured_at_sec', ''),
                scene_summary.get('captured_at_sec', ''),
                0.0,
            ),
            0.0,
        ),
        'observer': _first_non_empty(
            state_t0.get('observer', ''),
            scene_summary.get('observer', ''),
            '',
        ),
        'backend': _first_non_empty(
            state_t0.get('backend', ''),
            scene_summary.get('backend', ''),
            '',
        ),
    }
