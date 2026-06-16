"""Shared JSON contracts and normalization helpers for planner-related nodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
import sys
import time


DEFAULT_PLANNER_REQUEST_INTENT = 'planner_request'


class IntentLabels:
    """Stable hri_actions intent label strings used by pure contract code."""

    BRING_OBJECT = 'bring_object'
    GRAB_OBJECT = 'grab_object'
    GREET = 'greet'
    GUIDE = 'guide'
    MOVE_TO = 'move_to'
    PERFORM_MOTION = 'perform_motion'
    PLACE_OBJECT = 'place_object'
    PRESENT_CONTENT = 'present_content'
    RAW_USER_INPUT = 'raw_user_input'
    SAY = 'say'
    START_ACTIVITY = 'start_activity'
    STOP_ACTIVITY = 'stop_activity'
    SUSPEND = 'suspend'
    WAKEUP = 'wakeup'


PLAN_STEP_TYPES = ('noop', 'say', 'skill', 'look_at')
PLAN_FAILURE_POLICIES = (
    'fail',
    'continue',
    'replan',
    'clarify',
    'ask_user',
    'ignore',
)
_ASK_USER_STEP_NAMES = ('ask_user', 'ask_clarification', 'ask_for_help')
PLANNER_REQUEST_KINDS = (
    'new_goal',
    'goal_update',
    'clarification_answer',
    'cancel_request',
)
SUPERVISOR_STATUSES = (
    'idle',
    'planning',
    'executing',
    'blocked',
    'waiting_user',
    'replanning',
    'completed',
    'failed',
    'cancelled',
    'superseded',
)
PLANNER_DIALOGUE_ACTS = (
    'acknowledge',
    'progress_update',
    'ask_clarification',
    'ask_for_help',
    'explain_failure',
    'notify_completion',
    'notify_cancellation',
)
_DEFAULT_GROUNDED_CONTEXT = {
    'knowledge_snapshot': {},
    'scene_summary': {},
    'state_t0': {},
}
_COMPACT_GROUNDED_CONTEXT_KEYS = ('entities',)
_LLM_RELATION_PREDICATE_PRIORITY = (
    'rdf:type',
    'dbp:name',
    'dbp:color',
    'oro:isAt',
    'oro:isOn',
    'oro:contains',
    'foaf:knows',
)
_LLM_RELATION_PREDICATE_ALIASES = {
    'type': 'rdf:type',
    'rdf type': 'rdf:type',
    'name': 'dbp:name',
    'color': 'dbp:color',
    'isat': 'oro:isAt',
    'is_at': 'oro:isAt',
    'is at': 'oro:isAt',
    'ison': 'oro:isOn',
    'is_on': 'oro:isOn',
    'is on': 'oro:isOn',
    'contains': 'oro:contains',
    'knows': 'foaf:knows',
}
_MAX_LLM_RELATIONS_PER_ENTITY = 6
_DEFAULT_COMMUNICATION_POLICY = {
    'emit_acknowledge': False,
    'emit_progress': False,
    'emit_completion': True,
    'emit_failure': True,
}
_SPEECH_PRODUCING_STEP_NAMES = frozenset(('report_result', 'say'))
_LOOK_AT_RESET_POLICY_ALIASES = frozenset(
    (
        'reset',
        'look_at_reset',
        'look_reset',
        'gaze_reset',
        'reset_gaze',
        'reset_look_at',
    )
)
_LOOK_AT_RESET_TARGET_ALIASES = frozenset(
    (
        'head_center',
        'center',
        'forward',
        'straight',
    )
)
_LOOK_AT_TARGETLESS_POLICIES = frozenset(('auto', 'random', 'social'))

_FROZEN_DATACLASS_KWARGS = {'frozen': True}
if sys.version_info >= (3, 10):  # pragma: no branch - local macOS uses Python 3.9
    _FROZEN_DATACLASS_KWARGS['slots'] = True


def parse_json_object(payload) -> dict:
    """Parse one JSON object payload into a stable dict."""
    if isinstance(payload, dict):
        return dict(payload)

    text = str(payload or '').strip()
    if not text:
        return {}

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return {}

    if isinstance(parsed, dict):
        return parsed
    return {}


def extract_json_object(payload) -> dict:
    """Extract the first JSON object from raw model output."""
    text = str(payload or '').strip()
    if not text:
        return {}

    direct = parse_json_object(text)
    if direct:
        return direct

    if '```' in text:
        for part in text.split('```'):
            candidate = part.strip()
            if candidate.startswith('json'):
                candidate = candidate[4:].strip()
            parsed = parse_json_object(candidate)
            if parsed:
                return parsed

    start = text.find('{')
    end = text.rfind('}')
    if start == -1 or end == -1 or end <= start:
        return {}
    return parse_json_object(text[start : end + 1])


def truncate_text(value: str, max_chars: int) -> str:
    """Clamp text to a bounded number of characters."""
    clean_value = ' '.join(str(value or '').split())
    if max_chars <= 0:
        return ''
    if len(clean_value) <= max_chars:
        return clean_value
    if max_chars <= 1:
        return clean_value[:max_chars]
    return clean_value[: max_chars - 1] + '…'


def coerce_str_list(value) -> list[str]:
    """Normalize strings, tuples, and lists into a clean string list."""
    if isinstance(value, str):
        return [clean for clean in [value.strip()] if clean]
    if not isinstance(value, (list, tuple)):
        return []
    return [clean for clean in (str(item).strip() for item in value) if clean]


def coerce_bool(value) -> bool:
    """Normalize common JSON-ish boolean representations."""
    if isinstance(value, bool):
        return value
    clean_value = str(value or '').strip().lower()
    if clean_value in ('1', 'true', 'yes', 'on'):
        return True
    if clean_value in ('0', 'false', 'no', 'off'):
        return False
    return bool(value)


def _coerce_failure_policy(value) -> str:
    clean_value = str(value or '').strip().lower()
    if clean_value in PLAN_FAILURE_POLICIES:
        return clean_value
    return 'fail'


def _coerce_nonnegative_int(value) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return 0
    return max(0, parsed)


def _coerce_float(value, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(fallback)


def coerce_optional_float(value) -> float | None:
    """Coerce a value to float, returning None on failure."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def optional_float_fields(source: dict, keys: tuple[str, ...]) -> dict[str, float]:
    """Return requested source fields whose values are valid floats."""
    normalized: dict[str, float] = {}
    for key in keys:
        value = coerce_optional_float(source.get(key))
        if value is not None:
            normalized[key] = value
    return normalized


def request_requests_report(request) -> bool:
    """Return True if the request includes a report_result intent."""
    return any(
        str(intent_name or '').strip().lower() == 'report_result'
        for intent_name in getattr(request, 'normalized_intents', ())
    )


_LIVE_RESULT_REPORT_SKILLS = {
    'find_object',
    'inspect_area',
    'look_at',
    'navigate_to',
    'perform_motion',
    'scan',
    'walk_to',
    'wave_greet',
}


def live_result_report_summary_error(steps: list[dict]) -> str:
    """Check that report_result after executable skills reuses live result text."""
    previous_skill_name = ''
    for step in steps:
        step_name = str(step.get('name', '')).strip().lower()
        if step_name == 'report_result' and previous_skill_name in _LIVE_RESULT_REPORT_SKILLS:
            summary_text = str(
                (step.get('args', {}) or {}).get('summary_text', '')
            ).strip()
            if summary_text:
                return (
                    'report_result after %s must omit summary_text so the '
                    'executor reports the latest live skill result'
                ) % previous_skill_name
        if step_name == 'report_result':
            previous_skill_name = ''
            continue
        if str(step.get('type', '')).strip().lower() == 'skill':
            previous_skill_name = step_name
    return ''


def strip_live_result_report_summary_text(steps: list[dict]) -> list[dict]:
    """Remove prefilled report text when report_result must reuse live step output."""
    repaired_steps: list[dict] = []
    previous_skill_name = ''
    for step in steps:
        repaired_step = dict(step)
        step_name = str(repaired_step.get('name', '')).strip().lower()
        if step_name == 'report_result' and previous_skill_name in _LIVE_RESULT_REPORT_SKILLS:
            step_args = dict(repaired_step.get('args', {}) or {})
            for key in ('summary_text', 'result_summary', 'text', 'message', 'utterance'):
                step_args.pop(key, None)
            repaired_step['args'] = step_args
        repaired_steps.append(repaired_step)

        if step_name == 'report_result':
            previous_skill_name = ''
            continue
        if str(repaired_step.get('type', '')).strip().lower() == 'skill':
            previous_skill_name = step_name
    return repaired_steps


def scan_report_summary_error(steps: list[dict]) -> str:
    """Compatibility alias for the live-result report_result contract."""
    error = live_result_report_summary_error(steps)
    if error.startswith('report_result after scan must omit summary_text'):
        return (
            'report_result after scan must omit summary_text so the '
            'executor reports the latest live scan result'
        )
    return error


def missing_requested_report_error(request, steps: list[dict]) -> str:
    """Check that a report-asking request includes a report_result step."""
    if not request_requests_report(request):
        return ''
    if any(str(step.get('name', '')).strip().lower() == 'report_result' for step in steps):
        return ''
    return (
        'request asks for a user-facing report; include report_result after '
        'the evidence-producing step, with empty args after scan/perception '
        'so the executor reuses live skill evidence'
    )


def _first_non_empty(*values) -> str:
    for value in values:
        clean_value = str(value or '').strip()
        if clean_value:
            return clean_value
    return ''


def _clean_payload(value) -> dict:
    if not isinstance(value, dict):
        return {}
    return {
        str(key): item
        for key, item in value.items()
        if item not in (None, '', [])
    }


def _normalize_result_payload_item(value):
    if isinstance(value, dict):
        return _normalize_result_payload(value)
    if isinstance(value, list):
        return [
            _normalize_result_payload_item(item)
            for item in value
            if item is not None
        ]
    if isinstance(value, str):
        return value.strip()
    if isinstance(value, (bool, int, float)):
        return value
    return str(value)


def _normalize_result_payload(value) -> dict:
    if not isinstance(value, dict):
        return {}
    normalized = {}
    for key, item in value.items():
        clean_key = str(key or '').strip()
        if not clean_key or item is None:
            continue
        normalized[clean_key] = _normalize_result_payload_item(item)
    return normalized


def _normalize_look_at_args(step_args: dict) -> dict:
    if not isinstance(step_args, dict):
        return {}

    normalized = _clean_payload(step_args)
    policy = str(
        normalized.get('policy', normalized.get('object', ''))
    ).strip().lower()
    if policy in _LOOK_AT_RESET_POLICY_ALIASES:
        normalized['policy'] = 'reset'
        normalized.pop('target_frame', None)
        normalized.pop('frame_id', None)
        normalized.pop('target', None)
        normalized.pop('entity_id', None)
        return normalized
    if policy in _LOOK_AT_TARGETLESS_POLICIES:
        normalized['policy'] = policy
        normalized.pop('target_frame', None)
        normalized.pop('frame_id', None)
        normalized.pop('target', None)
        normalized.pop('entity_id', None)
        return normalized

    target_frame = _first_non_empty(
        normalized.get('target_frame', ''),
        normalized.get('frame_id', ''),
        normalized.get('target', ''),
        normalized.get('entity_id', ''),
    )
    if not target_frame:
        return normalized

    clean_target = str(target_frame).strip()
    if clean_target.lower() in _LOOK_AT_RESET_TARGET_ALIASES:
        normalized['policy'] = 'reset'
        normalized.pop('target_frame', None)
        normalized.pop('frame_id', None)
        normalized.pop('target', None)
        normalized.pop('entity_id', None)
        return normalized

    normalized['target_frame'] = clean_target
    normalized.pop('frame_id', None)
    normalized.pop('target', None)
    normalized.pop('entity_id', None)
    return normalized


def _normalize_choice(value: str, allowed: tuple[str, ...], fallback: str) -> str:
    clean_value = str(value or '').strip().lower()
    if clean_value in allowed:
        return clean_value
    return fallback


def make_runtime_id(prefix: str) -> str:
    """Create one traceable runtime identifier."""
    clean_prefix = str(prefix or '').strip() or 'id'
    return '%s_%d' % (clean_prefix, int(time.time() * 1000))


def make_plan_id(prefix: str = 'plan') -> str:
    """Create a plan identifier stable enough for local tracing."""
    return make_runtime_id(prefix or 'plan')


def make_goal_id(prefix: str = 'goal') -> str:
    """Create a goal identifier stable enough for local tracing."""
    return make_runtime_id(prefix or 'goal')


def normalize_grounded_context(value) -> dict:
    """Normalize the planner-facing grounded context envelope."""
    raw_payload = parse_json_object(value) if isinstance(value, str) else value
    if not isinstance(raw_payload, dict):
        return dict(_DEFAULT_GROUNDED_CONTEXT)

    if any(key in raw_payload for key in _COMPACT_GROUNDED_CONTEXT_KEYS):
        entities = raw_payload.get('entities', [])
        if not isinstance(entities, list):
            entities = []
        normalized = {
            'entities': [
                _normalize_grounded_entity(item)
                for item in entities
                if isinstance(item, dict)
            ],
        }
        state_t0 = raw_payload.get('state_t0', {})
        if isinstance(state_t0, dict) and state_t0:
            normalized['state_t0'] = dict(state_t0)
        return normalized

    payload = dict(_DEFAULT_GROUNDED_CONTEXT)
    for key in ('knowledge_snapshot', 'scene_summary', 'state_t0'):
        item = raw_payload.get(key, {})
        payload[key] = item if isinstance(item, dict) else {}
    return payload


def project_llm_grounded_context(
    grounded_context: dict,
    *,
    knowledge_rows: list[dict] | None = None,
    include_state_t0: bool = False,
    include_planner_details: bool = False,
    include_raw_relations: bool = False,
) -> dict:
    """Project raw grounding seams into the compact LLM-facing world view."""
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
        if include_planner_details:
            _copy_optional_planner_details(
                entity,
                item,
                (
                    'center_x',
                    'center_y',
                    'last_seen_sec',
                    'last_seen_age_sec',
                    'distance_m',
                ),
            )
        _copy_frame_qualified_position(entity, item)

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
        if include_planner_details:
            _copy_optional_planner_details(
                entity,
                item,
                (
                    'center_x',
                    'center_y',
                    'last_seen_sec',
                    'last_seen_age_sec',
                    'distance_m',
                ),
            )
        _copy_frame_qualified_position(entity, item)

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

    for row in knowledge_rows or []:
        if isinstance(row, dict):
            _merge_knowledge_row_relation(
                entities_by_id,
                row,
                include_raw_relation=include_raw_relations,
            )

    entities = sorted(
        (_finalize_compact_entity(item) for item in entities_by_id.values()),
        key=_llm_entity_sort_key,
    )
    compact = {'entities': entities}
    if include_state_t0 and isinstance(state_t0, dict) and state_t0:
        compact['state_t0'] = dict(state_t0)
    return compact


def grounded_context_to_context_ref(grounded_context: dict) -> dict:
    """Project planner-ingress grounding into compact plan lineage metadata."""
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
        'relations': _normalize_relations(
            item.get('relations', []),
            entity_class=item.get('class', item.get('type', '')),
        ),
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


def _normalize_relations(
    value,
    *,
    max_relations: int = _MAX_LLM_RELATIONS_PER_ENTITY,
    entity_class='',
) -> list[dict]:
    if not isinstance(value, list):
        return []
    relations_by_key = {}
    seen = set()
    seen_rdf_type = False
    compact_class = _compact_term(entity_class)
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
            if compact_class and _compact_term(obj) == compact_class:
                continue
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
    if not isinstance(references, list):
        return []
    return [item for item in references if isinstance(item, dict)]


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


def _copy_frame_qualified_position(entity: dict, source: dict) -> None:
    frame_id = str(source.get('frame_id', '')).strip()
    position = _normalize_position(source.get('position', {}))
    if not frame_id or not position:
        return
    entity['frame_id'] = frame_id
    entity['position'] = position
    distance_m = coerce_optional_float(source.get('distance_m'))
    if distance_m is not None:
        entity['distance_m'] = distance_m


def _normalize_position(value) -> dict:
    if not isinstance(value, dict):
        return {}
    position: dict[str, float] = {}
    for axis in ('x', 'y', 'z'):
        parsed = coerce_optional_float(value.get(axis))
        if parsed is not None:
            position[axis] = parsed
    return position if len(position) == 3 else {}


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
        entity_class=_compact_term(obj) if predicate in ('rdf:type', 'type') else '',
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


def _finalize_compact_entity(entity: dict) -> dict:
    finalized = {
        'id': str(entity.get('id', '')).strip(),
        'label': entity.get('label') if entity.get('label') else None,
        'kind': str(entity.get('kind', 'object')).strip() or 'object',
        'class': str(entity.get('class', '')).strip(),
        'visible': coerce_bool(entity.get('visible', True)),
        'relations': _normalize_relations(
            entity.get('relations', []),
            entity_class=entity.get('class', ''),
        ),
    }
    raw_relations = _normalize_raw_relations(entity.get('raw_relations', []))
    if raw_relations:
        finalized['raw_relations'] = raw_relations
    for key in ('center_x', 'center_y', 'last_seen_sec', 'last_seen_age_sec'):
        if key in entity:
            finalized[key] = entity[key]
    if str(entity.get('frame_id', '')).strip() and isinstance(entity.get('position'), dict):
        finalized['frame_id'] = str(entity.get('frame_id', '')).strip()
        finalized['position'] = dict(entity['position'])
    if 'distance_m' in entity:
        finalized['distance_m'] = entity['distance_m']
    return {
        key: value
        for key, value in finalized.items()
        if key == 'label' or value not in ('', [], {})
    }


def _llm_entity_sort_key(entity: dict) -> tuple:
    clean_id = str(entity.get('id', '')).strip().lower()
    generated_rank = 1 if clean_id.startswith(('detected_', 'anonymous_')) else 0
    kind_rank = 0 if str(entity.get('kind', '')).strip().lower() == 'object' else 1
    relation_rank = 0 if entity.get('relations') else 1
    return (generated_rank, kind_rank, relation_rank, clean_id)


def normalize_communication_policy(value) -> dict:
    """Normalize plan communication flags into a stable dict."""
    policy = dict(_DEFAULT_COMMUNICATION_POLICY)
    if not isinstance(value, dict):
        return policy

    for key in policy:
        if key in value:
            policy[key] = coerce_bool(value.get(key))
    return policy


def resolve_effective_communication_policy(value, steps) -> dict:
    """Resolve communication flags against the validated executable plan shape."""
    policy = normalize_communication_policy(value)
    normalized_steps = normalize_plan_steps(list(steps or []))
    if any(_step_produces_speech(step) for step in normalized_steps):
        policy['emit_completion'] = False
    return policy


def _step_produces_speech(step: dict) -> bool:
    step_type = str(step.get('type', '')).strip().lower()
    step_name = str(step.get('name', '')).strip().lower()
    return step_type == 'say' or step_name in _SPEECH_PRODUCING_STEP_NAMES


def normalize_plan_steps(steps) -> list[dict]:
    """Normalize a plan-step list into the orchestrator-expected structure."""
    if not isinstance(steps, list):
        return []

    normalized_steps: list[dict] = []
    for index, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            continue
        step_type = str(step.get('type', '')).strip().lower()
        if step_type not in PLAN_STEP_TYPES:
            continue
        step_name = str(step.get('name', '')).strip().lower()
        raw_failure_policy = _first_non_empty(
            step.get('on_failure', ''),
            step.get('failure_policy', ''),
        )
        normalized_failure_policy = _coerce_failure_policy(
            raw_failure_policy or 'fail'
        )
        if step_type == 'skill' and step_name in _ASK_USER_STEP_NAMES and not raw_failure_policy:
            normalized_failure_policy = 'ask_user'
        step_args = _clean_payload(step.get('args', {}))
        if step_type == 'look_at' or step_name == 'look_at':
            step_args = _normalize_look_at_args(step_args)

        normalized_steps.append(
            {
                'id': _first_non_empty(
                    step.get('id', ''),
                    step.get('step_id', ''),
                    f'step_{index}',
                ),
                'type': step_type,
                'name': step_name,
                'args': step_args,
                'requires': coerce_str_list(step.get('requires', step.get('preconditions', []))),
                'on_failure': normalized_failure_policy,
                'retry_budget': _coerce_nonnegative_int(
                    step.get('retry_budget', step.get('retries', 0))
                ),
            }
        )
    return normalized_steps


def build_plan_payload(
    *,
    request,
    steps,
    validation_status: str = 'draft',
    failure_reason: str = '',
    user_facing_reason: str = '',
    replan_hint: str = '',
    retry_budget: int = 0,
    scene_targets: list[str] | None = None,
    plan_id: str = '',
    goal_id: str = '',
    plan_version: int = 1,
    status: str = 'draft',
    communication_policy: dict | None = None,
    communication_policy_source: str = '',
) -> dict:
    """Build one planner result payload using the shared envelope shape."""
    resolved_scene_targets = list(scene_targets or getattr(request, 'scene_targets', []))
    resolved_goal_id = str(goal_id or getattr(request, 'goal_id', '')).strip()
    resolved_plan_id = str(plan_id or make_plan_id()).strip()
    resolved_steps = normalize_plan_steps(list(steps or []))
    resolved_policy = resolve_effective_communication_policy(
        communication_policy,
        resolved_steps,
    )
    resolved_grounded_context = normalize_grounded_context(
        getattr(request, 'grounded_context', {})
    )

    plan = {
        'goal_id': resolved_goal_id,
        'plan_id': resolved_plan_id,
        'plan_version': max(1, int(plan_version or 1)),
        'status': str(status or '').strip().lower() or 'draft',
        'validation_status': str(validation_status or '').strip().lower(),
        'user_facing_reason': str(user_facing_reason or '').strip(),
        'replan_hint': str(replan_hint or '').strip(),
        'retry_budget': _coerce_nonnegative_int(retry_budget),
        'scene_targets': resolved_scene_targets,
        'context_ref': grounded_context_to_context_ref(resolved_grounded_context),
        'communication_policy': resolved_policy,
        'communication_policy_source': str(communication_policy_source or '').strip(),
        'steps': resolved_steps,
    }
    clean_failure_reason = str(failure_reason or '').strip()
    if clean_failure_reason:
        plan['failure_reason'] = clean_failure_reason
    return {'plan': plan}


def build_dialogue_act_payload(
    *,
    goal_id: str,
    act: str,
    plan_id: str = '',
    plan_version: int = 0,
    priority: str = 'normal',
    await_user_response: bool = False,
    reason: str = '',
    text_hint: str = '',
    slots_needed: list[str] | None = None,
    context: dict | None = None,
) -> dict:
    """Build one planner dialogue act payload."""
    return {
        'goal_id': str(goal_id or '').strip(),
        'plan_id': str(plan_id or '').strip(),
        'plan_version': max(0, int(plan_version or 0)),
        'act': _normalize_choice(act, PLANNER_DIALOGUE_ACTS, 'progress_update'),
        'priority': str(priority or 'normal').strip().lower() or 'normal',
        'await_user_response': bool(await_user_response),
        'reason': str(reason or '').strip(),
        'text_hint': str(text_hint or '').strip(),
        'slots_needed': coerce_str_list(slots_needed or []),
        'context': _clean_payload(context or {}),
    }


def build_execution_feedback_payload(
    *,
    intent: str,
    source: str,
    plan_context: dict,
    status: str,
    event_type: str = '',
    reason: str = '',
    step: dict | None = None,
    blocking: bool = False,
    unmet_preconditions: list[str] | None = None,
    needs_user_input: bool = False,
    validation_errors: list[str] | None = None,
    timestamp_sec: float = 0.0,
    result_summary: str = '',
    result_payload: dict | None = None,
) -> dict:
    """Build one normalized planner feedback payload."""
    resolved_step = step if isinstance(step, dict) else None
    # Retry budget is plan-level remaining budget. Keep it independent from any
    # per-step retry metadata so supervisor replan accounting cannot stall.
    resolved_retry_budget = _coerce_nonnegative_int(plan_context.get('retry_budget', 0))
    normalized_result_payload = _normalize_result_payload(result_payload or {})
    normalized_result_summary = str(result_summary or '').strip()
    if not normalized_result_summary:
        normalized_result_summary = str(
            normalized_result_payload.get('summary_text', '')
        ).strip()
    payload = {
        'goal_id': str(plan_context.get('goal_id', '')).strip(),
        'plan_id': str(plan_context.get('plan_id', '')).strip(),
        'plan_version': max(0, _coerce_nonnegative_int(plan_context.get('plan_version', 0))),
        'intent': str(intent or '').strip(),
        'source': str(source or '').strip(),
        'event_type': str(
            event_type or _default_feedback_event_type(status)
        ).strip().lower(),
        'status': str(status or '').strip().lower(),
        'reason': str(reason or '').strip(),
        'validation_status': str(plan_context.get('validation_status', '')).strip().lower(),
        'replan_hint': str(plan_context.get('replan_hint', '')).strip(),
        'retry_budget': resolved_retry_budget,
        'blocking': bool(blocking),
        'unmet_preconditions': coerce_str_list(unmet_preconditions or []),
        'needs_user_input': bool(needs_user_input),
        'scene_targets': coerce_str_list(plan_context.get('scene_targets', [])),
        'validation_errors': coerce_str_list(validation_errors or []),
        'timestamp_sec': _coerce_float(timestamp_sec, time.time()),
        'result_summary': normalized_result_summary,
        'result_payload': normalized_result_payload,
    }
    if resolved_step is not None:
        payload['step'] = {
            'id': str(resolved_step.get('id', '')).strip(),
            'type': str(resolved_step.get('type', '')).strip().lower(),
            'name': str(resolved_step.get('name', '')).strip().lower(),
            'retry_budget': _coerce_nonnegative_int(
                resolved_step.get('retry_budget', resolved_retry_budget)
            ),
            'on_failure': _coerce_failure_policy(
                resolved_step.get('on_failure', resolved_step.get('failure_policy', 'fail'))
            ),
            'requires': coerce_str_list(
                resolved_step.get('requires', resolved_step.get('preconditions', []))
            ),
        }
    return payload


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class PlannerRequest:
    """Normalized planner ingress payload."""

    request_id: str
    goal_id: str
    parent_goal_id: str
    supersedes_goal_id: str
    request_kind: str
    goal_text: str
    user_text: str
    normalized_intents: tuple[str, ...]
    scene_targets: tuple[str, ...]
    dialogue_context: tuple[str, ...]
    grounded_context: dict
    planner_mode: str
    dialogue_turn_id: str

    @classmethod
    def from_payload(cls, payload) -> 'PlannerRequest':
        data = parse_json_object(payload)
        dialogue_context = data.get('dialogue_context', [])
        if isinstance(dialogue_context, str):
            dialogue_context = [dialogue_context]

        request_id = _first_non_empty(data.get('request_id', ''), make_runtime_id('request'))
        goal_id = _first_non_empty(data.get('goal_id', ''), make_goal_id())

        return cls(
            request_id=request_id,
            goal_id=goal_id,
            parent_goal_id=str(data.get('parent_goal_id', '')).strip(),
            supersedes_goal_id=str(data.get('supersedes_goal_id', '')).strip(),
            request_kind=_normalize_choice(
                data.get('request_kind', 'new_goal'),
                PLANNER_REQUEST_KINDS,
                'new_goal',
            ),
            goal_text=str(
                data.get('goal_text', data.get('goal', data.get('task', '')))
            ).strip(),
            user_text=str(data.get('user_text', '')).strip(),
            normalized_intents=tuple(coerce_str_list(data.get('normalized_intents', []))),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            dialogue_context=tuple(coerce_str_list(dialogue_context)),
            grounded_context=normalize_grounded_context(data.get('grounded_context', {})),
            planner_mode=str(data.get('planner_mode', 'default')).strip() or 'default',
            dialogue_turn_id=str(data.get('dialogue_turn_id', '')).strip(),
        )


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class SceneObject:
    """One grounded object entry from `/scene/summary`."""

    entity_id: str
    label: str
    kb_class: str
    score: float
    tracker_id: str
    source: str
    center_x: float
    center_y: float
    last_seen_sec: float
    frame_id: str
    position: dict
    distance_m: float | None

    @classmethod
    def from_dict(cls, payload: dict) -> 'SceneObject':
        return cls(
            entity_id=str(payload.get('entity_id', '')).strip(),
            label=str(payload.get('label', '')).strip(),
            kb_class=str(payload.get('kb_class', '')).strip(),
            score=_coerce_float(payload.get('score', 0.0)),
            tracker_id=str(payload.get('tracker_id', '')).strip(),
            source=str(payload.get('source', '')).strip(),
            center_x=_coerce_float(payload.get('center_x', 0.0)),
            center_y=_coerce_float(payload.get('center_y', 0.0)),
            last_seen_sec=_coerce_float(payload.get('last_seen_sec', 0.0)),
            frame_id=str(payload.get('frame_id', '')).strip(),
            position=_normalize_position(payload.get('position', {})),
            distance_m=coerce_optional_float(payload.get('distance_m')),
        )


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class SceneSummary:
    """Normalized `/scene/summary` payload."""

    observer: str
    backend: str
    objects: tuple[SceneObject, ...]

    @classmethod
    def from_payload(cls, payload) -> 'SceneSummary':
        data = parse_json_object(payload)
        raw_objects = data.get('objects', [])
        if not isinstance(raw_objects, list):
            raw_objects = []
        return cls(
            observer=str(data.get('observer', '')).strip(),
            backend=str(data.get('backend', '')).strip(),
            objects=tuple(
                SceneObject.from_dict(item)
                for item in raw_objects
                if isinstance(item, dict)
            ),
        )


def _default_feedback_event_type(status: str) -> str:
    mapping = {
        'accepted': 'plan_accepted',
        'running': 'step_started',
        'succeeded': 'step_succeeded',
        'completed': 'plan_completed',
        'invalid': 'plan_invalid',
        'failed': 'step_failed',
    }
    return mapping.get(str(status or '').strip().lower(), '')


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class ExecutionFeedback:
    """Normalized planner/executor feedback payload."""

    goal_id: str
    plan_id: str
    plan_version: int
    event_type: str
    status: str
    intent: str
    source: str
    reason: str
    validation_status: str
    replan_hint: str
    retry_budget: int
    blocking: bool
    unmet_preconditions: tuple[str, ...]
    needs_user_input: bool
    scene_targets: tuple[str, ...]
    validation_errors: tuple[str, ...]
    step_id: str
    step_type: str
    step_name: str
    step_retry_budget: int
    step_on_failure: str
    step_requires: tuple[str, ...]
    timestamp_sec: float
    result_summary: str
    result_payload: dict

    @classmethod
    def from_payload(cls, payload) -> 'ExecutionFeedback':
        data = parse_json_object(payload)
        step_payload = data.get('step', {})
        if not isinstance(step_payload, dict):
            step_payload = {}
        step_failure_policy = step_payload.get('on_failure', step_payload.get('failure_policy', ''))
        clean_step_failure_policy = str(step_failure_policy or '').strip().lower()
        status = str(data.get('status', '')).strip().lower()
        result_payload = _normalize_result_payload(data.get('result_payload', {}))
        result_summary = str(data.get('result_summary', '')).strip()
        if not result_summary:
            result_summary = str(result_payload.get('summary_text', '')).strip()
        return cls(
            goal_id=str(data.get('goal_id', '')).strip(),
            plan_id=str(data.get('plan_id', '')).strip(),
            plan_version=max(0, _coerce_nonnegative_int(data.get('plan_version', 0))),
            event_type=str(
                data.get('event_type', _default_feedback_event_type(status))
            ).strip().lower(),
            status=status,
            intent=str(data.get('intent', '')).strip(),
            source=str(data.get('source', '')).strip(),
            reason=str(data.get('reason', '')).strip(),
            validation_status=str(data.get('validation_status', '')).strip().lower(),
            replan_hint=str(data.get('replan_hint', '')).strip(),
            retry_budget=_coerce_nonnegative_int(data.get('retry_budget', 0)),
            blocking=coerce_bool(data.get('blocking', False)),
            unmet_preconditions=tuple(
                coerce_str_list(data.get('unmet_preconditions', []))
            ),
            needs_user_input=coerce_bool(data.get('needs_user_input', False)),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            validation_errors=tuple(coerce_str_list(data.get('validation_errors', []))),
            step_id=str(step_payload.get('id', '')).strip(),
            step_type=str(step_payload.get('type', '')).strip().lower(),
            step_name=str(step_payload.get('name', '')).strip().lower(),
            step_retry_budget=_coerce_nonnegative_int(step_payload.get('retry_budget', 0)),
            step_on_failure=(
                _coerce_failure_policy(step_failure_policy)
                if clean_step_failure_policy
                else ''
            ),
            step_requires=tuple(
                coerce_str_list(
                    step_payload.get('requires', step_payload.get('preconditions', []))
                )
            ),
            timestamp_sec=_coerce_float(data.get('timestamp_sec', 0.0)),
            result_summary=result_summary,
            result_payload=result_payload,
        )


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class PlannerDialogueAct:
    """Normalized planner-owned dialogue act payload."""

    goal_id: str
    plan_id: str
    plan_version: int
    act: str
    priority: str
    await_user_response: bool
    reason: str
    text_hint: str
    slots_needed: tuple[str, ...]
    context: dict

    @classmethod
    def from_payload(cls, payload) -> 'PlannerDialogueAct':
        data = parse_json_object(payload)
        return cls(
            goal_id=str(data.get('goal_id', '')).strip(),
            plan_id=str(data.get('plan_id', '')).strip(),
            plan_version=max(0, _coerce_nonnegative_int(data.get('plan_version', 0))),
            act=_normalize_choice(data.get('act', ''), PLANNER_DIALOGUE_ACTS, 'progress_update'),
            priority=str(data.get('priority', 'normal')).strip().lower() or 'normal',
            await_user_response=coerce_bool(data.get('await_user_response', False)),
            reason=str(data.get('reason', '')).strip(),
            text_hint=str(data.get('text_hint', '')).strip(),
            slots_needed=tuple(coerce_str_list(data.get('slots_needed', []))),
            context=_clean_payload(data.get('context', {})),
        )
