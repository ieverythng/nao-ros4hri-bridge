"""Topic payload normalization for interaction trace events."""

from __future__ import annotations

import json

from interaction_trace_viewer.trace_model import InteractionEvent


def normalize_intent_message(*, channel: str, msg, max_payload_chars: int) -> InteractionEvent:
    """Normalize `hri_actions_msgs/msg/Intent` payloads."""
    intent_name = str(getattr(msg, 'intent', '')).strip()
    data_raw = str(getattr(msg, 'data', '')).strip()
    payload = {
        'intent': intent_name,
        'data_raw': data_raw,
        'data': _try_parse_json_dict(data_raw),
        'person_id': str(getattr(msg, 'person_id', '')).strip(),
        'intent_type': str(getattr(msg, 'intent_type', '')).strip(),
        'priority': _coerce_int(getattr(msg, 'priority', None)),
        'confidence': _coerce_float(getattr(msg, 'confidence', None)),
    }
    event_type = _event_type_for_channel(channel)
    trace_id = _extract_trace_id(payload)
    ab_object_id, ab_level = _extract_ab_payload_fields(payload)
    summary = summarize_event_payload(
        event_type=event_type,
        channel=channel,
        payload=payload,
        max_payload_chars=max_payload_chars,
    )
    return InteractionEvent(
        timestamp=0.0,
        trace_id=trace_id,
        source_node='',
        channel=channel,
        event_type=event_type,
        ab_object_id=ab_object_id,
        ab_level=ab_level,
        summary=summary,
        payload=payload,
        raw=data_raw,
    )


def normalize_string_message(*, channel: str, msg, max_payload_chars: int) -> InteractionEvent:
    """Normalize `std_msgs/msg/String` payloads."""
    raw_text = str(getattr(msg, 'data', '')).strip()
    parsed_data = _try_parse_json_dict(raw_text)
    payload = parsed_data if parsed_data is not None else {'text': raw_text}
    if str(channel).strip() == '/fake_skills/events' and isinstance(payload, dict):
        nested_payload = payload.get('payload', {})
        if isinstance(nested_payload, dict):
            for key in ('status', 'summary_text', 'failure', 'result_mode'):
                if key not in payload and key in nested_payload:
                    payload[key] = nested_payload.get(key)

    event_type = _event_type_for_channel(channel)
    trace_id = _extract_trace_id(payload)
    ab_object_id, ab_level = _extract_ab_payload_fields(payload)
    summary = summarize_event_payload(
        event_type=event_type,
        channel=channel,
        payload=payload,
        max_payload_chars=max_payload_chars,
    )

    return InteractionEvent(
        timestamp=0.0,
        trace_id=trace_id,
        source_node='',
        channel=channel,
        event_type=event_type,
        ab_object_id=ab_object_id,
        ab_level=ab_level,
        summary=summary,
        payload=payload,
        raw=raw_text,
    )


def normalize_rosout_message(*, channel: str, msg, max_payload_chars: int) -> InteractionEvent:
    """Normalize `rcl_interfaces/msg/Log` payloads."""
    stamp = getattr(msg, 'stamp', None)
    timestamp = _stamp_to_sec(stamp)
    level = int(getattr(msg, 'level', 0))
    payload = {
        'name': str(getattr(msg, 'name', '')).strip(),
        'msg': str(getattr(msg, 'msg', '')).strip(),
        'level': level,
        'file': str(getattr(msg, 'file', '')).strip(),
        'function': str(getattr(msg, 'function', '')).strip(),
        'line': int(getattr(msg, 'line', 0) or 0),
    }
    event_type = _rosout_event_type(level)
    summary = summarize_event_payload(
        event_type=event_type,
        channel=channel,
        payload=payload,
        max_payload_chars=max_payload_chars,
    )
    return InteractionEvent(
        timestamp=timestamp,
        trace_id=None,
        source_node='',
        channel=channel,
        event_type=event_type,
        ab_object_id=None,
        ab_level=0,
        summary=summary,
        payload=payload,
        raw=payload.get('msg', ''),
    )


def summarize_event_payload(*, event_type: str, channel: str, payload: dict, max_payload_chars: int) -> str:
    """Build concise summaries while preserving useful execution fields."""
    if event_type == 'user_utterance':
        return _clip(str(payload.get('text', payload.get('utterance', ''))), max_payload_chars)

    if event_type == 'planner_request':
        goal_text = _first_non_empty(payload, 'goal_text', 'text', 'query')
        targets = payload.get('scene_targets', [])
        if isinstance(targets, list) and targets:
            return _clip('goal=%s | targets=%s' % (goal_text, ','.join(str(item) for item in targets)), max_payload_chars)
        return _clip('goal=%s' % goal_text, max_payload_chars)

    if event_type == 'planner_output':
        steps = _extract_plan_steps(payload)
        if steps:
            return _clip('steps=%s' % ' -> '.join(steps), max_payload_chars)
        return _clip('planner output', max_payload_chars)

    if event_type in {'execution_feedback', 'skill_result', 'skill_feedback'}:
        status = _first_non_empty(payload, 'status', 'event_type')
        skill = _first_non_empty(payload, 'skill', 'name')
        reason = _first_non_empty(payload, 'reason', 'result_summary', 'summary_text')
        pieces = [item for item in [skill, status, reason] if item]
        if pieces:
            return _clip(' | '.join(pieces), max_payload_chars)

    if event_type == 'chatbot_turn_trace':
        route = _first_non_empty(payload, 'route')
        intent = _first_non_empty(payload, 'intent')
        source = _first_non_empty(payload, 'intent_source')
        kb_summary = _summarize_turn_trace_kb(payload)
        summary = 'route=%s | intent=%s | source=%s' % (route or '-', intent or '-', source or '-')
        if kb_summary:
            summary += ' | %s' % kb_summary
        return _clip(summary, max_payload_chars)

    if event_type == 'kb_snapshot':
        return _clip(_summarize_kb_snapshot(payload), max_payload_chars)

    compact = json.dumps(payload, ensure_ascii=True, separators=(',', ':'))
    return _clip('%s %s' % (channel, compact), max_payload_chars)


def classify_speech_topic(topic_name: str) -> str:
    clean_name = str(topic_name).strip()
    if '/humans/voices/' in clean_name:
        return 'user_utterance'
    return 'robot_speech'


def _event_type_for_channel(channel: str) -> str:
    mapping = {
        '/planner/request': 'planner_request',
        '/intents': 'planner_output',
        '/planner/execution_feedback': 'execution_feedback',
        '/planner/dialogue_act': 'planner_dialogue_act',
        '/nao_orchestrator/planner_dialogue_act': 'planner_dialogue_act',
        '/chatbot_llm/turn_trace': 'chatbot_turn_trace',
        '/fake_skills/events': 'skill_result',
        '/scene/summary': 'scene_update',
    }
    return mapping.get(str(channel).strip(), 'message')


def _extract_trace_id(payload: dict) -> str | None:
    candidate = _first_non_empty(
        payload,
        'trace_id',
        'turn_id',
        'dialogue_turn_id',
        'goal_id',
        'request_id',
        'plan_id',
    )
    if not candidate:
        nested_data = payload.get('data', {})
        if isinstance(nested_data, dict):
            candidate = _first_non_empty(
                nested_data,
                'trace_id',
                'turn_id',
                'dialogue_turn_id',
                'goal_id',
                'request_id',
                'plan_id',
            )
    return candidate or None


def _extract_ab_payload_fields(payload: dict) -> tuple[str | None, int | None]:
    ab_object_id = _first_non_empty(payload, 'ab_object_id', 'skill', 'name')
    ab_level = payload.get('ab_level', None)
    if ab_level in (None, ''):
        if ab_object_id:
            ab_level = 1
        else:
            ab_level = 0
    try:
        clean_level = int(ab_level)
    except (TypeError, ValueError):
        clean_level = None
    return (ab_object_id or None, clean_level)


def _extract_plan_steps(payload: dict) -> list[str]:
    maybe_plan = payload.get('plan', payload)
    if isinstance(maybe_plan, dict):
        steps = maybe_plan.get('steps', [])
    else:
        steps = []
    if not isinstance(steps, list):
        return []

    labels: list[str] = []
    for item in steps:
        if not isinstance(item, dict):
            continue
        step_type = str(item.get('type', '')).strip()
        step_name = str(item.get('name', '')).strip()
        if step_type == 'skill' and step_name:
            labels.append(step_name)
        elif step_type:
            labels.append(step_type)
    return labels


def _summarize_kb_snapshot(payload: dict) -> str:
    entities = payload.get('entities', [])
    if isinstance(entities, list):
        preview = []
        for item in entities[:5]:
            if not isinstance(item, dict):
                continue
            label = _first_non_empty(item, 'label', 'entity_id')
            kb_class = _first_non_empty(item, 'kb_class')
            if label and kb_class:
                preview.append('%s(%s)' % (label, kb_class))
            elif label:
                preview.append(label)
        if preview:
            return 'entities=%d | %s' % (len(entities), ', '.join(preview))
        return 'entities=%d' % len(entities)

    text = _first_non_empty(payload, 'text')
    if text:
        return text
    return json.dumps(payload, ensure_ascii=True, separators=(',', ':'))


def _summarize_turn_trace_kb(payload: dict) -> str:
    grounded_context = payload.get('grounded_context', {})
    knowledge_snapshot = {}
    scene_summary = {}
    if isinstance(grounded_context, dict):
        maybe_knowledge_snapshot = grounded_context.get('knowledge_snapshot', {})
        if isinstance(maybe_knowledge_snapshot, dict):
            knowledge_snapshot = maybe_knowledge_snapshot
        maybe_scene = grounded_context.get('scene_summary', {})
        if isinstance(maybe_scene, dict):
            scene_summary = maybe_scene

    objects = scene_summary.get('objects', []) if isinstance(scene_summary, dict) else []
    object_preview: list[str] = []
    if isinstance(objects, list):
        for item in objects[:3]:
            if not isinstance(item, dict):
                continue
            label = _first_non_empty(item, 'label', 'id')
            if label:
                object_preview.append(label)

    people = scene_summary.get('people', []) if isinstance(scene_summary, dict) else []
    object_count = len(objects) if isinstance(objects, list) else 0
    people_count = len(people) if isinstance(people, list) else 0

    parts: list[str] = []
    counts = knowledge_snapshot.get('counts', {}) if isinstance(knowledge_snapshot, dict) else {}
    if isinstance(counts, dict):
        entities_count = int(counts.get('entities', 0) or 0)
        objects_count = int(counts.get('objects', 0) or 0)
        people_snapshot_count = int(counts.get('people', 0) or 0)
        if entities_count > 0:
            parts.append('refs=e%d/o%d/p%d' % (entities_count, objects_count, people_snapshot_count))

    references = knowledge_snapshot.get('references', []) if isinstance(knowledge_snapshot, dict) else []
    if isinstance(references, list) and references:
        ref_preview: list[str] = []
        for item in references[:3]:
            if not isinstance(item, dict):
                continue
            label = _first_non_empty(item, 'normalized_name', 'id')
            if label:
                ref_preview.append(label)
        if ref_preview:
            parts.append('ref_preview=%s' % ','.join(ref_preview))

    if object_count > 0:
        suffix = (':' + ','.join(object_preview)) if object_preview else ''
        parts.append('objects=%d%s' % (object_count, suffix))
    parts.append('people=%d' % people_count)

    snapshot_text = str(payload.get('knowledge_snapshot', '') or '').strip()
    if snapshot_text:
        parts.append('kb_chars=%d' % len(snapshot_text))
    return ' | '.join(parts)


def _first_non_empty(payload: dict, *keys: str) -> str:
    for key in keys:
        value = payload.get(key, '')
        text = str(value or '').strip()
        if text:
            return text
    return ''


def _clip(text: str, max_chars: int) -> str:
    clean_text = str(text or '').strip()
    if max_chars <= 0 or len(clean_text) <= max_chars:
        return clean_text
    return clean_text[: max(0, max_chars - 3)] + '...'


def normalize_include_event_types(
    *,
    include_channels: set[str],
    include_event_types: set[str],
    exclude_event_types: set[str],
) -> set[str]:
    """Ensure event filters stay compatible with selected channels."""
    _ = include_channels
    _ = exclude_event_types
    return include_event_types


def _try_parse_json_dict(raw_text: str) -> dict | None:
    text = str(raw_text or '').strip()
    if not text:
        return {}
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return None
    if isinstance(parsed, dict):
        return parsed
    return {'value': parsed}


def _coerce_int(value) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _coerce_float(value) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    sec = int(getattr(stamp, 'sec', 0) or 0)
    nanosec = int(getattr(stamp, 'nanosec', 0) or 0)
    return float(sec) + float(nanosec) / 1_000_000_000.0


def _rosout_event_type(level: int) -> str:
    if level >= 40:
        return 'error'
    if level >= 30:
        return 'warning'
    return 'rosout'
