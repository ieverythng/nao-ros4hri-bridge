"""Shared JSON contracts and normalization helpers for planner/WME nodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
import sys
import time


DEFAULT_PLANNER_REQUEST_INTENT = 'planner_request'
PLAN_STEP_TYPES = ('noop', 'say', 'skill', 'look_at')
PLAN_FAILURE_POLICIES = ('fail', 'continue', 'replan', 'clarify')

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

    fenced_text = text
    if '```' in fenced_text:
        parts = fenced_text.split('```')
        for part in parts:
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


def make_plan_id(prefix: str = 'plan') -> str:
    """Create a plan identifier stable enough for local tracing."""
    return '%s_%d' % (str(prefix or 'plan').strip() or 'plan', int(time.time() * 1000))


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
        normalized_steps.append(
            {
                'id': _first_non_empty(step.get('id', ''), step.get('step_id', ''), f'step_{index}'),
                'type': step_type,
                'name': str(step.get('name', '')).strip().lower(),
                'args': _clean_payload(step.get('args', {})),
                'requires': coerce_str_list(step.get('requires', step.get('preconditions', []))),
                'on_failure': _coerce_failure_policy(
                    step.get('on_failure', step.get('failure_policy', 'fail'))
                ),
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
    ack_text: str = '',
    ack_mode: str = '',
    validation_status: str = 'draft',
    failure_reason: str = '',
    replan_hint: str = '',
    retry_budget: int = 0,
    scene_targets: list[str] | None = None,
    plan_id: str = '',
) -> dict:
    """Build one planner result payload using the shared envelope shape."""
    resolved_scene_targets = list(scene_targets or getattr(request, 'scene_targets', []))
    return {
        'ack_text': str(ack_text or getattr(request, 'ack_text', '')).strip(),
        'ack_mode': str(ack_mode or getattr(request, 'ack_mode', '')).strip(),
        'scene_targets': resolved_scene_targets,
        'plan': {
            'plan_id': str(plan_id or make_plan_id()).strip(),
            'validation_status': str(validation_status or '').strip(),
            'failure_reason': str(failure_reason or '').strip(),
            'replan_hint': str(replan_hint or '').strip(),
            'retry_budget': _coerce_nonnegative_int(retry_budget),
            'scene_targets': resolved_scene_targets,
            'steps': normalize_plan_steps(list(steps or [])),
        },
    }


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class PlannerRequest:
    """Normalized planner ingress payload."""

    request_id: str
    user_text: str
    normalized_intents: tuple[str, ...]
    ack_text: str
    ack_mode: str
    scene_targets: tuple[str, ...]
    dialogue_context: tuple[str, ...]
    grounded_context: dict
    planner_mode: str

    @classmethod
    def from_payload(cls, payload) -> 'PlannerRequest':
        data = parse_json_object(payload)
        dialogue_context = data.get('dialogue_context', [])
        if isinstance(dialogue_context, str):
            dialogue_context = [dialogue_context]

        grounded_context = data.get('grounded_context', {})
        if not isinstance(grounded_context, dict):
            grounded_context = {}

        return cls(
            request_id=_first_non_empty(data.get('request_id', ''), make_plan_id('request')),
            user_text=str(data.get('user_text', '')).strip(),
            normalized_intents=tuple(coerce_str_list(data.get('normalized_intents', []))),
            ack_text=str(data.get('ack_text', '')).strip(),
            ack_mode=str(data.get('ack_mode', '')).strip(),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            dialogue_context=tuple(coerce_str_list(dialogue_context)),
            grounded_context=grounded_context,
            planner_mode=str(data.get('planner_mode', 'default')).strip() or 'default',
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


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class ExecutionFeedback:
    """Normalized planner/executor feedback payload."""

    plan_id: str
    status: str
    intent: str
    source: str
    reason: str
    validation_status: str
    replan_hint: str
    retry_budget: int
    scene_targets: tuple[str, ...]
    validation_errors: tuple[str, ...]
    step_id: str
    step_type: str
    step_name: str
    timestamp_sec: float

    @classmethod
    def from_payload(cls, payload) -> 'ExecutionFeedback':
        data = parse_json_object(payload)
        step_payload = data.get('step', {})
        if not isinstance(step_payload, dict):
            step_payload = {}
        return cls(
            plan_id=str(data.get('plan_id', '')).strip(),
            status=str(data.get('status', '')).strip().lower(),
            intent=str(data.get('intent', '')).strip(),
            source=str(data.get('source', '')).strip(),
            reason=str(data.get('reason', '')).strip(),
            validation_status=str(data.get('validation_status', '')).strip().lower(),
            replan_hint=str(data.get('replan_hint', '')).strip(),
            retry_budget=_coerce_nonnegative_int(data.get('retry_budget', 0)),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            validation_errors=tuple(coerce_str_list(data.get('validation_errors', []))),
            step_id=str(step_payload.get('id', '')).strip(),
            step_type=str(step_payload.get('type', '')).strip().lower(),
            step_name=str(step_payload.get('name', '')).strip().lower(),
            timestamp_sec=_coerce_float(data.get('timestamp_sec', 0.0)),
        )


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class EnrichedEntity:
    """One WME entity entry shared between planner and enrichment code."""

    entity_id: str
    label: str
    kb_class: str
    state: str
    score: float
    source: str
    last_seen_sec: float
    age_sec: float
    is_plan_relevant: bool
    risk_tags: tuple[str, ...]

    @classmethod
    def from_dict(cls, payload: dict) -> 'EnrichedEntity':
        return cls(
            entity_id=str(payload.get('entity_id', '')).strip(),
            label=str(payload.get('label', '')).strip(),
            kb_class=str(payload.get('kb_class', '')).strip(),
            state=str(payload.get('state', '')).strip().lower(),
            score=_coerce_float(payload.get('score', 0.0)),
            source=str(payload.get('source', '')).strip(),
            last_seen_sec=_coerce_float(payload.get('last_seen_sec', 0.0)),
            age_sec=_coerce_float(payload.get('age_sec', 0.0)),
            is_plan_relevant=bool(payload.get('is_plan_relevant', False)),
            risk_tags=tuple(coerce_str_list(payload.get('risk_tags', []))),
        )


@dataclass(**_FROZEN_DATACLASS_KWARGS)
class EnrichedSnapshot:
    """Normalized WME world-model snapshot."""

    observer: str
    backend: str
    active_plan_id: str
    execution_status: str
    execution_reason: str
    scene_targets: tuple[str, ...]
    entities: tuple[EnrichedEntity, ...]
    kb_rows: tuple[dict, ...]
    timestamp_sec: float

    @classmethod
    def from_payload(cls, payload) -> 'EnrichedSnapshot':
        data = parse_json_object(payload)
        raw_entities = data.get('entities', [])
        if not isinstance(raw_entities, list):
            raw_entities = []
        raw_kb_rows = data.get('kb_rows', [])
        if not isinstance(raw_kb_rows, list):
            raw_kb_rows = []
        return cls(
            observer=str(data.get('observer', '')).strip(),
            backend=str(data.get('backend', '')).strip(),
            active_plan_id=str(data.get('active_plan_id', '')).strip(),
            execution_status=str(data.get('execution_status', '')).strip().lower(),
            execution_reason=str(data.get('execution_reason', '')).strip(),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            entities=tuple(
                EnrichedEntity.from_dict(item)
                for item in raw_entities
                if isinstance(item, dict)
            ),
            kb_rows=tuple(item for item in raw_kb_rows if isinstance(item, dict)),
            timestamp_sec=_coerce_float(data.get('timestamp_sec', 0.0)),
        )


def build_world_model_text(
    snapshot: EnrichedSnapshot,
    *,
    max_chars: int = 2400,
    max_entities: int = 12,
    max_kb_rows: int = 8,
) -> str:
    """Render a bounded text block for prompt injection."""
    lines = [
        'Current world model context:',
        f'- observer: {snapshot.observer or "unknown"}',
        f'- backend: {snapshot.backend or "unknown"}',
    ]
    if snapshot.active_plan_id:
        lines.append(f'- active plan: {snapshot.active_plan_id} ({snapshot.execution_status or "unknown"})')
    if snapshot.execution_reason:
        lines.append(f'- execution note: {snapshot.execution_reason}')
    if snapshot.scene_targets:
        lines.append('- scene targets: ' + ', '.join(snapshot.scene_targets))

    if snapshot.entities:
        lines.append('- entities:')
        for entity in snapshot.entities[:max_entities]:
            parts = [
                entity.entity_id or entity.label or 'unknown_entity',
                entity.kb_class or entity.label or 'Unknown',
                f'state={entity.state or "unknown"}',
            ]
            if entity.is_plan_relevant:
                parts.append('plan-relevant')
            if entity.risk_tags:
                parts.append('risk=' + '|'.join(entity.risk_tags))
            lines.append('  - ' + ', '.join(parts))

    if snapshot.kb_rows:
        lines.append('- kb rows:')
        for row in snapshot.kb_rows[:max_kb_rows]:
            lines.append('  - ' + truncate_text(json.dumps(row, sort_keys=True), 180))

    return truncate_text('\n'.join(lines), max_chars)
