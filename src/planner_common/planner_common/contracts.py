"""Shared JSON contracts and normalization helpers for planner-related nodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
import sys
import time


DEFAULT_PLANNER_REQUEST_INTENT = 'planner_request'
PLAN_STEP_TYPES = ('noop', 'say', 'skill', 'look_at')
PLAN_FAILURE_POLICIES = (
    'fail',
    'continue',
    'replan',
    'clarify',
    'ask_user',
    'ignore',
)
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
    'world_model_snapshot': {},
    'world_model_text': '',
}
_DEFAULT_COMMUNICATION_POLICY = {
    'emit_acknowledge': False,
    'emit_progress': False,
    'emit_completion': True,
    'emit_failure': True,
}

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
    payload = dict(_DEFAULT_GROUNDED_CONTEXT)
    raw_payload = parse_json_object(value) if isinstance(value, str) else value
    if not isinstance(raw_payload, dict):
        return payload

    for key in ('knowledge_snapshot', 'scene_summary', 'world_model_snapshot'):
        item = raw_payload.get(key, {})
        payload[key] = item if isinstance(item, dict) else {}
    payload['world_model_text'] = str(raw_payload.get('world_model_text', '')).strip()
    return payload


def normalize_communication_policy(value) -> dict:
    """Normalize plan communication flags into a stable dict."""
    policy = dict(_DEFAULT_COMMUNICATION_POLICY)
    if not isinstance(value, dict):
        return policy

    for key in policy:
        if key in value:
            policy[key] = coerce_bool(value.get(key))
    return policy


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
                'id': _first_non_empty(
                    step.get('id', ''),
                    step.get('step_id', ''),
                    f'step_{index}',
                ),
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
    user_facing_reason: str = '',
    replan_hint: str = '',
    retry_budget: int = 0,
    scene_targets: list[str] | None = None,
    plan_id: str = '',
    goal_id: str = '',
    plan_version: int = 1,
    status: str = 'draft',
    communication_policy: dict | None = None,
) -> dict:
    """Build one planner result payload using the shared envelope shape."""
    resolved_scene_targets = list(scene_targets or getattr(request, 'scene_targets', []))
    resolved_goal_id = str(goal_id or getattr(request, 'goal_id', '')).strip()
    resolved_plan_id = str(plan_id or make_plan_id()).strip()
    resolved_ack_text = str(ack_text or getattr(request, 'ack_text', '')).strip()
    resolved_ack_mode = str(ack_mode or getattr(request, 'ack_mode', '')).strip()
    resolved_policy = normalize_communication_policy(communication_policy)

    return {
        'goal_id': resolved_goal_id,
        'ack_text': resolved_ack_text,
        'ack_mode': resolved_ack_mode,
        'user_facing_reason': str(user_facing_reason or '').strip(),
        'scene_targets': resolved_scene_targets,
        'grounded_context': normalize_grounded_context(
            getattr(request, 'grounded_context', {})
        ),
        'plan': {
            'goal_id': resolved_goal_id,
            'plan_id': resolved_plan_id,
            'plan_version': max(1, int(plan_version or 1)),
            'status': str(status or '').strip().lower() or 'draft',
            'validation_status': str(validation_status or '').strip().lower(),
            'failure_reason': str(failure_reason or '').strip(),
            'user_facing_reason': str(user_facing_reason or '').strip(),
            'replan_hint': str(replan_hint or '').strip(),
            'retry_budget': _coerce_nonnegative_int(retry_budget),
            'scene_targets': resolved_scene_targets,
            'communication_policy': resolved_policy,
            'steps': normalize_plan_steps(list(steps or [])),
        },
    }


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
) -> dict:
    """Build one normalized planner feedback payload."""
    resolved_step = step if isinstance(step, dict) else None
    resolved_retry_budget = _coerce_nonnegative_int(plan_context.get('retry_budget', 0))
    if resolved_step is not None:
        resolved_retry_budget = _coerce_nonnegative_int(
            resolved_step.get('retry_budget', resolved_retry_budget)
        )

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
    ack_text: str
    ack_mode: str
    scene_targets: tuple[str, ...]
    dialogue_context: tuple[str, ...]
    requested_plan: tuple[dict, ...]
    grounded_context: dict
    planner_mode: str
    interaction_mode: str
    dialogue_turn_id: str

    @classmethod
    def from_payload(cls, payload) -> 'PlannerRequest':
        data = parse_json_object(payload)
        dialogue_context = data.get('dialogue_context', [])
        if isinstance(dialogue_context, str):
            dialogue_context = [dialogue_context]

        request_id = _first_non_empty(data.get('request_id', ''), make_runtime_id('request'))
        goal_id = _first_non_empty(data.get('goal_id', ''), make_goal_id())
        requested_plan = tuple(
            normalize_plan_steps(
                data.get('requested_plan', data.get('plan', []))
            )
        )

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
            ack_text=str(data.get('ack_text', '')).strip(),
            ack_mode=str(data.get('ack_mode', '')).strip(),
            scene_targets=tuple(coerce_str_list(data.get('scene_targets', []))),
            dialogue_context=tuple(coerce_str_list(dialogue_context)),
            requested_plan=requested_plan,
            grounded_context=normalize_grounded_context(data.get('grounded_context', {})),
            planner_mode=str(data.get('planner_mode', 'default')).strip() or 'default',
            interaction_mode=str(data.get('interaction_mode', '')).strip() or 'default',
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


def _default_feedback_event_type(status: str) -> str:
    mapping = {
        'accepted': 'plan_accepted',
        'running': 'step_started',
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

    @classmethod
    def from_payload(cls, payload) -> 'ExecutionFeedback':
        data = parse_json_object(payload)
        step_payload = data.get('step', {})
        if not isinstance(step_payload, dict):
            step_payload = {}
        step_failure_policy = step_payload.get('on_failure', step_payload.get('failure_policy', ''))
        clean_step_failure_policy = str(step_failure_policy or '').strip().lower()
        status = str(data.get('status', '')).strip().lower()
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
        lines.append(
            f'- active plan: {snapshot.active_plan_id} ({snapshot.execution_status or "unknown"})'
        )
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
