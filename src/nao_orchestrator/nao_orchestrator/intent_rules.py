#!/usr/bin/env python3
"""Intent routing helpers for `nao_orchestrator`."""

from __future__ import annotations

import json

from planner_common.contracts import PLAN_FAILURE_POLICIES
from planner_common.contracts import PLAN_STEP_TYPES
from planner_common.contracts import IntentLabels as Intent


_STANDARD_INTENTS = {
    Intent.BRING_OBJECT,
    Intent.GRAB_OBJECT,
    Intent.GREET,
    Intent.GUIDE,
    Intent.MOVE_TO,
    Intent.PERFORM_MOTION,
    Intent.PLACE_OBJECT,
    Intent.PRESENT_CONTENT,
    Intent.RAW_USER_INPUT,
    Intent.SAY,
    Intent.START_ACTIVITY,
    Intent.STOP_ACTIVITY,
    Intent.SUSPEND,
    Intent.WAKEUP,
}

_LEGACY_INTENT_MAP = {
    'greet': (Intent.GREET, {}),
    '__intent_greet__': (Intent.GREET, {}),
    '__intent_hello__': (Intent.GREET, {}),
    'identity': (Intent.SAY, {'object': 'I am your Nao orchestrator.'}),
    '__intent_identity__': (Intent.SAY, {'object': 'I am your Nao orchestrator.'}),
    'wellbeing': (
        Intent.SAY,
        {'object': 'I am doing well. Thank you for asking.'},
    ),
    '__intent_wellbeing__': (
        Intent.SAY,
        {'object': 'I am doing well. Thank you for asking.'},
    ),
    'help': (
        Intent.SAY,
        {
            'object': (
                'You can greet me, ask me to stand, sit, kneel, or move my head.'
            )
        },
    ),
    '__intent_help__': (
        Intent.SAY,
        {
            'object': (
                'You can greet me, ask me to stand, sit, kneel, or move my head.'
            )
        },
    ),
    'posture_stand': (Intent.PERFORM_MOTION, {'object': 'stand'}),
    '__intent_stand__': (Intent.PERFORM_MOTION, {'object': 'stand'}),
    'posture_sit': (Intent.PERFORM_MOTION, {'object': 'sit'}),
    '__intent_sit__': (Intent.PERFORM_MOTION, {'object': 'sit'}),
    'posture_kneel': (Intent.PERFORM_MOTION, {'object': 'kneel'}),
    '__intent_kneel__': (Intent.PERFORM_MOTION, {'object': 'kneel'}),
    'head_center': (Intent.PERFORM_MOTION, {'object': 'head_center'}),
    '__intent_head_center__': (Intent.PERFORM_MOTION, {'object': 'head_center'}),
    'head_look_left': (Intent.PERFORM_MOTION, {'object': 'head_look_left'}),
    '__intent_look_left__': (Intent.PERFORM_MOTION, {'object': 'head_look_left'}),
    'head_look_right': (Intent.PERFORM_MOTION, {'object': 'head_look_right'}),
    '__intent_look_right__': (Intent.PERFORM_MOTION, {'object': 'head_look_right'}),
    'head_look_up': (Intent.PERFORM_MOTION, {'object': 'head_look_up'}),
    '__intent_look_up__': (Intent.PERFORM_MOTION, {'object': 'head_look_up'}),
    'head_look_down': (Intent.PERFORM_MOTION, {'object': 'head_look_down'}),
    '__intent_look_down__': (Intent.PERFORM_MOTION, {'object': 'head_look_down'}),
    '__intent_say__': (Intent.SAY, {}),
    '__intent_perform_motion__': (Intent.PERFORM_MOTION, {}),
    'fallback': (Intent.RAW_USER_INPUT, {}),
}

_LOOK_AT_RESET_ALIASES = {
    'look_at_reset',
    'look_reset',
    'gaze_reset',
    'reset_gaze',
    'reset_look_at',
}

_REPLAY_MOTION_MAP = {
    'stand': 'stand',
    'standinit': 'standinit',
    'sit': 'sit',
    'kneel': 'kneel',
    'crouch': 'crouch',
}

_HEAD_MOTION_MAP = {
    'head_center': {'yaw': 0.0, 'pitch': 0.0, 'relative': False},
    'head_look_left': {'yaw': 0.45, 'pitch': 0.0, 'relative': False},
    'head_look_right': {'yaw': -0.45, 'pitch': 0.0, 'relative': False},
    'head_look_up': {'yaw': 0.0, 'pitch': -0.20, 'relative': False},
    'head_look_down': {'yaw': 0.0, 'pitch': 0.20, 'relative': False},
}

_POSTURE_TOPIC_FALLBACKS = {
    'stand': 'stand',
    'standinit': 'stand',
    'sit': 'sit',
    'kneel': 'kneel',
    'crouch': 'kneel',
}

_PLAN_STEP_TYPES_SET = frozenset(PLAN_STEP_TYPES)
_PLAN_FAILURE_POLICIES_SET = frozenset(PLAN_FAILURE_POLICIES)
_SUPPORTED_SKILL_PLAN_NAMES = {
    '',
    'perform_motion',
    'motion',
    'look_at',
    'scan',
}

_PEOPLE_SCAN_TARGET_KINDS = {
    'person',
    'people',
    'human',
    'humans',
}


# -----------------------------------------------------------------------------
# Input parsing helpers
# -----------------------------------------------------------------------------


def parse_intent_data(raw_data: str) -> dict:
    """Parse JSON payloads carried by `hri_actions_msgs/Intent.data`."""
    if not raw_data:
        return {}
    try:
        parsed = json.loads(raw_data)
    except json.JSONDecodeError:
        return {'raw': raw_data}
    return parsed if isinstance(parsed, dict) else {'raw': raw_data}


# -----------------------------------------------------------------------------
# Intent normalization
# -----------------------------------------------------------------------------


def normalize_legacy_intent(
    raw_payload: str,
    default_greeting: str,
) -> tuple[str, dict]:
    """Normalize old `/chatbot/intent` payloads into HRI intent form."""
    clean_payload = str(raw_payload).strip()
    if not clean_payload:
        return Intent.RAW_USER_INPUT, {}

    parsed_data = {}
    intent_name = clean_payload
    try:
        parsed = json.loads(clean_payload)
    except json.JSONDecodeError:
        parsed = None

    if isinstance(parsed, str):
        intent_name = parsed
    elif isinstance(parsed, dict):
        parsed_data = dict(parsed)
        intent_name = str(
            parsed_data.pop(
                'intent',
                parsed_data.pop(
                    'type',
                    parsed_data.pop('name', clean_payload),
                ),
            )
        ).strip()

    return normalize_incoming_intent(
        intent_name=intent_name,
        data=parsed_data,
        default_greeting=default_greeting,
    )


def normalize_incoming_intent(
    intent_name: str,
    data: dict,
    default_greeting: str,
) -> tuple[str, dict]:
    """Normalize custom or legacy intent labels into canonical HRI routing."""
    clean_name = str(intent_name).strip()
    payload = _clean_payload(data)
    if not clean_name:
        return Intent.RAW_USER_INPUT, payload

    if clean_name in _STANDARD_INTENTS:
        return clean_name, payload

    lower_name = clean_name.lower()
    mapped = _LEGACY_INTENT_MAP.get(lower_name)
    if mapped is not None:
        mapped_intent, mapped_payload = mapped
        merged_payload = dict(mapped_payload)
        merged_payload.update(payload)
        if mapped_intent == Intent.GREET:
            merged_payload.setdefault('suggested_response', default_greeting)
        return mapped_intent, merged_payload

    if lower_name in _LOOK_AT_RESET_ALIASES:
        payload.setdefault('object', 'look_at_reset')
        return Intent.PERFORM_MOTION, payload

    return clean_name, payload


# -----------------------------------------------------------------------------
# Downstream routing helpers
# -----------------------------------------------------------------------------


def resolve_say_text(
    intent_name: str,
    data: dict,
    default_greeting: str,
) -> str:
    """Resolve the text payload to send to `/nao/say`."""
    clean_intent = str(intent_name).strip()
    payload = _clean_payload(data)

    if clean_intent == Intent.GREET:
        return _first_non_empty(
            payload.get('suggested_response', ''),
            payload.get('object', ''),
            default_greeting,
        )

    if clean_intent == Intent.SAY:
        return _first_non_empty(
            payload.get('object', ''),
            payload.get('suggested_response', ''),
        )

    return ''


def resolve_ack_text(
    intent_name: str,
    data: dict,
    default_greeting: str,
) -> str:
    """Resolve acknowledgement text without forcing a duplicate speech dispatch."""
    payload = _clean_payload(data)
    explicit_ack = _first_non_empty(
        payload.get('ack_text', ''),
        payload.get('suggested_response', ''),
    )
    if explicit_ack:
        return explicit_ack
    return resolve_say_text(intent_name, payload, default_greeting)


def resolve_scan_result(
    step_args: dict,
    *,
    default_result_mode: str = 'success',
    default_summary: str = '',
) -> tuple[bool, str, dict]:
    """Normalize deterministic scan execution output for the orchestrator."""
    result_mode = str(
        step_args.get('result_mode', default_result_mode)
    ).strip().lower()
    target = str(step_args.get('target', '')).strip()
    target_kind = str(step_args.get('target_kind', target or 'scene')).strip() or 'scene'
    explicit_summary = str(step_args.get('summary', '')).strip()
    summary = explicit_summary or str(default_summary).strip()
    metadata = {
        'target': target,
        'target_kind': target_kind,
        'result_mode': result_mode or 'success',
    }
    if result_mode in ('fail', 'failed', 'failure'):
        return False, 'scan requested failure for %s' % target_kind, metadata
    if explicit_summary or target_kind == 'scene':
        return True, summary or 'scan completed', metadata
    target_label = target or target_kind
    return True, (
        'I completed the scan for %s, but no confirmed detection result was reported.'
        % target_label
    ), metadata


def is_people_scan_target(target_kind: str, target: str = '') -> bool:
    """Return whether a scan target is explicitly person-oriented."""
    clean_kind = str(target_kind or '').strip().lower()
    if clean_kind in _PEOPLE_SCAN_TARGET_KINDS:
        return True
    clean_target = str(target or '').strip().lower()
    return clean_target in _PEOPLE_SCAN_TARGET_KINDS


def summarize_people_detection(person_ids: list[str]) -> str:
    """Render a user-facing scan summary from tracked person identifiers."""
    clean_ids = [
        str(person_id).strip()
        for person_id in person_ids
        if str(person_id).strip()
    ]
    count = len(clean_ids)
    if count <= 0:
        return ''
    if count == 1:
        return 'I found one person (id: %s).' % clean_ids[0]
    return 'I found %d people (for example: %s).' % (count, clean_ids[0])


# -----------------------------------------------------------------------------
# Structured execution-plan helpers
# -----------------------------------------------------------------------------


def parse_execution_plan(data: dict) -> list[dict]:
    """Parse an optional structured execution plan embedded in intent data."""
    if not isinstance(data, dict):
        return []

    parsed_steps: list[dict] = []
    for index, step in enumerate(_plan_steps(data), start=1):
        normalized_step = _normalize_plan_step(step, index=index)
        if normalized_step is not None:
            parsed_steps.append(normalized_step)
    return parsed_steps


def parse_plan_envelope(data: dict) -> dict:
    """Normalize planner metadata carried alongside or inside `plan`."""
    if not isinstance(data, dict):
        return _empty_plan_envelope()

    parsed_plan_dict, _raw_plan = _parsed_plan_value(data)

    return {
        'goal_id': _first_non_empty(
            _plan_metadata_value(data, parsed_plan_dict, 'goal_id', 'goalId'),
            '',
        ),
        'plan_id': _first_non_empty(
            _plan_metadata_value(data, parsed_plan_dict, 'plan_id', 'id', 'planId'),
            '',
        ),
        'plan_version': _coerce_nonnegative_int(
            _plan_metadata_value(data, parsed_plan_dict, 'plan_version', 'version')
        ),
        'status': str(
            _plan_metadata_value(data, parsed_plan_dict, 'status')
            or ''
        ).strip().lower(),
        'validation_status': str(
            _plan_metadata_value(data, parsed_plan_dict, 'validation_status', 'status')
            or ''
        ).strip().lower(),
        'failure_reason': str(
            _plan_metadata_value(
                data,
                parsed_plan_dict,
                'failure_reason',
                'plan_failure_reason',
            )
            or ''
        ).strip(),
        'replan_hint': str(
            _plan_metadata_value(data, parsed_plan_dict, 'replan_hint') or ''
        ).strip(),
        'retry_budget': _coerce_nonnegative_int(
            _plan_metadata_value(data, parsed_plan_dict, 'retry_budget')
        ),
        'scene_targets': _coerce_str_list(
            _plan_metadata_value(
                data,
                parsed_plan_dict,
                'scene_targets',
                'expected_scene_targets',
            )
        ),
        'communication_policy': _normalize_communication_policy(
            _plan_metadata_value(data, parsed_plan_dict, 'communication_policy')
        ),
        'steps': parse_execution_plan(data),
        'has_explicit_plan': 'plan' in data,
    }


def validate_execution_plan(intent_name: str, data: dict) -> dict:
    """Validate parsed plan steps before the orchestrator executes them."""
    envelope = parse_plan_envelope(data)
    errors: list[str] = []
    validated_steps: list[dict] = []
    seen_step_ids: set[str] = set()

    if envelope['has_explicit_plan'] and not envelope['steps']:
        errors.append('plan contains no valid executable steps')

    for step in envelope['steps']:
        step_id = str(step.get('id', '')).strip()
        if step_id in seen_step_ids:
            errors.append(f'duplicate plan step id: {step_id}')
            continue
        seen_step_ids.add(step_id)

        error = _plan_step_validation_error(intent_name, step)
        if error:
            errors.append(f'{step_id}: {error}')
            continue

        validated_steps.append(step)

    envelope['steps'] = validated_steps
    envelope['errors'] = errors
    return envelope


def classify_motion_target(intent_name: str, data: dict) -> tuple[str, dict]:
    """Map canonical motion intents to the concrete NAO execution path."""
    clean_intent = str(intent_name).strip()
    payload = _clean_payload(data)
    obj = _first_non_empty(
        payload.get('object', ''),
        payload.get('motion_name', ''),
    ).lower()
    policy = str(payload.get('policy', '')).strip().lower()

    if clean_intent != Intent.PERFORM_MOTION and clean_intent:
        obj = obj or clean_intent.lower()

    if obj in _REPLAY_MOTION_MAP:
        return 'replay_motion', {'motion_name': _REPLAY_MOTION_MAP[obj]}

    if obj in _HEAD_MOTION_MAP:
        return 'head_motion', dict(_HEAD_MOTION_MAP[obj])

    if obj in _LOOK_AT_RESET_ALIASES or policy == 'reset':
        return 'look_at_reset', {'policy': 'reset'}

    return 'unsupported', {'object': obj, 'policy': policy}


def posture_topic_fallback_for_motion(motion_name: str) -> str:
    """Translate replay motions to the legacy posture command topic names."""
    return _POSTURE_TOPIC_FALLBACKS.get(str(motion_name).strip().lower(), '')


def make_intent_signature(intent_name: str, data: dict) -> str:
    """Build a stable dedupe key for recently processed intents."""
    payload = _clean_payload(data)
    payload.pop('ack_text', None)
    payload.pop('scene_targets', None)
    serialized = json.dumps(payload, sort_keys=True, separators=(',', ':'))
    return f'{str(intent_name).strip()}::{serialized}'


def _clean_payload(data: dict) -> dict:
    if not isinstance(data, dict):
        return {}
    return {
        str(key): value
        for key, value in data.items()
        if value not in (None, '', [])
    }


def _coerce_str_list(value) -> list[str]:
    if isinstance(value, str):
        return [value] if value.strip() else []
    if not isinstance(value, (list, tuple)):
        return []
    return [str(item).strip() for item in value if str(item).strip()]


def _coerce_nonnegative_int(value) -> int:
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def _plan_look_at_error(step_args: dict) -> str:
    policy = str(
        step_args.get('policy', step_args.get('object', ''))
    ).strip().lower()
    if policy in ('reset', 'look_at_reset'):
        return ''
    if _first_non_empty(
        step_args.get('target_frame', ''),
        step_args.get('frame_id', ''),
    ):
        return ''
    return 'look_at step is missing target_frame or reset policy'


def _first_non_empty(*values: str) -> str:
    for value in values:
        clean = str(value).strip()
        if clean:
            return clean
    return ''


def _empty_plan_envelope() -> dict:
    return {
        'goal_id': '',
        'plan_id': '',
        'plan_version': 0,
        'status': '',
        'validation_status': '',
        'failure_reason': '',
        'replan_hint': '',
        'retry_budget': 0,
        'scene_targets': [],
        'communication_policy': _normalize_communication_policy({}),
        'steps': [],
        'has_explicit_plan': False,
    }


def _parsed_plan_value(data: dict) -> tuple[dict, list]:
    raw_plan = data.get('plan', [])
    if isinstance(raw_plan, str):
        try:
            raw_plan = json.loads(raw_plan)
        except json.JSONDecodeError:
            raw_plan = []

    if isinstance(raw_plan, dict):
        raw_steps = raw_plan.get('steps', [])
        return raw_plan, raw_steps if isinstance(raw_steps, list) else []
    if isinstance(raw_plan, list):
        return {}, raw_plan
    return {}, []


def _plan_steps(data: dict) -> list[dict]:
    _parsed_plan_dict, raw_steps = _parsed_plan_value(data)
    return [step for step in raw_steps if isinstance(step, dict)]


def _plan_metadata_value(data: dict, plan_data: dict, *keys: str):
    for key in keys:
        if key in plan_data:
            return plan_data.get(key)
        if key in data:
            return data.get(key)
    return None


def _normalize_plan_step(step: dict, *, index: int) -> dict | None:
    step_type = str(step.get('type', '')).strip().lower()
    if step_type not in _PLAN_STEP_TYPES_SET:
        return None

    return {
        'id': _first_non_empty(
            step.get('id', ''),
            step.get('step_id', ''),
            f'step_{index}',
        ),
        'type': step_type,
        'name': str(step.get('name', '')).strip().lower(),
        'args': _clean_payload(step.get('args', {})),
        'requires': _coerce_str_list(
            step.get('requires', step.get('preconditions', []))
        ),
        'on_failure': _coerce_failure_policy(
            step.get('on_failure', step.get('failure_policy', 'fail'))
        ),
        'retry_budget': _coerce_nonnegative_int(
            step.get('retry_budget', step.get('retries', 0))
        ),
    }


def _coerce_failure_policy(value) -> str:
    clean_value = str(value).strip().lower()
    if clean_value in _PLAN_FAILURE_POLICIES_SET:
        return clean_value
    return 'fail'


def _normalize_communication_policy(value) -> dict:
    if not isinstance(value, dict):
        return {
            'emit_acknowledge': False,
            'emit_progress': False,
            'emit_completion': True,
            'emit_failure': True,
        }
    return {
        'emit_acknowledge': bool(value.get('emit_acknowledge', False)),
        'emit_progress': bool(value.get('emit_progress', False)),
        'emit_completion': bool(value.get('emit_completion', True)),
        'emit_failure': bool(value.get('emit_failure', True)),
    }


def _plan_step_validation_error(intent_name: str, step: dict) -> str:
    step_type = step.get('type', '')
    step_name = step.get('name', '')
    step_args = dict(step.get('args', {}))

    if step_type == 'say':
        if _first_non_empty(step_args.get('text', ''), step_args.get('object', '')):
            return ''
        return 'say step is missing text'

    if step_type == 'look_at':
        return _plan_look_at_error(step_args)

    if step_type != 'skill':
        return ''

    if step_name not in _SUPPORTED_SKILL_PLAN_NAMES:
        return f'unsupported skill step "{step_name}"'
    if step_name == 'look_at':
        return _plan_look_at_error(step_args)
    if step_name == 'scan':
        return ''

    route, _resolved_payload = classify_motion_target(
        intent_name or Intent.PERFORM_MOTION,
        step_args,
    )
    if route == 'unsupported':
        return 'unsupported motion payload %s' % json.dumps(
            step_args,
            sort_keys=True,
        )
    return ''
