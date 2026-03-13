#!/usr/bin/env python3
"""Intent routing helpers for `nao_orchestrator`."""

from __future__ import annotations

import json

from hri_actions_msgs.msg import Intent


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


def parse_intent_data(raw_data: str) -> dict:
    """Parse JSON payloads carried by `hri_actions_msgs/Intent.data`."""
    if not raw_data:
        return {}
    try:
        parsed = json.loads(raw_data)
    except json.JSONDecodeError:
        return {'raw': raw_data}
    return parsed if isinstance(parsed, dict) else {'raw': raw_data}


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


def _first_non_empty(*values: str) -> str:
    for value in values:
        clean = str(value).strip()
        if clean:
            return clean
    return ''
