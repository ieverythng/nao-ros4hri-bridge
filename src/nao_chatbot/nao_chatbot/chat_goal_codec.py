#!/usr/bin/env python3
"""Goal/result codecs for communication_skills/Chat."""

from __future__ import annotations

import json

from nao_chatbot.chat_history import coerce_history


def parse_json_dict(payload: str) -> dict:
    """Parse JSON payload as dict; return empty dict on errors."""
    if not payload:
        return {}
    try:
        parsed = json.loads(payload)
    except json.JSONDecodeError:
        return {}
    if not isinstance(parsed, dict):
        return {}
    return parsed


def extract_json_object(payload: str) -> dict:
    """Extract first JSON object from payload string."""
    parsed = parse_json_dict(payload)
    if parsed:
        return parsed

    decoder = json.JSONDecoder()
    for start in range(len(payload)):
        if payload[start] != "{":
            continue
        try:
            maybe_obj, _ = decoder.raw_decode(payload[start:])
        except json.JSONDecodeError:
            continue
        if isinstance(maybe_obj, dict):
            return maybe_obj
    return {}


def coerce_user_intent(user_intent) -> dict:
    """Normalize user intent payload to canonical dictionary form."""
    if isinstance(user_intent, dict):
        cleaned = {}
        for key in ("type", "object", "recipient", "input", "goal"):
            value = str(user_intent.get(key, "")).strip()
            if value:
                cleaned[key] = value
        return cleaned
    if isinstance(user_intent, str) and user_intent.strip():
        return {"type": user_intent.strip()}
    return {}


def extract_canonical_goal(goal) -> tuple[str, list[str], str, str]:
    """Read canonical user message, history, user_id, and turn_id from Chat goal."""
    payload = parse_json_dict(goal.role.configuration)
    user_message = str(payload.get("user_message", "")).strip()
    if not user_message and goal.initial_input.strip():
        user_message = goal.initial_input.strip()
    if not user_message:
        user_message = str(payload.get("input", "")).strip()

    history_raw = payload.get("conversation_history", payload.get("history", []))
    history = coerce_history(history_raw)

    user_id = str(payload.get("user_id", "")).strip()
    if not user_id:
        user_id = str(goal.person_id).strip()
    if not user_id:
        user_id = "user1"

    turn_id = str(payload.get("turn_id", payload.get("trace_id", ""))).strip()
    if not turn_id:
        group_id = str(getattr(goal, "group_id", "")).strip()
        person_id = str(getattr(goal, "person_id", "")).strip()
        if group_id.startswith("turn:"):
            turn_id = group_id[5:].strip()
        elif person_id.startswith("turn:"):
            turn_id = person_id[5:].strip()
    if not turn_id:
        turn_id = "unknown"

    return user_message, history, user_id, turn_id


def canonical_result(
    success: bool,
    message: str,
    updated_history: list[str],
    assistant_response: str,
    error_code: int,
    intent: str,
    intent_source: str,
    intent_confidence: float,
    user_intent: dict,
    turn_id: str = "",
):
    """Build canonical Chat.Result payload used by mission controller."""
    from communication_skills.action import Chat

    result = Chat.Result()
    result.result.error_code = int(error_code)
    result.result.error_msg = message
    result.role_results = json.dumps(
        {
            "success": bool(success),
            "assistant_response": assistant_response,
            "verbal_ack": assistant_response,
            "updated_history": list(updated_history),
            "message": message,
            "intent": intent,
            "intent_source": intent_source,
            "intent_confidence": float(intent_confidence),
            "user_intent": dict(user_intent),
            "turn_id": str(turn_id).strip(),
        },
        separators=(",", ":"),
    )
    return result
