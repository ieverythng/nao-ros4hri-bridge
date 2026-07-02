"""Planner dialogue relay and request-context helpers for nao_orchestrator."""

from __future__ import annotations

import json
from typing import Any, Callable

MAX_RELAYED_PLANNER_ACTS = 256
MAX_PLANNER_REQUEST_CONTEXTS = 64


def planner_dialogue_act_signature(payload: str) -> str:
    """Canonicalize one semantic planner event for relay deduplication."""
    try:
        parsed = json.loads(str(payload or '').strip())
    except (TypeError, ValueError, json.JSONDecodeError):
        return str(payload or '').strip()
    if not isinstance(parsed, dict):
        return str(payload or '').strip()
    try:
        plan_version = max(0, int(parsed.get('plan_version', 0) or 0))
    except (TypeError, ValueError):
        plan_version = 0
    signature = {
        'goal_id': str(parsed.get('goal_id', '')).strip(),
        'plan_id': str(parsed.get('plan_id', '')).strip(),
        'plan_version': plan_version,
        'act': str(parsed.get('act', '')).strip().lower(),
    }
    if signature['act'] == 'progress_update':
        signature.update(
            {
                'reason': str(parsed.get('reason', '')).strip(),
                'text_hint': str(parsed.get('text_hint', '')).strip(),
                'context': parsed.get('context', {}) if isinstance(parsed.get('context', {}), dict) else {},
            }
        )
    elif signature['act'] == 'ask_clarification':
        slots_needed = parsed.get('slots_needed', [])
        signature['slots_needed'] = sorted(
            str(slot).strip() for slot in slots_needed if str(slot).strip()
        ) if isinstance(slots_needed, list) else []
    return json.dumps(signature, sort_keys=True, separators=(',', ':'), ensure_ascii=True)


def remember_relayed_planner_act(
    signature: str,
    *,
    signatures: list[str],
    signature_set: set[str],
) -> None:
    """Bound the exact planner-act relay ledger while preserving recent history."""
    if not signature:
        return
    signatures.append(signature)
    signature_set.add(signature)
    if len(signatures) <= MAX_RELAYED_PLANNER_ACTS:
        return
    expired = signatures.pop(0)
    signature_set.discard(expired)


def relay_planner_dialogue_act(payload: str, *, publisher) -> None:
    """Publish planner dialogue acts on the orchestrator-owned relay topic."""
    if publisher is None:
        return
    from std_msgs.msg import String

    relay_msg = String()
    relay_msg.data = payload
    publisher.publish(relay_msg)


def remember_planner_request_context(
    goal_id: str,
    payload,
    *,
    parse_json_object: Callable[[Any], dict],
    request_context_by_goal: dict[str, dict],
    request_context_order: list[str],
) -> None:
    """Retain admitted request evidence for execution reports and replans."""
    clean_goal_id = str(goal_id or '').strip()
    request_context = parse_json_object(payload)
    if not clean_goal_id or not request_context:
        return
    if clean_goal_id in request_context_by_goal:
        request_context_order.remove(clean_goal_id)
    request_context_by_goal[clean_goal_id] = request_context
    request_context_order.append(clean_goal_id)
    while len(request_context_order) > MAX_PLANNER_REQUEST_CONTEXTS:
        expired_goal_id = request_context_order.pop(0)
        request_context_by_goal.pop(expired_goal_id, None)


def on_planner_dialogue_act(
    payload_text: str,
    *,
    parse_json_object: Callable[[Any], dict],
    relayed_signature_set: set[str],
    remember_relayed_planner_act_fn: Callable[[str], None],
    planner_gate,
    relay_planner_dialogue_act_fn: Callable[[str], None],
    stats: Any,
    logger,
) -> None:
    """Observe and relay planner acts through orchestrator-owned topic seam."""
    payload = parse_json_object(payload_text)
    act = str(payload.get('act', '')).strip().lower()
    if act == 'acknowledge':
        stats.duplicates_ignored += 1
        logger.info(
            'Suppressed contract-violating planner acknowledge; '
            'chatbot owns immediate acknowledgement'
        )
        return

    signature = planner_dialogue_act_signature(payload_text)
    if signature in relayed_signature_set:
        stats.duplicates_ignored += 1
        logger.warn('Ignored duplicate planner dialogue act')
        return
    remember_relayed_planner_act_fn(signature)

    active_goal_before = planner_gate.active_goal_id
    planner_gate.observe_dialogue_act(payload_text)
    active_goal_after = planner_gate.active_goal_id
    if active_goal_before and not active_goal_after:
        logger.info('Planner gate cleared by planner dialogue act | goal_id=%s' % active_goal_before)
    relay_planner_dialogue_act_fn(payload_text)
