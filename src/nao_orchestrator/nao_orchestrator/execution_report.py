"""Execution-report shaping helpers for nao_orchestrator.

These functions keep planner feedback and report-result fallback assembly out of
`orchestrator.py` while preserving the existing payload semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from nao_orchestrator.intent_rules import build_scan_result_payload, is_unresolved_report_template

MAX_EXECUTION_REPORT_STEPS = 8


@dataclass(slots=True, frozen=True)
class ExecutionReportResult:
    text: str = ''
    source: str = ''


def _first_non_empty_text(*values) -> str:
    for value in values:
        text = str(value or '').strip()
        if text:
            return text
    return ''


def _first_non_empty_value(data: dict, *keys: str) -> str:
    if not isinstance(data, dict):
        return ''
    return _first_non_empty_text(*(data.get(key, '') for key in keys))


def looks_like_machine_payload(text: str) -> bool:
    clean_text = str(text or '').strip()
    return clean_text.startswith(('{', '[', '```', '"{'))


def report_text_from_result_payload(result_payload: dict) -> str:
    """Resolve conservative report text from a prior live skill result payload."""
    if not isinstance(result_payload, dict):
        return ''

    report_text = _first_non_empty_value(
        result_payload,
        'summary_text',
        'result_summary',
        'message',
    )
    if report_text and not is_unresolved_report_template(report_text):
        return report_text

    skill_name = str(result_payload.get('skill', '')).strip().lower()
    if skill_name == 'scan' or any(key in result_payload for key in ('objects', 'people')):
        scan_payload = build_scan_result_payload(result_payload)
        report_text = str(scan_payload.get('summary_text', '')).strip()
        if report_text and not is_unresolved_report_template(report_text):
            return report_text

    target = _first_non_empty_value(result_payload, 'target', 'object', 'location')
    status = str(result_payload.get('status', '')).strip().lower()
    if target and status in ('succeeded', 'success', 'completed'):
        return 'I completed the task for %s.' % target
    return ''


def execution_step_record(
    step: dict,
    *,
    status: str,
    reason: str = '',
    result_summary: str = '',
    result_payload: dict | None = None,
) -> dict:
    """Build compact execution evidence for chatbot-authored reports."""
    return {
        'id': str(step.get('id', '')).strip(),
        'type': str(step.get('type', '')).strip().lower(),
        'name': str(step.get('name', '')).strip().lower(),
        'args': dict(step.get('args', {})) if isinstance(step.get('args', {}), dict) else {},
        'status': str(status or '').strip().lower(),
        'reason': str(reason or '').strip(),
        'result_summary': str(result_summary or '').strip(),
        'result_payload': dict(result_payload or {}),
    }


def report_text_from_execution_results(execution_results: list) -> str:
    if not isinstance(execution_results, list):
        return ''
    summaries = []
    for step in execution_results[-MAX_EXECUTION_REPORT_STEPS:]:
        if not isinstance(step, dict):
            continue
        if str(step.get('status', '')).strip().lower() != 'succeeded':
            continue
        summary = str(step.get('result_summary', '')).strip()
        if summary and not is_unresolved_report_template(summary) and summary not in summaries:
            summaries.append(summary)
    return ' '.join(summaries)


def build_execution_report_context(fallback_data: dict) -> dict:
    plan_context = fallback_data.get('plan_context', {})
    if not isinstance(plan_context, dict):
        plan_context = {}
    execution_results = fallback_data.get('execution_results', [])
    if not isinstance(execution_results, list):
        execution_results = []
    bounded_steps = [
        dict(step)
        for step in execution_results[-MAX_EXECUTION_REPORT_STEPS:]
        if isinstance(step, dict)
    ]
    plan_steps = fallback_data.get('plan_steps', [])
    if not isinstance(plan_steps, list):
        plan_steps = []
    current_step_index = int(fallback_data.get('current_step_index', -1) or -1)
    future_steps = [
        dict(step)
        for step in plan_steps[current_step_index + 1:]
        if isinstance(step, dict)
    ] if current_step_index >= 0 else []
    future_action_steps = [
        step for step in future_steps if str(step.get('name', '')).strip().lower() != 'report_result'
    ]
    report_role = 'intermediate' if future_action_steps else 'final'
    return {
        'goal_text': _first_non_empty_value(
            fallback_data,
            'goal_text',
            'goal',
            'task',
            'raw_input',
            'text',
        ),
        'requested_intents': [
            str(item).strip()
            for item in fallback_data.get('normalized_intents', [])
            if str(item).strip()
        ] if isinstance(fallback_data.get('normalized_intents', []), list) else [],
        'dialogue_context': [
            str(item).strip()
            for item in fallback_data.get('dialogue_context', [])
            if str(item).strip()
        ][-MAX_EXECUTION_REPORT_STEPS:]
        if isinstance(fallback_data.get('dialogue_context', []), list)
        else [],
        'scene_targets': list(plan_context.get('scene_targets', []))
        if isinstance(plan_context.get('scene_targets', []), list)
        else [],
        'grounded_context': dict(
            fallback_data.get('grounded_context', {})
            if isinstance(fallback_data.get('grounded_context', {}), dict)
            else {}
        ),
        'plan_id': str(plan_context.get('plan_id', '')).strip(),
        'plan_version': int(plan_context.get('plan_version', 0) or 0),
        'report_role': report_role,
        'future_steps': future_steps[-MAX_EXECUTION_REPORT_STEPS:],
        'steps': bounded_steps,
        'latest_result_summary': str(fallback_data.get('last_result_summary', '')).strip(),
        'latest_result_payload': dict(
            fallback_data.get('last_result_payload', {})
            if isinstance(fallback_data.get('last_result_payload', {}), dict)
            else {}
        ),
    }


def resolve_report_result_text(
    step_args: dict,
    fallback_data: dict,
    *,
    request_execution_report_text: Callable[[dict], ExecutionReportResult],
) -> tuple[str, ExecutionReportResult]:
    explicit_text = _first_non_empty_value(
        step_args,
        'summary_text',
        'result_summary',
        'text',
        'message',
        'utterance',
        'content',
        'suggested_response',
        'text_hint',
        'object',
    )
    report_context = build_execution_report_context(fallback_data)
    if explicit_text and not is_unresolved_report_template(explicit_text):
        report_context['requested_summary'] = explicit_text
    chatbot_result = request_execution_report_text(report_context)
    if chatbot_result.text:
        return chatbot_result.text, chatbot_result

    if explicit_text and not is_unresolved_report_template(explicit_text):
        return explicit_text, chatbot_result

    chain_text = report_text_from_execution_results(fallback_data.get('execution_results', []))
    if chain_text:
        return chain_text, chatbot_result

    fallback_text = _first_non_empty_value(
        fallback_data,
        'last_result_summary',
        'result_summary',
        'summary_text',
    )
    if fallback_text and not is_unresolved_report_template(fallback_text):
        return fallback_text, chatbot_result
    return report_text_from_result_payload(fallback_data.get('last_result_payload', {})), chatbot_result
