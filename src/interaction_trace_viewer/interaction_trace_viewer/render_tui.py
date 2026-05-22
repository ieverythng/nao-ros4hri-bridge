"""Terminal rendering helpers for interaction events."""

from __future__ import annotations

import json

from interaction_trace_viewer.trace_model import InteractionEvent


def format_event_line(event: InteractionEvent, *, verbose: bool = False) -> str:
    """Render one interaction event line for terminal output."""
    stamp = '%.3f' % float(event.timestamp)
    flow = _flow_hint(event.payload)
    source = str(event.source_node or event.payload.get('source', '')).strip()
    head = '[%s] %s | %s | %s' % (
        stamp,
        event.trace_id or '-',
        event.event_type,
        event.channel,
    )
    if source:
        head += ' | node=%s' % source
    if flow:
        head += ' | flow=%s' % flow
    if not verbose:
        if event.summary:
            return head + '\n  ' + event.summary
        return head

    payload_text = json.dumps(event.payload, ensure_ascii=True, indent=2, sort_keys=True)
    return head + '\n  summary: %s\n  payload:\n%s' % (
        event.summary,
        _indent(payload_text),
    )


def _indent(text: str) -> str:
    return '\n'.join('    ' + line for line in text.splitlines())


def _flow_hint(payload: dict) -> str:
    if not isinstance(payload, dict):
        return ''
    parts = []
    for key in ('goal_id', 'plan_id', 'step_id', 'event_type'):
        value = str(payload.get(key, '')).strip()
        if value:
            parts.append('%s=%s' % (key, value))
    step = payload.get('step', {})
    if isinstance(step, dict):
        step_name = str(step.get('name', '')).strip()
        if step_name:
            parts.append('step=%s' % step_name)
    route = str(payload.get('route', '')).strip()
    if route:
        parts.append('route=%s' % route)
    return ','.join(parts[:4])
