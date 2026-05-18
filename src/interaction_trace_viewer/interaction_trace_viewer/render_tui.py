"""Terminal rendering helpers for interaction events."""

from __future__ import annotations

import json

from interaction_trace_viewer.trace_model import InteractionEvent


def format_event_line(event: InteractionEvent, *, verbose: bool = False) -> str:
    """Render one interaction event line for terminal output."""
    stamp = '%.3f' % float(event.timestamp)
    head = '[%s] %s | %s | %s' % (
        stamp,
        event.trace_id or '-',
        event.event_type,
        event.channel,
    )
    if not verbose:
        if event.summary:
            return head + '\n  ' + event.summary
        return head

    payload_text = json.dumps(event.payload, ensure_ascii=True, indent=2, sort_keys=True)
    raw_text = str(event.raw or '').strip()
    if raw_text:
        return head + '\n  summary: %s\n  payload:\n%s\n  raw: %s' % (
            event.summary,
            _indent(payload_text),
            raw_text,
        )
    return head + '\n  summary: %s\n  payload:\n%s' % (
        event.summary,
        _indent(payload_text),
    )


def _indent(text: str) -> str:
    return '\n'.join('    ' + line for line in text.splitlines())
