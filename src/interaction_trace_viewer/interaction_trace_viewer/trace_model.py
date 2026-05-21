"""Trace models and persistence helpers for interaction observability."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace
import json
from pathlib import Path
import time


@dataclass(frozen=True)
class InteractionEvent:
    """Normalized runtime event used by trace viewer and dashboard paths."""

    timestamp: float
    trace_id: str | None
    source_node: str
    channel: str
    event_type: str
    ab_object_id: str | None
    ab_level: int | None
    summary: str
    payload: dict
    raw: str | None = None

    def to_dict(self) -> dict:
        return {
            'timestamp': float(self.timestamp),
            'trace_id': self.trace_id,
            'source_node': self.source_node,
            'channel': self.channel,
            'event_type': self.event_type,
            'ab_object_id': self.ab_object_id,
            'ab_level': self.ab_level,
            'summary': self.summary,
            'payload': self.payload,
            'raw': self.raw,
        }

    @classmethod
    def from_dict(cls, payload: dict) -> 'InteractionEvent':
        return cls(
            timestamp=float(payload.get('timestamp', 0.0)),
            trace_id=_as_optional_str(payload.get('trace_id')),
            source_node=str(payload.get('source_node', '')).strip(),
            channel=str(payload.get('channel', '')).strip(),
            event_type=str(payload.get('event_type', '')).strip(),
            ab_object_id=_as_optional_str(payload.get('ab_object_id')),
            ab_level=_as_optional_int(payload.get('ab_level')),
            summary=str(payload.get('summary', '')).strip(),
            payload=dict(payload.get('payload', {}) or {}),
            raw=_as_optional_str(payload.get('raw')),
        )


class TraceRecorder:
    """Assign trace ids and keep an ordered in-memory event list."""

    def __init__(self) -> None:
        self._events: list[InteractionEvent] = []
        self._current_trace_id = ''
        self._trace_seq = 0

    def add(self, event: InteractionEvent) -> InteractionEvent:
        assigned_trace_id = event.trace_id or self._resolve_trace_id(event)
        if not assigned_trace_id:
            assigned_trace_id = self._new_trace_id()
            self._current_trace_id = assigned_trace_id

        enriched = replace(event, trace_id=assigned_trace_id)
        self._events.append(enriched)

        if enriched.event_type in {'planner_dialogue_act', 'robot_speech'}:
            self._current_trace_id = ''
        return enriched

    def events(self) -> tuple[InteractionEvent, ...]:
        return tuple(self._events)

    def _resolve_trace_id(self, event: InteractionEvent) -> str:
        if event.event_type == 'user_utterance':
            self._current_trace_id = self._new_trace_id()
            return self._current_trace_id
        if event.event_type == 'planner_request' and not self._current_trace_id:
            self._current_trace_id = self._new_trace_id()
            return self._current_trace_id
        return self._current_trace_id

    def _new_trace_id(self) -> str:
        self._trace_seq += 1
        return 'trace_%04d' % self._trace_seq


class JsonlTraceWriter:
    """Append interaction events to JSONL for later report/export."""

    def __init__(self, output_dir: str, *, prefix: str = 'interaction_trace') -> None:
        clean_dir = Path(output_dir).expanduser()
        clean_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.path = clean_dir / ('%s_%s.jsonl' % (prefix, stamp))
        self._handle = self.path.open('a', encoding='utf-8')

    def write(self, event: InteractionEvent) -> None:
        self._handle.write(json.dumps(event.to_dict(), ensure_ascii=True) + '\n')
        self._handle.flush()

    def close(self) -> None:
        if not self._handle.closed:
            self._handle.close()


def load_events_from_jsonl(path: str) -> list[InteractionEvent]:
    events: list[InteractionEvent] = []
    with Path(path).expanduser().open('r', encoding='utf-8') as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            payload = json.loads(line)
            if not isinstance(payload, dict):
                continue
            events.append(InteractionEvent.from_dict(payload))
    return events


def _as_optional_str(value) -> str | None:
    text = str(value or '').strip()
    return text if text else None


def _as_optional_int(value) -> int | None:
    if value in (None, ''):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None
