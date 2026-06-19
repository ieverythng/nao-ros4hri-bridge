#!/usr/bin/env python3
"""Summarize interaction-trace JSONL into thesis-ready validation metrics."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, asdict
from html import escape
import json
from pathlib import Path


@dataclass(frozen=True)
class TraceMetrics:
    trace_id: str
    event_count: int
    duration_sec: float
    planner_output_count: int
    feedback_count: int
    dialogue_act_count: int
    final_status: str
    completed: bool


def load_events(path: Path) -> list[dict]:
    events: list[dict] = []
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        clean_line = line.strip()
        if not clean_line:
            continue
        try:
            event = json.loads(clean_line)
        except json.JSONDecodeError as error:
            raise ValueError(f'{path}:{line_number}: invalid JSONL event') from error
        if isinstance(event, dict):
            events.append(event)
    return events


def summarize_events(events: list[dict]) -> list[TraceMetrics]:
    grouped: dict[str, list[dict]] = {}
    for event in events:
        trace_id = _event_trace_id(event)
        if trace_id:
            grouped.setdefault(trace_id, []).append(event)

    return [
        _summarize_trace(trace_id, trace_events)
        for trace_id, trace_events in sorted(grouped.items())
    ]


def _event_trace_id(event: dict) -> str:
    payload = event.get('payload', {})
    if isinstance(payload, dict):
        goal_id = _nested_goal_id(payload)
        if goal_id:
            return goal_id
    return str(event.get('trace_id', '')).strip()


def _nested_goal_id(payload: dict) -> str:
    goal_id = str(payload.get('goal_id', '')).strip()
    if goal_id:
        return goal_id
    for key in ('plan', 'data', 'payload', 'result_payload'):
        nested = payload.get(key)
        if isinstance(nested, dict):
            goal_id = _nested_goal_id(nested)
            if goal_id:
                return goal_id
    return ''


def _summarize_trace(trace_id: str, events: list[dict]) -> TraceMetrics:
    timestamps = [
        float(event.get('timestamp', 0.0) or 0.0)
        for event in events
        if float(event.get('timestamp', 0.0) or 0.0) > 0.0
    ]
    feedback = [
        event
        for event in events
        if str(event.get('channel', '')).strip() == '/planner/execution_feedback'
    ]
    final_payload = feedback[-1].get('payload', {}) if feedback else {}
    final_status = (
        str(final_payload.get('status', '')).strip()
        if isinstance(final_payload, dict)
        else ''
    )
    return TraceMetrics(
        trace_id=trace_id,
        event_count=len(events),
        duration_sec=round(max(timestamps) - min(timestamps), 3) if timestamps else 0.0,
        planner_output_count=_count_event_type(events, 'planner_output'),
        feedback_count=len(feedback),
        dialogue_act_count=_count_event_type(events, 'planner_dialogue_act'),
        final_status=final_status,
        completed=final_status == 'completed',
    )


def _count_event_type(events: list[dict], event_type: str) -> int:
    return sum(
        1
        for event in events
        if str(event.get('event_type', '')).strip() == event_type
    )


def write_outputs(metrics: list[TraceMetrics], output_prefix: Path) -> None:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(item) for item in metrics]
    output_prefix.with_suffix('.json').write_text(
        json.dumps({'traces': rows, 'aggregate': _aggregate(metrics)}, indent=2) + '\n',
        encoding='utf-8',
    )
    with output_prefix.with_suffix('.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(TraceMetrics.__dataclass_fields__))
        writer.writeheader()
        writer.writerows(rows)
    output_prefix.with_suffix('.html').write_text(_render_html(metrics), encoding='utf-8')


def _aggregate(metrics: list[TraceMetrics]) -> dict:
    correlated = [item for item in metrics if item.feedback_count > 0]
    total = len(correlated)
    completed = sum(1 for item in correlated if item.completed)
    durations = sorted(item.duration_sec for item in correlated)
    if not durations:
        median_duration = 0.0
    elif total % 2:
        median_duration = durations[total // 2]
    else:
        middle = total // 2
        median_duration = (durations[middle - 1] + durations[middle]) / 2.0
    return {
        'trace_count': total,
        'uncorrelated_trace_count': len(metrics) - total,
        'completed_count': completed,
        'completion_rate': round(completed / total, 3) if total else 0.0,
        'median_duration_sec': median_duration,
    }


def _render_html(metrics: list[TraceMetrics]) -> str:
    aggregate = _aggregate(metrics)
    rows = '\n'.join(
        '<tr>'
        f'<td>{escape(item.trace_id)}</td><td>{item.event_count}</td>'
        f'<td>{item.duration_sec:.3f}</td><td>{item.planner_output_count}</td>'
        f'<td>{item.feedback_count}</td><td>{item.dialogue_act_count}</td>'
        f'<td>{escape(item.final_status or "unknown")}</td>'
        '</tr>'
        for item in metrics
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>Validation Trace Metrics</title>
<style>
body{{font-family:Georgia,serif;background:#f5f0e6;color:#28231d;margin:2rem}}
.cards{{display:flex;gap:1rem;flex-wrap:wrap}}.card{{background:#fffaf0;padding:1rem 1.4rem;border:1px solid #d8cbb7;border-radius:12px}}
table{{width:100%;border-collapse:collapse;margin-top:1.5rem;background:#fffaf0}}th,td{{padding:.7rem;border-bottom:1px solid #ddd0bc;text-align:left}}th{{background:#e9ddca}}
</style></head><body><h1>Validation Trace Metrics</h1>
<div class="cards"><div class="card"><strong>Traces</strong><br>{aggregate['trace_count']}</div>
<div class="card"><strong>Completion rate</strong><br>{aggregate['completion_rate']:.1%}</div>
<div class="card"><strong>Median duration</strong><br>{aggregate['median_duration_sec']:.3f}s</div>
<div class="card"><strong>Uncorrelated groups</strong><br>{aggregate['uncorrelated_trace_count']}</div></div>
<table><thead><tr><th>Trace</th><th>Events</th><th>Duration (s)</th><th>Planner outputs</th><th>Feedback</th><th>Dialogue acts</th><th>Final status</th></tr></thead>
<tbody>{rows}</tbody></table></body></html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace_jsonl', type=Path)
    parser.add_argument(
        '--output-prefix',
        type=Path,
        default=Path('docs/artifacts/validation_trace_metrics'),
    )
    args = parser.parse_args()
    write_outputs(summarize_events(load_events(args.trace_jsonl)), args.output_prefix)


if __name__ == '__main__':
    main()
