"""Static HTML renderer for recorded interaction traces."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path

from interaction_trace_viewer.trace_model import InteractionEvent
from interaction_trace_viewer.trace_model import load_events_from_jsonl


def render_events_html(events: list[InteractionEvent], *, title: str = 'Interaction Trace Report') -> str:
    event_types = sorted({str(event.event_type or '').strip() for event in events if str(event.event_type or '').strip()})
    trace_ids = sorted({str(event.trace_id or '').strip() for event in events if str(event.trace_id or '').strip()})
    cards: list[str] = []
    for event in events:
        payload_text = json.dumps(event.payload, ensure_ascii=False, indent=2, sort_keys=True)
        default_open = event.event_type in {'planner_request', 'planner_output', 'execution_feedback', 'planner_dialogue_act'}
        open_attr = ' open' if default_open else ''
        trace_id = str(event.trace_id or '-')
        source_node = str(event.source_node or '')
        searchable = ' '.join(
            [
                str(event.event_type or ''),
                str(event.channel or ''),
                trace_id,
                str(event.summary or ''),
                source_node,
                str(payload_text),
            ]
        ).lower()
        cards.append(
            '<article class="card" data-event-type="%s" data-trace-id="%s" data-source-node="%s" data-searchable="%s">'
            % (
                html.escape(str(event.event_type or '')),
                html.escape(trace_id),
                html.escape(source_node),
                html.escape(searchable),
            )
            + '<header><strong>%s</strong> <span>%s</span> <code>%s</code></header>'
            % (
                html.escape(event.event_type),
                html.escape('%.3f' % float(event.timestamp)),
                html.escape(trace_id),
            )
            + '<p><code>%s</code> <small>%s</small></p>' % (
                html.escape(event.channel),
                html.escape(source_node or '-'),
            )
            + '<p>%s</p>' % html.escape(event.summary)
            + '<details%s><summary>payload</summary><pre>%s</pre></details>'
            % (open_attr, html.escape(payload_text))
            + ('<details><summary>raw</summary><pre>%s</pre></details>' % html.escape(str(event.raw)) if event.raw else '')
            + '</article>'
        )

    return '''<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>{title}</title>
  <style>
    :root {{ --bg:#f7f3ea; --ink:#2b2118; --line:#dfd2bf; --card:#fffaf2; --accent:#c46a2b; }}
    body {{ margin:0; font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, sans-serif; background:var(--bg); color:var(--ink); }}
    main {{ max-width: 1160px; margin: 0 auto; padding: 28px 18px 56px; }}
    h1 {{ margin: 0 0 12px; }}
    .meta {{ margin: 0 0 24px; color:#5d4a38; }}
    .controls {{ display:grid; gap:10px; grid-template-columns: repeat(auto-fit, minmax(220px,1fr)); margin: 0 0 16px; padding: 10px; border:1px solid var(--line); border-radius:10px; background:#fff7ec; }}
    .controls label {{ display:flex; flex-direction:column; gap:4px; font-size:0.9rem; }}
    .controls input, .controls select {{ font: inherit; padding:6px 8px; border:1px solid var(--line); border-radius:8px; background:#fff; color:var(--ink); }}
    .grid {{ display:grid; gap:12px; }}
    .card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:10px 12px; }}
    .card header {{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; margin-bottom:8px; }}
    .card p {{ margin:6px 0; }}
    code {{ background:#efe5d6; padding:.08rem .22rem; border-radius:4px; }}
    pre {{ background:#2b2118; color:#fff7ea; border-radius:8px; padding:8px 10px; overflow-x:auto; }}
    details > summary {{ cursor:pointer; }}
    .hidden {{ display:none; }}
    .meta-inline {{ color:#5d4a38; font-size:0.9rem; }}
  </style>
</head>
<body>
  <main>
    <h1>{title}</h1>
    <p class="meta">Events: <span id="event-count-total">{count}</span> | Visible: <span id="event-count-visible">{count}</span></p>
    <section class="controls">
      <label>Search
        <input id="filter-text" type="text" placeholder="goal_id, skill, status, node, summary...">
      </label>
      <label>Event Type
        <select id="filter-event-type">
          <option value="">All</option>
          {event_type_options}
        </select>
      </label>
      <label>Trace
        <select id="filter-trace-id">
          <option value="">All</option>
          {trace_id_options}
        </select>
      </label>
      <label>Source Node
        <input id="filter-source-node" type="text" placeholder="e.g. nao_orchestrator">
      </label>
    </section>
    <section class="grid">
      {cards}
    </section>
  </main>
  <script>
    (() => {{
      const cards = Array.from(document.querySelectorAll('.card'));
      const filterText = document.getElementById('filter-text');
      const filterEventType = document.getElementById('filter-event-type');
      const filterTraceId = document.getElementById('filter-trace-id');
      const filterSourceNode = document.getElementById('filter-source-node');
      const visibleCount = document.getElementById('event-count-visible');

      const apply = () => {{
        const query = (filterText.value || '').trim().toLowerCase();
        const eventType = (filterEventType.value || '').trim();
        const traceId = (filterTraceId.value || '').trim();
        const sourceNode = (filterSourceNode.value || '').trim().toLowerCase();
        let visible = 0;
        for (const card of cards) {{
          const cardType = card.dataset.eventType || '';
          const cardTraceId = card.dataset.traceId || '';
          const cardSource = (card.dataset.sourceNode || '').toLowerCase();
          const searchable = card.dataset.searchable || '';
          const matchesType = !eventType || cardType === eventType;
          const matchesTrace = !traceId || cardTraceId === traceId;
          const matchesSource = !sourceNode || cardSource.includes(sourceNode);
          const matchesQuery = !query || searchable.includes(query);
          const show = matchesType && matchesTrace && matchesSource && matchesQuery;
          card.classList.toggle('hidden', !show);
          if (show) visible += 1;
        }}
        visibleCount.textContent = String(visible);
      }};

      filterText.addEventListener('input', apply);
      filterEventType.addEventListener('change', apply);
      filterTraceId.addEventListener('change', apply);
      filterSourceNode.addEventListener('input', apply);
      apply();
    }})();
  </script>
</body>
</html>
'''.format(
        title=html.escape(title),
        count=len(events),
        event_type_options=''.join(
            '<option value="{value}">{value}</option>'.format(value=html.escape(event_type))
            for event_type in event_types
        ),
        trace_id_options=''.join(
            '<option value="{value}">{value}</option>'.format(value=html.escape(trace_id))
            for trace_id in trace_ids
        ),
        cards='\n'.join(cards),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Render interaction trace JSONL to HTML')
    parser.add_argument('--input', required=True, help='Path to input JSONL trace file')
    parser.add_argument('--output', required=True, help='Path to output HTML report')
    parser.add_argument('--title', default='Interaction Trace Report', help='Report title')
    args = parser.parse_args(argv)

    events = load_events_from_jsonl(args.input)
    html_text = render_events_html(events, title=args.title)

    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
