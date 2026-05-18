"""Static HTML renderer for recorded interaction traces."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path

from interaction_trace_viewer.trace_model import InteractionEvent
from interaction_trace_viewer.trace_model import load_events_from_jsonl


def render_events_html(events: list[InteractionEvent], *, title: str = 'Interaction Trace Report') -> str:
    cards: list[str] = []
    for event in events:
        payload_text = json.dumps(event.payload, ensure_ascii=False, indent=2, sort_keys=True)
        cards.append(
            '<article class="card">'
            + '<header><strong>%s</strong> <span>%s</span> <code>%s</code></header>'
            % (
                html.escape(event.event_type),
                html.escape('%.3f' % float(event.timestamp)),
                html.escape(event.trace_id or '-'),
            )
            + '<p><code>%s</code></p>' % html.escape(event.channel)
            + '<p>%s</p>' % html.escape(event.summary)
            + '<details open><summary>payload</summary><pre>%s</pre></details>'
            % html.escape(payload_text)
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
    .grid {{ display:grid; gap:12px; }}
    .card {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:10px 12px; }}
    .card header {{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; margin-bottom:8px; }}
    .card p {{ margin:6px 0; }}
    code {{ background:#efe5d6; padding:.08rem .22rem; border-radius:4px; }}
    pre {{ background:#2b2118; color:#fff7ea; border-radius:8px; padding:8px 10px; overflow-x:auto; }}
    details > summary {{ cursor:pointer; }}
  </style>
</head>
<body>
  <main>
    <h1>{title}</h1>
    <p class="meta">Events: {count}</p>
    <section class="grid">
      {cards}
    </section>
  </main>
</body>
</html>
'''.format(
        title=html.escape(title),
        count=len(events),
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
