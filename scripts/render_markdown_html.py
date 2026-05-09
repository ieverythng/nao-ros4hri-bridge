#!/usr/bin/env python3
"""Render Markdown into lightweight standalone HTML without external deps."""

from __future__ import annotations

import argparse
import html
import re
from pathlib import Path


def _escape(text: str) -> str:
    return html.escape(text, quote=True)


def _inline(text: str) -> str:
    escaped = _escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<code>\1</code>", escaped)
    escaped = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", escaped)
    escaped = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", escaped)
    escaped = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', escaped)
    return escaped


def _is_table_row(line: str) -> bool:
    return line.count("|") >= 2 and line.strip().startswith("|") and line.strip().endswith("|")


def _is_table_divider(line: str) -> bool:
    stripped = line.strip()
    if not _is_table_row(stripped):
        return False
    core = stripped.strip("|").replace(" ", "")
    return bool(core) and all(ch in "-:|" for ch in core)


def _table_cells(line: str) -> list[str]:
    return [cell.strip() for cell in line.strip().strip("|").split("|")]


def render_markdown(markdown_text: str) -> tuple[str, str]:
    lines = markdown_text.splitlines()
    output: list[str] = []
    title = "Document"

    in_code = False
    code_lang = ""
    code_lines: list[str] = []

    in_ul = False
    in_ol = False
    in_blockquote = False

    i = 0
    while i < len(lines):
        raw = lines[i]
        line = raw.rstrip("\n")
        stripped = line.strip()

        if stripped.startswith("```"):
            if in_code:
                code_text = _escape("\n".join(code_lines))
                cls = f' class="language-{_escape(code_lang)}"' if code_lang else ""
                output.append(f"<pre><code{cls}>{code_text}</code></pre>")
                in_code = False
                code_lang = ""
                code_lines = []
            else:
                in_code = True
                code_lang = stripped[3:].strip()
            i += 1
            continue

        if in_code:
            code_lines.append(line)
            i += 1
            continue

        if not stripped:
            if in_ul:
                output.append("</ul>")
                in_ul = False
            if in_ol:
                output.append("</ol>")
                in_ol = False
            if in_blockquote:
                output.append("</blockquote>")
                in_blockquote = False
            i += 1
            continue

        if _is_table_row(stripped):
            table_lines = [stripped]
            j = i + 1
            while j < len(lines) and _is_table_row(lines[j].strip()):
                table_lines.append(lines[j].strip())
                j += 1
            if len(table_lines) >= 2 and _is_table_divider(table_lines[1]):
                if in_ul:
                    output.append("</ul>")
                    in_ul = False
                if in_ol:
                    output.append("</ol>")
                    in_ol = False
                if in_blockquote:
                    output.append("</blockquote>")
                    in_blockquote = False
                header = _table_cells(table_lines[0])
                output.append("<table><thead><tr>")
                for cell in header:
                    output.append(f"<th>{_inline(cell)}</th>")
                output.append("</tr></thead><tbody>")
                for row in table_lines[2:]:
                    output.append("<tr>")
                    for cell in _table_cells(row):
                        output.append(f"<td>{_inline(cell)}</td>")
                    output.append("</tr>")
                output.append("</tbody></table>")
                i = j
                continue

        heading = re.match(r"^(#{1,6})\s+(.+)$", stripped)
        if heading:
            level = len(heading.group(1))
            text = heading.group(2).strip()
            if title == "Document" and level == 1:
                title = text
            if in_ul:
                output.append("</ul>")
                in_ul = False
            if in_ol:
                output.append("</ol>")
                in_ol = False
            if in_blockquote:
                output.append("</blockquote>")
                in_blockquote = False
            output.append(f"<h{level}>{_inline(text)}</h{level}>")
            i += 1
            continue

        if stripped in ("---", "***"):
            if in_ul:
                output.append("</ul>")
                in_ul = False
            if in_ol:
                output.append("</ol>")
                in_ol = False
            if in_blockquote:
                output.append("</blockquote>")
                in_blockquote = False
            output.append("<hr />")
            i += 1
            continue

        if stripped.startswith("> "):
            if in_ul:
                output.append("</ul>")
                in_ul = False
            if in_ol:
                output.append("</ol>")
                in_ol = False
            if not in_blockquote:
                output.append("<blockquote>")
                in_blockquote = True
            output.append(f"<p>{_inline(stripped[2:])}</p>")
            i += 1
            continue
        if in_blockquote:
            output.append("</blockquote>")
            in_blockquote = False

        unordered = re.match(r"^[-*]\s+(.+)$", stripped)
        if unordered:
            if in_ol:
                output.append("</ol>")
                in_ol = False
            if not in_ul:
                output.append("<ul>")
                in_ul = True
            output.append(f"<li>{_inline(unordered.group(1).strip())}</li>")
            i += 1
            continue

        ordered = re.match(r"^\d+\.\s+(.+)$", stripped)
        if ordered:
            if in_ul:
                output.append("</ul>")
                in_ul = False
            if not in_ol:
                output.append("<ol>")
                in_ol = True
            output.append(f"<li>{_inline(ordered.group(1).strip())}</li>")
            i += 1
            continue

        if in_ul:
            output.append("</ul>")
            in_ul = False
        if in_ol:
            output.append("</ol>")
            in_ol = False

        output.append(f"<p>{_inline(stripped)}</p>")
        i += 1

    if in_code:
        code_text = _escape("\n".join(code_lines))
        cls = f' class="language-{_escape(code_lang)}"' if code_lang else ""
        output.append(f"<pre><code{cls}>{code_text}</code></pre>")
    if in_ul:
        output.append("</ul>")
    if in_ol:
        output.append("</ol>")
    if in_blockquote:
        output.append("</blockquote>")

    return title, "\n".join(output)


def build_html(title: str, body: str) -> str:
    page_title = _escape(title)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{page_title}</title>
  <style>
    :root {{
      color-scheme: light dark;
      --fg: #1a1a1a;
      --muted: #5a5a5a;
      --bg: #f8f8f8;
      --card: #ffffff;
      --border: #d8d8d8;
      --accent: #0b5fff;
    }}
    @media (prefers-color-scheme: dark) {{
      :root {{
        --fg: #e8e8e8;
        --muted: #9e9e9e;
        --bg: #111111;
        --card: #1d1d1d;
        --border: #333333;
        --accent: #7fb4ff;
      }}
    }}
    body {{
      font-family: system-ui, -apple-system, "Segoe UI", Roboto, Ubuntu, sans-serif;
      max-width: 58rem;
      margin: 0 auto;
      padding: 1.5rem 1.25rem 3rem;
      line-height: 1.58;
      color: var(--fg);
      background: var(--bg);
    }}
    h1, h2, h3 {{ line-height: 1.25; }}
    h2 {{ border-bottom: 1px solid var(--border); padding-bottom: 0.3rem; }}
    pre {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 0.8rem;
      overflow-x: auto;
    }}
    code {{
      font-family: ui-monospace, "Cascadia Code", monospace;
      font-size: 0.9em;
      background: rgba(0, 0, 0, 0.06);
      padding: 0.12em 0.3em;
      border-radius: 4px;
    }}
    pre code {{
      background: transparent;
      padding: 0;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      background: var(--card);
      border: 1px solid var(--border);
    }}
    th, td {{
      border: 1px solid var(--border);
      padding: 0.45rem 0.55rem;
      text-align: left;
      vertical-align: top;
    }}
    th {{ background: rgba(0,0,0,0.04); }}
    blockquote {{
      border-left: 4px solid var(--accent);
      margin: 0.7rem 0;
      padding: 0.1rem 0.9rem;
      color: var(--muted);
    }}
    a {{ color: var(--accent); }}
  </style>
</head>
<body>
{body}
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input markdown file")
    parser.add_argument("output", help="output html file")
    parser.add_argument("--title", default="", help="optional title override")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    markdown_text = input_path.read_text(encoding="utf-8")
    title, body = render_markdown(markdown_text)
    if args.title.strip():
        title = args.title.strip()
    output_path.write_text(build_html(title, body), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
