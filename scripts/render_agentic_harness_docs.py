#!/usr/bin/env python3
"""Render the canonical Agentic Harness Markdown files with the shared web theme."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RENDERER = ROOT / "scripts" / "render_markdown_html.py"
DOCS = (
    "universal_agentic_harness_masterplan",
    "universal_agentic_harness_foundation",
    "neural_workbench_adaptive_ab_harness",
)
THEME_LINKS = (
    '  <link rel="stylesheet" href="assets/harness-theme.css" />\n'
    '  <script defer src="assets/harness-theme.js"></script>\n'
)


def render(name: str) -> None:
    docs_dir = ROOT / "docs" / "agentic_harness"
    markdown_path = docs_dir / f"{name}.md"
    html_path = docs_dir / f"{name}.html"
    subprocess.run(
        [sys.executable, str(RENDERER), str(markdown_path), str(html_path)],
        check=True,
    )
    html = html_path.read_text(encoding="utf-8")
    html_path.write_text(html.replace("</head>", THEME_LINKS + "</head>"), encoding="utf-8")


def main() -> int:
    for name in DOCS:
        render(name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
