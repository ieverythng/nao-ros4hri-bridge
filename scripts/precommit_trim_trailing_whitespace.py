#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


def _trim_text(text: str, is_markdown: bool) -> tuple[str, bool]:
    changed = False
    output_lines: list[str] = []

    for line in text.splitlines(keepends=True):
        if line.endswith("\r\n"):
            body, ending = line[:-2], "\r\n"
        elif line.endswith("\n"):
            body, ending = line[:-1], "\n"
        else:
            body, ending = line, ""

        stripped = body.rstrip(" \t")
        if stripped != body:
            if is_markdown and body.endswith("  ") and not body.endswith("\t"):
                body = stripped + "  "
            else:
                body = stripped
            changed = True

        output_lines.append(body + ending)

    return "".join(output_lines), changed


def main(argv: list[str]) -> int:
    changed = False
    for name in argv[1:]:
        path = Path(name)
        if not path.is_file():
            continue

        raw = path.read_bytes()
        if b"\0" in raw:
            continue

        text = raw.decode("utf-8", errors="surrogateescape")
        updated, file_changed = _trim_text(text, path.suffix.lower() == ".md")
        if file_changed:
            path.write_text(updated, encoding="utf-8", errors="surrogateescape")
            print(f"trimmed trailing whitespace: {path}")
            changed = True

    return 1 if changed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
