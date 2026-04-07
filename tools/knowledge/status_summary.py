#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path


def build_summary(repo_root: Path) -> list[str]:
    meta_path = repo_root / ".gitnexus" / "meta.json"
    status_doc = repo_root / "docs" / "knowledge" / "INDEX_STATUS.md"
    wiki_dir = repo_root / "docs" / "knowledge" / "wiki"
    module_tree = repo_root / "docs" / "knowledge" / "module_tree.json"

    meta = json.loads(meta_path.read_text())
    stats = meta.get("stats") or {}

    lines = [
        "[knowledge] Local artifact summary",
        f"[knowledge] Indexed at: {meta.get('indexedAt', 'unknown')}",
        f"[knowledge] Indexed commit: {meta.get('lastCommit', 'unknown')}",
        (
            "[knowledge] Stats: "
            f"{stats.get('files', '?')} files, "
            f"{stats.get('nodes', '?')} nodes, "
            f"{stats.get('edges', '?')} edges, "
            f"{stats.get('processes', '?')} flows"
        ),
    ]

    if status_doc.exists():
        is_stale = status_doc.stat().st_mtime < meta_path.stat().st_mtime
        lines.append(f"[knowledge] INDEX_STATUS.md stale: {'yes' if is_stale else 'no'}")
    else:
        lines.append("[knowledge] INDEX_STATUS.md stale: missing")

    if wiki_dir.exists():
        wiki_pages = sorted(path for path in wiki_dir.glob("*.md") if path.name != "README.md")
        lines.append(f"[knowledge] Tracked wiki pages: {len(wiki_pages)}")
    else:
        lines.append("[knowledge] Tracked wiki pages: 0")

    lines.append(f"[knowledge] module_tree.json present: {'yes' if module_tree.exists() else 'no'}")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Print a short GitNexus artifact summary.")
    parser.add_argument("--repo-root", required=True)
    args = parser.parse_args()

    for line in build_summary(Path(args.repo_root).resolve()):
        print(line)


if __name__ == "__main__":
    main()
