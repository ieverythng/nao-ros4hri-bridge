#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path


def load_meta(meta_path: Path) -> dict | None:
    if not meta_path.exists():
        return None
    return json.loads(meta_path.read_text())


def write_index_status(repo_root: Path, meta: dict | None) -> None:
    docs_dir = repo_root / "docs" / "knowledge"
    docs_dir.mkdir(parents=True, exist_ok=True)
    output = docs_dir / "INDEX_STATUS.md"
    repo_name = repo_root.name

    if meta is None:
        output.write_text(
            "\n".join(
                [
                    "# Knowledge Index Status",
                    "",
                    f"- Repo: `{repo_name}`",
                    "- Backend: GitNexus (not indexed yet)",
                    "- Local index: missing",
                    "- Next step: `tools/knowledge/index_repo.sh`",
                ]
            )
            + "\n"
        )
        return

    stats = meta.get("stats") or {}
    lines = [
        "# Knowledge Index Status",
        "",
        f"- Repo: `{repo_name}`",
        "- Backend: GitNexus `1.5.3`",
        "- Local index: present",
        f"- Indexed at: `{meta.get('indexedAt', 'unknown')}`",
        f"- Indexed commit: `{meta.get('lastCommit', 'unknown')}`",
        f"- Files: `{stats.get('files', '?')}`",
        f"- Nodes: `{stats.get('nodes', '?')}`",
        f"- Edges: `{stats.get('edges', '?')}`",
        f"- Communities: `{stats.get('communities', '?')}`",
        f"- Processes: `{stats.get('processes', '?')}`",
        f"- Embeddings: `{stats.get('embeddings', 0)}`",
        "",
        "Refresh commands:",
        "- `tools/knowledge/index_repo.sh`",
        "- `tools/knowledge/status.sh`",
        "- `tools/knowledge/generate_wiki.sh`",
    ]
    output.write_text("\n".join(lines) + "\n")


def sync_wiki(repo_root: Path) -> None:
    source_dir = repo_root / ".gitnexus" / "wiki"
    target_dir = repo_root / "docs" / "knowledge" / "wiki"
    target_dir.mkdir(parents=True, exist_ok=True)

    tracked_source = {path.name for path in source_dir.glob("*.md")} if source_dir.exists() else set()

    for path in list(target_dir.glob("*.md")):
        if path.name == "README.md":
            continue
        if path.name not in tracked_source:
            path.unlink()

    if not source_dir.exists():
        return

    for path in source_dir.glob("*.md"):
        shutil.copy2(path, target_dir / path.name)


def build_tree_from_paths(paths: list[str], repo_name: str) -> dict:
    tree: dict[str, dict] = {}
    root = {
        "repo": repo_name,
        "source": "git-ls-files fallback",
        "children": tree,
    }

    for rel_path in paths:
        cursor = tree
        parts = [part for part in rel_path.split("/") if part]

        for index, part in enumerate(parts):
            is_leaf = index == len(parts) - 1
            node = cursor.setdefault(
                part,
                {
                    "name": part,
                    "type": "file" if is_leaf else "directory",
                    "children": {},
                },
            )
            if is_leaf:
                node["type"] = "file"
                node.pop("children", None)
                break
            cursor = node.setdefault("children", {})

    def normalize(node: dict) -> dict:
        children = node.get("children")
        if not children:
            return {k: v for k, v in node.items() if k != "children"}
        normalized_children = [
            normalize(child) for _, child in sorted(children.items(), key=lambda item: item[0])
        ]
        normalized = {k: v for k, v in node.items() if k != "children"}
        normalized["children"] = normalized_children
        return normalized

    return normalize(root)


def write_module_tree(repo_root: Path) -> None:
    output = repo_root / "docs" / "knowledge" / "module_tree.json"
    source_dir = repo_root / ".gitnexus" / "wiki"
    module_tree = source_dir / "module_tree.json"

    if module_tree.exists():
        shutil.copy2(module_tree, output)
        return

    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    )
    repo_files = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    output.write_text(json.dumps(build_tree_from_paths(repo_files, repo_root.name), indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Sync GitNexus outputs into tracked docs.")
    parser.add_argument("--repo-root", required=True)
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    meta = load_meta(repo_root / ".gitnexus" / "meta.json")
    write_index_status(repo_root, meta)
    sync_wiki(repo_root)
    write_module_tree(repo_root)


if __name__ == "__main__":
    main()
