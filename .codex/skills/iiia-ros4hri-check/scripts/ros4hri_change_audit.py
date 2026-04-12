#!/usr/bin/env python3
"""Summarize changed ROS packages and repo-specific guardrails."""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
SENSITIVE_PACKAGES = {
    "chatbot_llm": "nested repo / user-facing LLM package; prefer seam-focused changes",
    "dialogue_manager": "nested repo / dialogue owner; keep lifecycle and speaking ownership intact",
    "interaction_skills": "ROS4HRI-aligned upstream package; avoid broad local deltas",
    "communication_skills": "ROS4HRI-aligned upstream package; avoid broad local deltas",
    "motions_skills": "ROS4HRI-aligned upstream package; avoid broad local deltas",
    "std_skills": "ROS4HRI-aligned upstream package; avoid broad local deltas",
}
FIRST_PARTY_HINTS = {
    "planner_common": "shared planner contract layer",
    "planner_llm": "planner and supervisor runtime",
    "nao_orchestrator": "deterministic executor",
    "nao_chatbot": "launch and operator surfaces",
    "nao_scene_grounding": "detector-to-KB grounding bridge",
    "kb_skills": "KnowledgeCore transport boundary",
}


@dataclass
class PackageInfo:
    name: str
    root: pathlib.Path
    files: list[pathlib.Path]
    notes: list[str]
    package_type: str | None
    scaffold_basis: str | None
    has_lifecycle: bool


def _run_git_command(command: list[str]) -> list[pathlib.Path]:
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return [
        REPO_ROOT / line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    ]


def run_git_diff(mode: str) -> list[pathlib.Path]:
    if mode == "working":
        return _run_git_command(["git", "diff", "--name-only", "HEAD"])
    if mode == "staged":
        return _run_git_command(["git", "diff", "--name-only", "--cached"])

    combined: dict[str, pathlib.Path] = {}
    for path in _run_git_command(["git", "diff", "--name-only", "HEAD"]):
        combined[str(path)] = path
    for path in _run_git_command(["git", "diff", "--name-only", "--cached"]):
        combined[str(path)] = path
    return list(combined.values())


def find_package_root(path: pathlib.Path) -> pathlib.Path | None:
    current = path if path.is_dir() else path.parent
    for candidate in [current, *current.parents]:
        if candidate == REPO_ROOT.parent:
            break
        if (candidate / "package.xml").exists():
            return candidate
        if candidate == REPO_ROOT:
            break
    return None


def parse_package_name(package_xml: pathlib.Path) -> str:
    root = ET.parse(package_xml).getroot()
    name = root.findtext("name")
    if not name:
        raise ValueError(f"package.xml missing <name>: {package_xml}")
    return name.strip()


def read_readme_metadata(package_root: pathlib.Path) -> tuple[str | None, str | None]:
    readme = package_root / "README.md"
    if not readme.exists():
        return None, None
    package_type = None
    scaffold_basis = None
    for line in readme.read_text(encoding="utf-8").splitlines():
        lowered = line.lower().strip()
        if lowered.startswith("- package type:") and package_type is None:
            package_type = line.split(":", 1)[1].strip()
        if lowered.startswith("- scaffold basis:") and scaffold_basis is None:
            scaffold_basis = line.split(":", 1)[1].strip()
    return package_type, scaffold_basis


def detect_lifecycle(package_root: pathlib.Path) -> bool:
    launch_files = list(package_root.rglob("*.launch.py"))
    code_files = [p for p in package_root.rglob("*.py") if "/test/" not in str(p)]
    patterns = ["LifecycleNode", "rclpy.lifecycle", "lifecycle_msgs"]
    for file_path in [*launch_files, *code_files, package_root / "package.xml"]:
        if not file_path.exists():
            continue
        try:
            text = file_path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        if any(pattern in text for pattern in patterns):
            return True
    return False


def classify_package(name: str) -> list[str]:
    notes: list[str] = []
    if name in SENSITIVE_PACKAGES:
        notes.append(SENSITIVE_PACKAGES[name])
    if name in FIRST_PARTY_HINTS:
        notes.append(FIRST_PARTY_HINTS[name])
    if name.startswith("nao_"):
        notes.append("local robot-adapter package; keep ROS4HRI public seams above it")
    return notes


def collect_packages(paths: Iterable[pathlib.Path]) -> dict[str, PackageInfo]:
    packages: dict[str, PackageInfo] = {}
    for path in paths:
        if not path.exists():
            continue
        package_root = find_package_root(path)
        if package_root is None:
            continue
        package_name = parse_package_name(package_root / "package.xml")
        if package_name not in packages:
            package_type, scaffold_basis = read_readme_metadata(package_root)
            packages[package_name] = PackageInfo(
                name=package_name,
                root=package_root,
                files=[],
                notes=classify_package(package_name),
                package_type=package_type,
                scaffold_basis=scaffold_basis,
                has_lifecycle=detect_lifecycle(package_root),
            )
        packages[package_name].files.append(path)
    return packages


def relativize(path: pathlib.Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def print_report(packages: dict[str, PackageInfo]) -> int:
    if not packages:
        print("No ROS package changes detected.")
        return 0

    print("ROS4HRI change audit")
    print("====================")
    print()
    for package in sorted(packages.values(), key=lambda item: item.name):
        print(f"Package: {package.name}")
        print(f"  Root: {relativize(package.root)}")
        print(f"  Lifecycle signals: {'yes' if package.has_lifecycle else 'no obvious signal'}")
        if package.package_type:
            print(f"  README package type: {package.package_type}")
        if package.scaffold_basis:
            print(f"  README scaffold basis: {package.scaffold_basis}")
        if package.notes:
            for note in package.notes:
                print(f"  Note: {note}")
        for file_path in sorted(package.files):
            print(f"  Changed: {relativize(file_path)}")
        print()

    print("Next reads:")
    print("  - .codex/skills/iiia-ros4hri-check/references/repo-boundaries.md")
    print("  - .codex/skills/iiia-ros4hri-check/references/ros4hri-guardrails.md")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", help="explicit files to audit")
    parser.add_argument(
        "--files",
        nargs="*",
        default=None,
        help="explicit files to audit; equivalent to positional paths",
    )
    parser.add_argument(
        "--mode",
        choices=["working", "staged", "all"],
        default="working",
        help="git diff mode when files are not provided",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    explicit_paths = args.files if args.files is not None else args.paths
    if explicit_paths:
        paths = [REPO_ROOT / file_name for file_name in explicit_paths]
    else:
        paths = run_git_diff(args.mode)
    return print_report(collect_packages(paths))


if __name__ == "__main__":
    sys.exit(main())
