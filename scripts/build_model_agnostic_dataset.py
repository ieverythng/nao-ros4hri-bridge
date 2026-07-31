#!/usr/bin/env python3
"""Build a provenance-preserving dataset from runtime questionnaire artifacts."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Any


FIELDS = [
    "record_id",
    "source_file",
    "run_group",
    "backend",
    "model",
    "case_set",
    "case_name",
    "category",
    "status",
    "expected_outcome",
    "all_required_context",
    "evidence_consistent",
    "route_observed",
    "planner_request_observed",
    "execution_feedback_observed",
    "clarification_observed",
    "failure_observed",
    "replan_observed",
    "fallback_total",
    "target_selection_count",
    "executed_skill_count",
    "spoken_text_count",
    "reasons",
    "attribution_class",
]


def _infer_backend_model(path: Path, data: dict[str, Any]) -> tuple[str, str]:
    metadata = data.get("runtime_metadata", {}) or {}
    chatbot = metadata.get("chatbot_generation", {}) or {}
    planner = metadata.get("planner_generation", {}) or {}
    model = str(chatbot.get("model") or planner.get("model") or "").strip()
    provider = str(chatbot.get("provider") or planner.get("provider") or "").strip()
    lowered = path.name.lower()
    if not model:
        if "nemotron" in lowered:
            model = "nemotron-3-super:cloud"
        elif "gemma4_31b" in lowered:
            model = "gemma4:31b-cloud"
        elif "gemma4" in lowered:
            model = "gemma4:cloud"
        elif "qwen35" in lowered:
            model = "cyankiwi/Qwen3.5-35B-A3B-AWQ-4bit"
    if not provider:
        provider = "ollama" if "ollama" in str(path).lower() else "openai_compatible"
    return provider, model


def _run_group(path: Path) -> str:
    for group in (
        "current_v34",
        "current_ollama_switch",
        "ablations",
        "historical_vllm",
        "historical_ollama",
    ):
        if group in path.parts:
            return group
    return "other"


def _bool(value: Any) -> str:
    if value is None:
        return ""
    return "true" if bool(value) else "false"


def _int(value: Any) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0


def _attribution(status: str, phase: dict[str, Any], reasons: list[str], backend: str) -> str:
    markers = phase.get("fallback_markers", {}) or {}
    if any(_int(markers.get(key)) for key in ("language_model_unreachable_speech", "llm_response_failed")):
        return "runtime_dependency"
    if phase.get("evidence_inconsistencies"):
        return "harness_observability"
    reason_text = " ".join(reasons).lower()
    if status in {"degraded", "fail", "failed"}:
        if any(token in reason_text for token in ("coverage", "selection", "malformed", "model", "planner")):
            return "model_or_backend_variance"
        if backend == "openai_compatible":
            return "model_or_backend_variance"
        return "stack_contract_or_runtime"
    if status == "pass":
        return "stack_coherent_success"
    return "unclassified"


def build(root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    inventory: list[dict[str, Any]] = []
    for path in sorted(root.rglob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        backend, model = _infer_backend_model(path, data)
        cases = data.get("cases", []) or []
        inventory.append(
            {
                "source_file": path.relative_to(root).as_posix(),
                "run_group": _run_group(path),
                "case_set": data.get("case_set", ""),
                "run_status": data.get("run_status", ""),
                "case_count": len(cases),
                "backend": backend,
                "model": model,
            }
        )
        for index, case in enumerate(cases):
            assessment = case.get("case_assessment", {}) or {}
            phase = case.get("phase_observations", {}) or {}
            reasons = [str(item) for item in assessment.get("reasons", []) or []]
            markers = phase.get("fallback_markers", {}) or {}
            status = case.get("status") or assessment.get("status", "")
            row = {
                "record_id": "%s#%d" % (path.stem, index + 1),
                "source_file": path.relative_to(root).as_posix(),
                "run_group": _run_group(path),
                "backend": backend,
                "model": model,
                "case_set": data.get("case_set", ""),
                "case_name": case.get("name", ""),
                "category": case.get("category", ""),
                "status": status,
                "expected_outcome": assessment.get("expected_outcome", ""),
                "all_required_context": _bool(assessment.get("all_required_context")),
                "evidence_consistent": _bool(phase.get("evidence_consistent")),
                "route_observed": _bool(phase.get("route_observed")),
                "planner_request_observed": _bool(phase.get("planner_request_observed")),
                "execution_feedback_observed": _bool(phase.get("execution_feedback_observed")),
                "clarification_observed": _bool(phase.get("clarification_observed")),
                "failure_observed": _bool(phase.get("failure_observed")),
                "replan_observed": _bool(phase.get("replan_observed")),
                "fallback_total": _int(markers.get("total")),
                "target_selection_count": len(phase.get("target_selections", []) or []),
                "executed_skill_count": len(phase.get("executed_skills", []) or []),
                "spoken_text_count": len(phase.get("spoken_texts", []) or []),
                "reasons": " | ".join(reasons),
                "attribution_class": _attribution(
                    str(status), phase, reasons, backend
                ),
            }
            rows.append(row)
    status_counts = Counter(str(row["status"]) for row in rows)
    model_counts = Counter(str(row["model"]) for row in rows)
    attribution_counts = Counter(str(row["attribution_class"]) for row in rows)
    summary = {
        "record_count": len(rows),
        "run_file_count": len(inventory),
        "status_counts": dict(sorted(status_counts.items())),
        "model_counts": dict(sorted(model_counts.items())),
        "attribution_counts": dict(sorted(attribution_counts.items())),
        "run_inventory": inventory,
    }
    return rows, summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Evidence directory containing JSON artifacts.")
    parser.add_argument("output_dir", type=Path, help="Directory for dataset.csv, dataset.jsonl, and summary.json.")
    args = parser.parse_args()
    rows, summary = build(args.root)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "dataset.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    with (args.output_dir / "dataset.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    (args.output_dir / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({key: summary[key] for key in ("record_count", "run_file_count", "status_counts", "model_counts", "attribution_counts")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
