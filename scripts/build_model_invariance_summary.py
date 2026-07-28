#!/usr/bin/env python3
"""Build a comparative, thesis-facing summary from frozen model runs."""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


MODEL_INFO = {
    "gemma4_31b": {
        "model": "gemma4:31b-cloud",
        "label": "Gemma4 31B cloud",
        "provider": "ollama",
        "profile": "historical baseline, start_naoqi_driver=true",
        "startup_snapshot": "startup_snapshot.json",
        "comparison_role": "baseline",
    },
    "gemma4_cloud": {
        "model": "gemma4:cloud",
        "label": "Gemma4 cloud",
        "provider": "ollama",
        "profile": "controlled semantic cell, start_naoqi_driver=false",
        "startup_snapshot": "startup_snapshot.json",
        "comparison_role": "third model",
    },
    "nemotron_3_super": {
        "model": "nemotron-3-super:cloud",
        "label": "Nemotron 3 Super cloud",
        "provider": "ollama",
        "profile": "controlled semantic cell, start_naoqi_driver=false",
        "startup_snapshot": "startup_snapshot_controlled.json",
        "comparison_role": "second model",
    },
}

SUITE_ORDER = (
    "environment",
    "main",
    "kb_stress",
    "robustness",
    "fake_deep_all_success",
    "capability_extreme",
    "fake_deep_fail_once_navigation_targeted",
)


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _case_files(model_dir: Path) -> list[Path]:
    return sorted(
        path
        for path in model_dir.glob("*.json")
        if path.name not in {
            "final_snapshot.json",
            "startup_snapshot.json",
            "startup_snapshot_controlled.json",
            "startup_snapshot_final.json",
        }
    )


def _status_counts(cases: list[dict[str, Any]]) -> dict[str, int]:
    return dict(sorted(Counter(str(case.get("status", "")) for case in cases).items()))


def _scored(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [case for case in cases if case.get("status") in {"pass", "degraded", "fail"}]


def _rate(cases: list[dict[str, Any]]) -> dict[str, Any]:
    scored = _scored(cases)
    counts = Counter(str(case.get("status")) for case in scored)
    return {
        "pass": counts.get("pass", 0),
        "degraded": counts.get("degraded", 0),
        "fail": counts.get("fail", 0),
        "not_scored": sum(1 for case in cases if case.get("status") == "not_scored"),
        "unresolved": sum(1 for case in cases if not case.get("status")),
        "scored": len(scored),
        "strict_pass_rate": round(counts.get("pass", 0) / len(scored), 4) if scored else None,
    }


def _failure_reason(case: dict[str, Any]) -> list[str]:
    assessment = case.get("case_assessment", {}) or {}
    return [str(reason) for reason in assessment.get("reasons", []) or []]


def build(root: Path, output: Path) -> dict[str, Any]:
    models: list[dict[str, Any]] = []
    case_matrix: dict[str, dict[str, dict[str, Any]]] = defaultdict(dict)

    for directory_name, info in MODEL_INFO.items():
        model_dir = root / directory_name
        suite_payload: dict[str, Any] = {}
        all_cases: list[dict[str, Any]] = []
        for path in _case_files(model_dir):
            data = _load(path)
            cases = data.get("cases", []) or []
            all_cases.extend(cases)
            suite_name = path.stem
            summary = {
                "source_file": path.relative_to(root).as_posix(),
                "case_set": data.get("case_set", ""),
                "run_status": data.get("run_status", ""),
                "case_count": len(cases),
                "status_counts": _status_counts(cases),
                "rate": _rate(cases),
                "cases": [
                    {
                        "name": case.get("name", ""),
                        "status": case.get("status", ""),
                        "reasons": _failure_reason(case),
                        "executed_skills": (case.get("phase_observations", {}) or {}).get(
                            "executed_skills", []
                        ),
                        "replan_observed": (case.get("phase_observations", {}) or {}).get(
                            "replan_observed", False
                        ),
                    }
                    for case in cases
                ],
            }
            suite_payload[suite_name] = summary
            for case in cases:
                name = str(case.get("name", ""))
                if name:
                    case_matrix[name][info["model"]] = {
                        "suite": suite_name,
                        "status": case.get("status", ""),
                        "reasons": _failure_reason(case),
                    }

        snapshot_path = model_dir / str(info["startup_snapshot"])
        snapshot = _load(snapshot_path) if snapshot_path.exists() else {}
        preflight = (snapshot.get("derived", {}) or {}).get("preflight", {}) or {}
        models.append(
            {
                **info,
                "directory": directory_name,
                "startup_snapshot": snapshot_path.relative_to(root).as_posix(),
                "preflight": {
                    "status": preflight.get("status", "unknown"),
                    "llm_preflight_failures": preflight.get("llm_preflight_failures", {}),
                    "missing_nodes": preflight.get("missing_nodes", []),
                    "knowledge_core_ready": preflight.get("knowledge_core_ready"),
                    "lifecycle_states": preflight.get("lifecycle_states", {}),
                    "fallback_events": (snapshot.get("derived", {}) or {}).get(
                        "fallback_event_total_count", 0
                    ),
                },
                "suite_summary": suite_payload,
                "all_case_rate": _rate(all_cases),
            }
        )

    shared_failure_sets: dict[str, set[str]] = defaultdict(set)
    for model in models:
        for suite in model["suite_summary"].values():
            for case in suite["cases"]:
                if case["status"] in {"fail", "degraded"}:
                    shared_failure_sets[case["name"]].add(model["model"])

    shared_failures = [
        {
            "case_name": name,
            "models": sorted(model_names),
            "shared_by_all_three": len(model_names) == len(models),
        }
        for name, model_names in sorted(shared_failure_sets.items())
    ]

    output_payload = {
        "run_id": "model-invariance-e2e-2026-07-21",
        "date": "2026-07-21",
        "frozen_image": "iiia:nao-runtime-v34-final-frozen-review",
        "frozen_image_id": (root / "image_id.txt").read_text(encoding="utf-8").strip()
        if (root / "image_id.txt").exists()
        else "unknown",
        "vllm_probe": {
            "endpoint": "http://10.7.138.215:8004/v1/models",
            "status": "unavailable",
            "http_status": 0,
            "observation": "TCP connection refused or unavailable during final inventory probe",
            "semantic_cell": "not_scored",
        },
        "ollama_inventory": [
            "nemotron-3-super:cloud",
            "kimi-k2.6:cloud",
            "gemma4:31b-cloud",
            "gemma4:cloud",
            "minimax-m2.7:cloud",
            "gemini-3-flash-preview:latest",
            "glm-5.2:cloud",
            "deepseek-v4-flash:cloud",
            "qwen3-coder:480b-cloud",
            "kimi-k2.5:cloud",
            "qwen3.5:cloud",
        ],
        "selection": {
            "baseline": "gemma4:31b-cloud",
            "second": "nemotron-3-super:cloud",
            "third": "gemma4:cloud",
            "same_image": True,
            "same_case_families": True,
            "same_model_parameters": True,
            "controlled_semantic_profile": "start_naoqi_driver=false to remove the unavailable external NAOqi endpoint from semantic comparison",
            "caveat": "The Gemma4 31B historical baseline was also retained with start_naoqi_driver=true. Alternative cells used the controlled no-driver profile after the NAOqi connection timeout delayed lifecycle readiness.",
        },
        "suite_order": list(SUITE_ORDER),
        "models": models,
        "case_matrix": {name: values for name, values in sorted(case_matrix.items())},
        "shared_failure_matrix": shared_failures,
        "total_case_records": sum(model["all_case_rate"]["scored"] + model["all_case_rate"]["not_scored"] + model["all_case_rate"]["unresolved"] for model in models),
        "interpretation": {
            "deterministic_tests": "Source and harness tests remain separate from these live questionnaire records.",
            "live_status_semantics": "pass is scoreable success, degraded is scoreable but incomplete, fail is scoreable failure, not_scored is excluded from rates, and blank status is an unresolved global timeout.",
            "attribution_rule": "Shared failures across both Gemma variants are treated as stack or contract-sensitive until a discriminating probe proves otherwise. Nemotron-only failures are model or backend variance candidates unless evidence correlation identifies a harness issue.",
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(output_payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    return output_payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    payload = build(args.root, args.output)
    print(json.dumps({"run_id": payload["run_id"], "models": [model["model"] for model in payload["models"]], "shared_failure_count": len(payload["shared_failure_matrix"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
