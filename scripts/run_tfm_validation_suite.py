#!/usr/bin/env python3
"""TFM Fake Skill Validation Suite — CLI runner.

Runs controlled fake-skill scenarios through the validation engine,
collects metrics, and writes output bundles for TFM evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "src" / "fake_skills"))
sys.path.insert(0, str(REPO_ROOT / "src" / "interaction_trace_viewer"))

from tfm_validation_suite.validation_core import ValidationEngine
from tfm_validation_suite.test_cases import get_test_cases


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="TFM Fake Skill Validation Suite")
    p.add_argument("--output-dir", default=str(REPO_ROOT / "docs" / "evaluation" / "runs"))
    p.add_argument("--phase", default="all", choices=["phase1", "phase2", "phase3", "phase4", "phase5", "all"])
    p.add_argument("--repetitions", type=int, default=1)
    p.add_argument("--seeds", default="42")
    return p.parse_args()


def run_suite(args: argparse.Namespace) -> list[dict]:
    output_dir = Path(args.output_dir) / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = [int(s) for s in args.seeds.split(",")]
    cases = get_test_cases(phase=args.phase)

    all_metrics = []
    all_traces = []

    for rep_idx in range(args.repetitions):
        seed = seeds[rep_idx % len(seeds)]
        engine = ValidationEngine(output_dir=str(output_dir))

        for case in cases:
            result = engine.run_case(case)
            metrics = _case_metrics(case, result, seed, rep_idx)
            all_metrics.append(metrics)
            all_traces.extend(result.trace_events)

        engine.close()

    # Write outputs
    suite_config = {
        "suite": "tfm_fake_skill_validation",
        "phase": args.phase,
        "repetitions": args.repetitions,
        "seeds": seeds,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "cases_count": len(cases),
    }

    _write_config(output_dir, suite_config)
    _write_csv(output_dir, all_metrics)
    _write_aggregate(output_dir, all_metrics)
    _write_report_md(output_dir, all_metrics, suite_config)
    _write_report_html(output_dir, all_metrics, suite_config)
    _write_trace_jsonl(output_dir, all_traces)

    return all_metrics


def _case_metrics(case, result, seed: int, rep_idx: int) -> dict:
    passed_assertions = sum(1 for a in result.assertions if a["passed"])
    failed_assertions = len(result.assertions) - passed_assertions
    skill_statuses = [sr.get("status", "?") for sr in result.skill_results]
    final_status = skill_statuses[-1] if skill_statuses else "none"
    return {
        "case_id": case.case_id,
        "passed": result.passed,
        "duration_sec": round(result.duration_sec, 4),
        "assertions_passed": passed_assertions,
        "assertions_failed": failed_assertions,
        "skill_statuses": "|".join(skill_statuses),
        "final_status": final_status,
        "global_mode": case.global_mode,
        "scenario_id": case.scenario_id or "-",
        "seed": seed,
        "repetition": rep_idx,
    }


def _write_config(directory: Path, config: dict) -> None:
    lines = ["suite_config:\n"]
    for k, v in config.items():
        val = json.dumps(v) if isinstance(v, (list, dict)) else v
        lines.append(f"  {k}: {val}\n")
    (directory / "suite_config.yaml").write_text("".join(lines), encoding="utf-8")


def _write_csv(directory: Path, metrics: list[dict]) -> None:
    cols = ["case_id", "passed", "duration_sec", "assertions_passed", "assertions_failed", "skill_statuses", "final_status", "global_mode", "scenario_id", "seed", "repetition"]
    path = directory / "per_case_metrics.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(metrics)


def _write_aggregate(directory: Path, metrics: list[dict]) -> None:
    total = len(metrics)
    passed = sum(1 for m in metrics if m["passed"])
    agg = {
        "total_runs": total,
        "passed": passed,
        "failed": total - passed,
        "success_rate": round(passed / max(total, 1), 4),
        "mean_latency_sec": round(sum(m["duration_sec"] for m in metrics) / max(total, 1), 4),
        "phases_tested": list({m.get("global_mode") for m in metrics}),
    }
    (directory / "aggregate_metrics.json").write_text(json.dumps(agg, indent=2) + "\n", encoding="utf-8")


def _write_report_md(directory: Path, metrics: list[dict], config: dict) -> None:
    lines = [f"# TFM Validation Report\n"]
    lines.append(f"**Timestamp:** {config['timestamp']}\n")
    lines.append(f"**Phase:** {config['phase']}  ")
    lines.append(f"**Cases:** {config['cases_count']}  ")
    lines.append(f"**Repetitions:** {config['repetitions']}\n")

    total = len(metrics)
    passed = sum(1 for m in metrics if m["passed"])
    lines.append("## Summary\n")
    lines.append(f"| Metric | Value |")
    lines.append(f"|---|---|")
    lines.append(f"| Total runs | {total} |")
    lines.append(f"| Passed | {passed} |")
    lines.append(f"| Failed | {total - passed} |")
    lines.append(f"| Success rate | {passed / max(total, 1):.1%} |\n")

    lines.append("## Per-Case Results\n")
    lines.append("| Case | Status | Duration | Assertions | Skills | Mode |")
    lines.append("|---|---|---|---|---|---|")
    for m in metrics:
        status = "✅ PASS" if m["passed"] else "❌ FAIL"
        lines.append(f"| {m['case_id']} | {status} | {m['duration_sec']:.3f}s | {m['assertions_passed']}/{m['assertions_passed'] + m['assertions_failed']} | {m['skill_statuses']} | {m['global_mode']} |")

    (directory / "report.md").write_text("".join(lines) + "\n", encoding="utf-8")


def _write_report_html(directory: Path, metrics: list[dict], config: dict) -> None:
    total = len(metrics)
    passed = sum(1 for m in metrics if m["passed"])
    rows = ""
    for m in metrics:
        color = "#d4edda" if m["passed"] else "#f8d7da"
        status = "PASS" if m["passed"] else "FAIL"
        rows += f'<tr style="background:{color}"><td>{m["case_id"]}</td><td>{status}</td><td>{m["duration_sec"]:.3f}s</td><td>{m["assertions_passed"]}/{m["assertions_passed"]+m["assertions_failed"]}</td><td>{m["skill_statuses"]}</td><td>{m["global_mode"]}</td></tr>\n'

    html = f"""<!DOCTYPE html>
<html><head><title>TFM Validation Report</title>
<style>body{{font-family:system-ui,sans-serif;margin:2rem}}table{{border-collapse:collapse}}th,td{{border:1px solid #ccc;padding:6px 10px;text-align:left}}</h1></style>
</head><body>
<h1>TFM Validation Report</h1>
<p>Phase: {config['phase']} | Cases: {config['cases_count']} | Repetitions: {config['repetitions']}</p>
<p>Passed: {passed}/{total} ({passed/max(total,1):.1%})</p>
<table><thead><tr><th>Case</th><th>Status</th><th>Duration</th><th>Assertions</th><th>Skills</th><th>Mode</th></tr></thead>
<tbody>{rows}</tbody></table>
</body></html>"""
    (directory / "report.html").write_text(html, encoding="utf-8")


def _write_trace_jsonl(directory: Path, events: list[dict]) -> None:
    path = directory / "trace.jsonl"
    with open(path, "w", encoding="utf-8") as f:
        for event in events:
            f.write(json.dumps(event) + "\n")


def main() -> int:
    args = parse_args()
    metrics = run_suite(args)

    total = len(metrics)
    passed = sum(1 for m in metrics if m["passed"])
    print(f"\n{'='*60}")
    print(f"TFM Validation Suite — {args.phase}")
    print(f"{'='*60}")
    print(f"Total: {total} | Passed: {passed} | Failed: {total - passed} | Rate: {passed/max(total,1):.1%}")
    print(f"{'='*60}\n")

    return 0 if all(m["passed"] for m in metrics) else 1


if __name__ == "__main__":
    sys.exit(main())
