#!/usr/bin/env python3
"""Build a provenance-preserving Qwen3VL failure-position dataset."""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


MODEL = "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ"
IMAGE = "iiia:nao"

FIELDS = [
    "record_id",
    "run_id",
    "run_date",
    "runtime_image",
    "runtime_image_id",
    "source_file",
    "case_set",
    "case_name",
    "category",
    "failure_profile",
    "case_role",
    "status",
    "expected_outcome",
    "all_required_context",
    "planner_request_observed",
    "execution_feedback_observed",
    "failure_observed",
    "replan_observed",
    "terminal_observed",
    "speech_observed",
    "post_terminal_speech_observed",
    "fallback_total",
    "failure_site",
    "observed_failure_stage",
    "classification_confidence",
    "position_basis",
    "failure_step_names",
    "plan_event_trace",
    "reasons",
]


def _bool(value: Any) -> str:
    if value is None:
        return ""
    return "true" if bool(value) else "false"


def _int(value: Any) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0


def _status(case: dict[str, Any]) -> str:
    status = str(case.get("status") or (case.get("case_assessment") or {}).get("status") or "")
    return status or "not_scored"


def _plan_events(log_excerpt: str) -> dict[str, list[tuple[str, str]]]:
    plans: dict[str, list[tuple[str, str]]] = defaultdict(list)
    pattern = re.compile(
        r"goal_id=([^,\s]+),plan_id=([^,\s]+),event_type=([^,\s]+)(?:,step=([^\s]+))?"
    )
    for line in str(log_excerpt or "").splitlines():
        if "execution_feedback" not in line:
            continue
        match = pattern.search(line)
        if not match:
            continue
        plan_id = match.group(2)
        event = (match.group(3), match.group(4) or "")
        if not plans[plan_id] or plans[plan_id][-1] != event:
            plans[plan_id].append(event)
    return dict(plans)


def _trace_text(plans: dict[str, list[tuple[str, str]]]) -> str:
    chunks = []
    for plan_id, events in plans.items():
        encoded = " ".join(
            event + (f":{step}" if step else "") for event, step in events
        )
        chunks.append(f"{plan_id} [{encoded}]")
    return " | ".join(chunks)


def _failure_steps(plans: dict[str, list[tuple[str, str]]]) -> list[str]:
    steps: list[str] = []
    for events in plans.values():
        for event, step in events:
            if event == "step_failed" and step and step not in steps:
                steps.append(step)
    return steps


def _classify(
    *,
    source_name: str,
    case_name: str,
    profile: str,
    status: str,
    plans: dict[str, list[tuple[str, str]]],
) -> tuple[str, str, str, str, str, str]:
    """Return role, site, stage, confidence, basis, and source label."""
    source_label = "F13 semantic audit" if "F13" in source_name else "F14 targeted hardening"

    if profile == "fail_once_navigation" and case_name == "fake_deep_ordered_walk_report":
        return (
            "failure_position_target",
            "navigate_to",
            "beginning",
            "medium",
            "The targeted ordered-walk objective begins with navigation; the case reports an injected failure and missing recovery closure, but the retained excerpt does not expose a unique failed-step index.",
            source_label,
        )
    if profile == "fail_once_pick" and case_name == "skill_pick_phone_generic":
        return (
            "failure_position_target",
            "pick_object",
            "middle",
            "high",
            "The trace shows find_object succeeding before pick_object fails, followed by a successful retry plan.",
            source_label,
        )
    if profile == "delivery_blocked" and case_name == "fake_deep_grouped_work_table_delivery":
        return (
            "failure_position_target",
            "bring_object",
            "middle",
            "high",
            "The trace shows find/scan preparation before bring_object fails and replanning is observed.",
            source_label,
        )
    if profile == "recipient_missing" and case_name == "fake_deep_grouped_work_table_delivery":
        return (
            "failure_position_target",
            "bring_object_recipient_boundary",
            "end",
            "high",
            "The trace reaches navigation and object finding before recipient-bound delivery fails; the case then reports clarification despite complete fixture context.",
            source_label,
        )
    if profile in {"every_other", "random_seeded"} and case_name == "fake_deep_grouped_work_table_delivery":
        return (
            "failure_position_target",
            "mixed_fake_skill_policy",
            "middle_to_end",
            "medium",
            "The profile applies stochastic or alternating failure across the composite request; the artifact does not isolate one deterministic failed step.",
            source_label,
        )
    if case_name == "fake_deep_missing_object_recovery":
        return (
            "failure_position_target",
            "find_object",
            "beginning",
            "high",
            "The trace shows the first executable find_object step failing, followed by a recovery/replan path.",
            source_label,
        )
    if case_name == "fake_deep_gold_apple_multiturn" and status in {"fail", "degraded"} and not plans:
        return (
            "failure_position_target",
            "planner_admission_or_target_selection",
            "beginning",
            "high",
            "The planner publishes ask_clarification before executable plan steps, despite complete grounded fixture context.",
            source_label,
        )
    if case_name == "kb_mutation_add_red_cup":
        return (
            "failure_position_target",
            "kb_add_compiler_boundary",
            "beginning",
            "high",
            "The first planned kb_add step is rejected for non-RDF-style statements, then the harness records the expected recovery trajectory.",
            source_label,
        )
    if profile == "fail_once_navigation" and case_name == "fake_deep_iiia_kitchen_delivery":
        return (
            "failure_position_target",
            "navigate_to",
            "unknown",
            "low",
            "The navigation failure profile was configured, but this case's retained excerpt shows only a successful bring_object plan; applicability is not proven.",
            source_label,
        )
    if profile == "delivery_blocked" and case_name == "fake_deep_iiia_kitchen_delivery":
        return (
            "failure_position_target",
            "bring_object",
            "unknown",
            "low",
            "The delivery-blocked profile is configured, but the retained excerpt does not isolate a failed bring_object step for this case.",
            source_label,
        )
    if profile in {"fail_once_navigation", "delivery_blocked", "recipient_missing"} and status in {"degraded", "fail"}:
        return (
            "closure_or_recovery_observation",
            "post_failure_closure",
            "end",
            "medium",
            "The case assessment identifies missing recovery closure after terminal evidence; this is an end-of-plan speech/supervision observation.",
            source_label,
        )
    if status == "pass" and case_name in {
        "composite_walk_every_object_reports_main",
        "fake_deep_grouped_work_table_delivery",
        "fake_deep_iiia_kitchen_delivery",
        "fake_deep_gold_apple_followup",
    }:
        return (
            "baseline_control",
            "control_trajectory",
            "control",
            "high",
            "Successful control trace retained to compare execution and closure against the corresponding failure profile.",
            source_label,
        )
    return ("other_qwen3vl_evidence", "", "not_targeted", "high", "Not a selected failure-position target.", source_label)


def build(evidence_root: Path, output_dir: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    source_files: list[Path] = []
    for path in sorted(evidence_root.rglob("raw/*.json")):
        source_files.append(path)
        data = json.loads(path.read_text(encoding="utf-8"))
        profile = str((data.get("runtime_metadata") or {}).get("fake_policy_profile") or "none")
        source_name = path.relative_to(evidence_root).as_posix()
        run_id = "F13_semantic_audit" if "F13" in source_name else "F14_targeted_hardening"
        run_date = "2026-07-13" if run_id.startswith("F13") else "2026-07-14"
        metadata = {
            "runtime_image_id": "",
            "runtime_image": IMAGE,
        }
        for case_index, case in enumerate(data.get("cases", []) or [], start=1):
            phase = case.get("phase_observations") or {}
            assessment = case.get("case_assessment") or {}
            status = _status(case)
            plans = _plan_events(case.get("log_excerpt", ""))
            role, site, stage, confidence, basis, source_label = _classify(
                source_name=source_name,
                case_name=str(case.get("name", "")),
                profile=profile,
                status=status,
                plans=plans,
            )
            markers = phase.get("fallback_markers") or {}
            rows.append(
                {
                    "record_id": f"{run_id}:{path.stem}:{case_index}",
                    "run_id": run_id,
                    "run_date": run_date,
                    "runtime_image": metadata["runtime_image"],
                    "runtime_image_id": metadata["runtime_image_id"],
                    "source_file": source_name,
                    "case_set": data.get("case_set", ""),
                    "case_name": case.get("name", ""),
                    "category": case.get("category", ""),
                    "failure_profile": profile,
                    "case_role": role,
                    "status": status,
                    "expected_outcome": assessment.get("expected_outcome", ""),
                    "all_required_context": _bool(assessment.get("all_required_context")),
                    "planner_request_observed": _bool(phase.get("planner_request_observed")),
                    "execution_feedback_observed": _bool(phase.get("execution_feedback_observed")),
                    "failure_observed": _bool(phase.get("failure_observed")),
                    "replan_observed": _bool(phase.get("replan_observed")),
                    "terminal_observed": _bool(phase.get("terminal_observed")),
                    "speech_observed": _bool(phase.get("speech_observed")),
                    "post_terminal_speech_observed": _bool(phase.get("post_terminal_speech_observed")),
                    "fallback_total": _int(markers.get("total")),
                    "failure_site": site,
                    "observed_failure_stage": stage,
                    "classification_confidence": confidence,
                    "position_basis": basis,
                    "failure_step_names": ",".join(_failure_steps(plans)),
                    "plan_event_trace": _trace_text(plans),
                    "reasons": " | ".join(str(item) for item in assessment.get("reasons", []) or []),
                    "source_label": source_label,
                }
            )

    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "failure_position_dataset.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in FIELDS} for row in rows)
    with (output_dir / "failure_position_dataset.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")

    stage_rows = [row for row in rows if row["observed_failure_stage"] not in {"not_targeted", "control"}]
    summary = {
        "run_id": "qwen3vl-failure-position-addendum-2026-07-22",
        "model": MODEL,
        "runtime_image": IMAGE,
        "source_run_ids": ["F13_semantic_audit", "F14_targeted_hardening"],
        "source_file_count": len(source_files),
        "case_record_count": len(rows),
        "status_counts": dict(sorted(Counter(row["status"] for row in rows).items())),
        "failure_profile_counts": dict(sorted(Counter(row["failure_profile"] for row in rows).items())),
        "observed_failure_stage_counts": dict(sorted(Counter(row["observed_failure_stage"] for row in stage_rows).items())),
        "case_role_counts": dict(sorted(Counter(row["case_role"] for row in rows).items())),
        "targeted_stage_matrix": {
            stage: {
                "records": len([row for row in stage_rows if row["observed_failure_stage"] == stage]),
                "statuses": dict(sorted(Counter(row["status"] for row in stage_rows if row["observed_failure_stage"] == stage).items())),
                "case_names": sorted({row["case_name"] for row in stage_rows if row["observed_failure_stage"] == stage}),
            }
            for stage in sorted({row["observed_failure_stage"] for row in stage_rows})
        },
        "provenance_note": "F13 and F14 are diagnostic Qwen3VL runs. They span distinct runtime images and are not a single frozen qualification tuple.",
        "classification_note": "Beginning/middle/end labels describe the failure injection or observed closure position in the emitted plan, not physical robot execution proof. Unknown is retained when the artifact does not isolate a step.",
    }
    (output_dir / "failure_position_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")

    excerpts: list[str] = [
        "# Failure-position trace excerpts\n",
        "These excerpts are derived from the raw per-case `log_excerpt` fields. The raw JSON and stack logs remain authoritative.\n",
    ]
    for row in stage_rows:
        excerpts.append(f"## `{row['case_name']}`\n")
        excerpts.append(f"- Source: `{row['source_file']}`\n- Profile: `{row['failure_profile']}`\n- Stage: **{row['observed_failure_stage']}** ({row['classification_confidence']} confidence)\n- Site: `{row['failure_site']}`\n- Status: `{row['status']}`\n- Basis: {row['position_basis']}\n- Reasons: {row['reasons'] or 'none recorded'}\n- Trace: `{row['plan_event_trace'] or 'no plan events captured'}`\n")
    (output_dir / "failure_position_trace_excerpts.md").write_text("\n".join(excerpts), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_root", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    summary = build(args.evidence_root, args.output_dir)
    print(json.dumps({key: summary[key] for key in ("case_record_count", "status_counts", "observed_failure_stage_counts")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
