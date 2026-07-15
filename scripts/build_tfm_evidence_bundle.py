#!/usr/bin/env python3
"""Build a durable, hash-addressed TFM runtime evidence bundle."""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
from collections import Counter
from pathlib import Path
import shutil
from typing import Any


QUESTIONNAIRE_SUFFIXES = {".json"}
VALID_CLASSIFICATIONS = {"final", "historical", "diagnostic", "source_only"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def case_status(case: dict[str, Any]) -> str:
    assessment = case.get("case_assessment") or case.get("assessment") or {}
    return str(case.get("status") or assessment.get("status") or "not_scored")


def case_rows(source_name: str, payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for case in payload.get("cases", []):
        if not isinstance(case, dict):
            continue
        observations = case.get("phase_observations") or {}
        assessment = case.get("case_assessment") or case.get("assessment") or {}
        fallback = observations.get("fallback_markers") or {}
        rows.append(
            {
                "source_artifact": source_name,
                "case_set": payload.get("case_set", ""),
                "case_id": case.get("name", ""),
                "category": case.get("category", ""),
                "status": case_status(case),
                "expected_outcome": assessment.get("expected_outcome", ""),
                "turn_injected": observations.get("turn_injected", ""),
                "route_observed": observations.get("route_observed", ""),
                "planner_request_observed": observations.get(
                    "planner_request_observed", ""
                ),
                "target_selection_observed": observations.get(
                    "target_selection_observed", ""
                ),
                "execution_feedback_observed": observations.get(
                    "execution_feedback_observed", ""
                ),
                "failure_observed": observations.get("failure_observed", ""),
                "replan_observed": observations.get("replan_observed", ""),
                "terminal_observed": observations.get("terminal_observed", ""),
                "speech_observed": observations.get("speech_observed", ""),
                "post_terminal_speech_observed": observations.get(
                    "post_terminal_speech_observed", ""
                ),
                "fallback_total": fallback.get("total", 0),
                "assessment_reasons": " | ".join(assessment.get("reasons", [])),
            }
        )
    return rows


def copy_raw_artifact(source: Path, destination_dir: Path) -> tuple[Path, bool]:
    destination_dir.mkdir(parents=True, exist_ok=True)
    if source.suffix not in {".log", ".txt"} and source.stat().st_size < 1024 * 1024:
        destination = destination_dir / source.name
        if source.suffix == ".json":
            content = source.read_bytes()
            destination.write_bytes(content.rstrip(b"\n") + b"\n")
        else:
            shutil.copy2(source, destination)
        return destination, False

    destination = destination_dir / f"{source.name}.gz"
    with (
        source.open("rb") as input_stream,
        destination.open("wb") as raw_output,
        gzip.GzipFile(
            filename="", mode="wb", fileobj=raw_output, mtime=0
        ) as output_stream,
    ):
        shutil.copyfileobj(input_stream, output_stream)
    return destination, True


def write_case_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def build_bundle(
    *,
    run_id: str,
    classification: str,
    artifacts: list[Path],
    output_root: Path,
    metadata: dict[str, Any] | None = None,
    notes: str = "",
) -> Path:
    if classification not in VALID_CLASSIFICATIONS:
        raise ValueError(f"unsupported classification: {classification}")
    if not artifacts:
        raise ValueError("at least one artifact is required")

    bundle_dir = output_root / run_id
    if bundle_dir.exists():
        raise FileExistsError(f"bundle already exists: {bundle_dir}")
    raw_dir = bundle_dir / "raw"
    bundle_dir.mkdir(parents=True)

    artifact_records: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    questionnaire_metadata: list[dict[str, Any]] = []
    for source in artifacts:
        source = source.resolve()
        if not source.is_file():
            raise FileNotFoundError(source)
        copied, compressed = copy_raw_artifact(source, raw_dir)
        record = {
            "name": source.name,
            "bundle_path": str(copied.relative_to(bundle_dir)),
            "compressed": compressed,
            "source_size_bytes": source.stat().st_size,
            "source_sha256": sha256_file(source),
            "bundle_sha256": sha256_file(copied),
        }
        artifact_records.append(record)
        if source.suffix in QUESTIONNAIRE_SUFFIXES:
            try:
                payload = load_json_object(source)
            except (json.JSONDecodeError, ValueError):
                continue
            if isinstance(payload.get("cases"), list):
                rows.extend(case_rows(source.name, payload))
                questionnaire_metadata.append(
                    {
                        "artifact": source.name,
                        "case_set": payload.get("case_set", ""),
                        "runtime_metadata": payload.get("runtime_metadata", {}),
                    }
                )

    status_counts = Counter(row["status"] for row in rows)
    aggregate = {
        "run_id": run_id,
        "classification": classification,
        "case_count": len(rows),
        "status_counts": dict(sorted(status_counts.items())),
        "weighted_case_rate": (
            (status_counts["pass"] + 0.5 * status_counts["degraded"]) / len(rows)
            if rows
            else None
        ),
        "note": (
            "This aggregate preserves recorded harness assessments. It does not "
            "upgrade trajectory evidence to semantic or physical proof."
        ),
    }
    manifest = {
        "schema_version": 1,
        "run_id": run_id,
        "classification": classification,
        "notes": notes,
        "metadata": metadata or {},
        "questionnaire_metadata": questionnaire_metadata,
        "artifacts": artifact_records,
    }
    (bundle_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (bundle_dir / "aggregate_metrics.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_case_csv(bundle_dir / "per_case_metrics.csv", rows)
    return bundle_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--classification", choices=sorted(VALID_CLASSIFICATIONS), required=True
    )
    parser.add_argument("--artifact", action="append", type=Path, required=True)
    parser.add_argument(
        "--output-root", type=Path, default=Path("docs/evaluation/runs")
    )
    parser.add_argument("--metadata-json", type=Path)
    parser.add_argument("--notes", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metadata = load_json_object(args.metadata_json) if args.metadata_json else {}
    bundle = build_bundle(
        run_id=args.run_id,
        classification=args.classification,
        artifacts=args.artifact,
        output_root=args.output_root,
        metadata=metadata,
        notes=args.notes,
    )
    print(bundle)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
