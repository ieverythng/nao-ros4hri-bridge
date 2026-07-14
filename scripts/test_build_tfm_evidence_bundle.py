import csv
import importlib.util
import json
from pathlib import Path
import sys


def _load_module():
    path = Path(__file__).with_name("build_tfm_evidence_bundle.py")
    spec = importlib.util.spec_from_file_location("build_tfm_evidence_bundle", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_bundle_hashes_raw_artifact_and_preserves_case_assessment(tmp_path):
    module = _load_module()
    artifact = tmp_path / "questionnaire.json"
    artifact.write_text(
        json.dumps(
            {
                "case_set": "main",
                "runtime_metadata": {"chatbot_turn_pipeline_mode": "response_first"},
                "cases": [
                    {
                        "name": "ordered_objects",
                        "category": "composite",
                        "status": "fail",
                        "case_assessment": {
                            "status": "fail",
                            "expected_outcome": "execute_no_clarification",
                            "reasons": ["wrong selected set"],
                        },
                        "phase_observations": {
                            "turn_injected": True,
                            "route_observed": True,
                            "target_selection_observed": True,
                            "fallback_markers": {"total": 2},
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    bundle = module.build_bundle(
        run_id="F-test",
        classification="diagnostic",
        artifacts=[artifact],
        output_root=tmp_path / "runs",
    )

    manifest = json.loads((bundle / "manifest.json").read_text())
    aggregate = json.loads((bundle / "aggregate_metrics.json").read_text())
    with (bundle / "per_case_metrics.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))

    assert manifest["artifacts"][0]["source_sha256"] == module.sha256_file(artifact)
    assert aggregate["status_counts"] == {"fail": 1}
    assert aggregate["weighted_case_rate"] == 0.0
    assert rows[0]["assessment_reasons"] == "wrong selected set"
    assert rows[0]["fallback_total"] == "2"


def test_bundle_refuses_to_overwrite_existing_run(tmp_path):
    module = _load_module()
    artifact = tmp_path / "trace.log"
    artifact.write_text("evidence", encoding="utf-8")
    output_root = tmp_path / "runs"
    module.build_bundle(
        run_id="F-test",
        classification="historical",
        artifacts=[artifact],
        output_root=output_root,
    )

    try:
        module.build_bundle(
            run_id="F-test",
            classification="historical",
            artifacts=[artifact],
            output_root=output_root,
        )
    except FileExistsError:
        pass
    else:
        raise AssertionError("existing evidence bundle was overwritten")
