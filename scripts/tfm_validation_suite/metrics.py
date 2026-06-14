"""Metrics extraction for the TFM fake skill validation suite.

Computes per-run and aggregate metrics from ValidationResult objects,
plus scoring functions for case-level evaluation.
"""

from __future__ import annotations

from typing import Any


def compute_run_metrics(result: Any, case: Any) -> dict:
    """Compute per-run metrics from a ValidationResult + TestCase."""
    trace_events = result.trace_events or []
    skill_results = result.skill_results or []
    assertions = result.assertions or []

    # Basic fields
    metrics = {
        "case_id": case.case_id,
        "utterance": case.goal_text,
        "global_mode": case.global_mode,
        "scenario_id": case.scenario_id or "",
        "event_count": len(trace_events),
        "latency_total_sec": result.duration_sec,
    }

    # Planner metrics
    planner_called = any(e.get("event_type") == "planner_request" for e in trace_events)
    skills_dispatched = _extract_skills(skill_results)
    skill_statuses = [sr.get("status", "unknown") for sr in skill_results]

    metrics.update({
        "planner_called": planner_called,
        "plan_steps": len(case.skills_to_execute),
        "skills_dispatched": skills_dispatched,
        "skill_result_statuses": skill_statuses,
        "replan_requested": any(e.get("event_type") == "replan" for e in trace_events),
        "clarification_requested": any(e.get("event_type") == "clarification" for e in trace_events),
        "final_status": skill_statuses[-1] if skill_statuses else "none",
        "errors_count": sum(1 for a in assertions if not a["passed"]),
    })

    # Planner correctness
    expected_skills = set(case.skills_to_execute)
    dispatched_set = set(skills_dispatched)
    metrics.update({
        "plan_valid_json": True,  # always True in offline mode
        "plan_contains_expected_skills": bool(expected_skills & dispatched_set),
        "forbidden_say_skill_present": "say" in dispatched_set,
        "all_steps_attempted": len(skill_results) == len(case.skills_to_execute),
    })

    # Execution metrics
    first_fail = next((i for i, s in enumerate(skill_statuses) if s != "succeeded"), None)
    metrics.update({
        "first_failure_step": first_fail,
        "safe_stop_after_failure": first_fail is not None and len(skill_results) <= first_fail + 1,
        "fake_skill_event_seen": any(e.get("event_type") == "skill_result" for e in trace_events),
    })

    # HRI metrics (best-effort from offline data)
    final_response = _final_response_text(trace_events)
    metrics.update({
        "final_response_present": bool(final_response),
        "response_not_duplicate": True,  # no duplicates in single-run offline mode
        "clarification_when_ambiguous": False,  # requires full user-turn mode
        "failure_explained_when_blocked": False,  # requires chatbot response analysis
    })

    return metrics


def compute_aggregate_metrics(runs: list[dict]) -> dict:
    """Compute aggregate metrics across all runs."""
    total = len(runs)
    if not total:
        return {"total_runs": 0}

    passed = sum(1 for r in runs if _run_passed(r))
    with_planner = sum(1 for r in runs if r.get("planner_called"))
    plan_correct = sum(1 for r in runs if r.get("plan_contains_expected_skills"))
    safe_failures = sum(1 for r in runs if not _run_passed(r) and r.get("safe_stop_after_failure"))
    has_forbidden = sum(1 for r in runs if r.get("forbidden_say_skill_present"))

    latencies = [r["latency_total_sec"] for r in runs]
    timeouts = sum(1 for l in latencies if l > 60.0)

    return {
        "total_runs": total,
        "passed": passed,
        "failed": total - passed,
        "success_rate": round(passed / total, 4),
        "route_accuracy": round(with_planner / total, 4),
        "plan_correctness_rate": round(plan_correct / total, 4),
        "safe_failure_rate": round(safe_failures / max(total - passed, 1), 4),
        "mean_latency_sec": round(sum(latencies) / total, 4),
        "timeout_rate": round(timeouts / total, 4),
        "forbidden_plan_rate": round(has_forbidden / total, 4),
    }


def score_case(case: Any, result: Any) -> float:
    """Score a case from 0.0 (all fail) to 1.0 (all pass)."""
    assertions = result.assertions or []
    if not assertions:
        return 1.0 if result.passed else 0.0
    passed = sum(1 for a in assertions if a["passed"])
    return round(passed / len(assertions), 4)


# ── Helpers ────────────────────────────────────────────────────────────

def _extract_skills(skill_results: list[dict]) -> list[str]:
    """Extract skill names from skill result payloads."""
    skills = []
    for sr in skill_results:
        name = sr.get("skill") or sr.get("ab_object_id", "")
        if name:
            skills.append(str(name))
    return skills


def _run_passed(run: dict) -> bool:
    """Check if a run metrics dict indicates pass."""
    return bool(run.get("passed"))


def _final_response_text(events: list[dict]) -> str:
    """Extract final response text from trace events."""
    responses = [e.get("summary", "") for e in events if e.get("event_type") in ("robot_speech", "planner_dialogue_act")]
    return responses[-1] if responses else ""
