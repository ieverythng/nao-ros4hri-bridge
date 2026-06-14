"""Test case definitions for the TFM fake skill validation suite.

Each test case maps to a seam from the plan document:
  - Phase 1: Skill-level isolation (individual skills in controlled modes)
  - Phase 2: Scenario-driven behavior (named scenarios with expected outcomes)
  - Phase 3: Global policy modes (always_success, always_fail, every_other, random_seeded)
  - Phase 4: Planner contract validation (request → plan → execution feedback loop)
  - Phase 5: Integration / closed-loop (trace recording + stack observation)
"""

from __future__ import annotations

from tfm_validation_suite.validation_core import TestCase


# ────────────────────────────────────────────────────────────────────────
# PHASE 1: Skill-Level Isolation Tests
# Each skill tested in default scenario mode with success outcome
# ────────────────────────────────────────────────────────────────────────

PHASE1_SKILL_ISOLATION: list[TestCase] = [
    TestCase(
        case_id="P1_find_object_success",
        global_mode="scenario",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"status": "succeeded", "result_mode": "found"},
        description="find_object in scenario mode should return found/succeeded",
    ),
    TestCase(
        case_id="P1_navigate_to_success",
        global_mode="scenario",
        goal_text="navigate to the kitchen",
        normalized_intents=("move_to",),
        scene_targets=("kitchen",),
        skills_to_execute=("navigate_to",),
        skill_args={"target": "kitchen"},
        expected_outcomes={"status": "succeeded", "result_mode": "success"},
        description="navigate_to in scenario mode should return success",
    ),
    TestCase(
        case_id="P1_perform_motion_success",
        global_mode="scenario",
        goal_text="wave hello",
        normalized_intents=("perform_motion",),
        scene_targets=(),
        skills_to_execute=("perform_motion",),
        skill_args={"target": "wave"},
        expected_outcomes={"status": "succeeded", "result_mode": "success"},
        description="perform_motion in scenario mode should return success",
    ),
    TestCase(
        case_id="P1_wave_greet_success",
        global_mode="scenario",
        goal_text="greet the user",
        normalized_intents=("greet",),
        scene_targets=(),
        skills_to_execute=("wave_greet",),
        skill_args={"target": "user"},
        expected_outcomes={"status": "succeeded", "result_mode": "success"},
        description="wave_greet in scenario mode should return success",
    ),
    TestCase(
        case_id="P1_inspect_area_clear",
        global_mode="scenario",
        goal_text="inspect the room",
        normalized_intents=("inspect_area",),
        scene_targets=("room",),
        skills_to_execute=("inspect_area",),
        skill_args={"target": "room"},
        expected_outcomes={"status": "succeeded", "result_mode": "clear"},
        description="inspect_area in scenario mode should return clear",
    ),
    TestCase(
        case_id="P1_walk_to_success",
        global_mode="scenario",
        goal_text="walk to the table",
        normalized_intents=("move_to",),
        scene_targets=("table",),
        skills_to_execute=("walk_to",),
        skill_args={"target": "table"},
        expected_outcomes={"status": "succeeded", "result_mode": "dry_run"},
        description="walk_to in scenario mode should return dry_run (default)",
    ),
]


# ────────────────────────────────────────────────────────────────────────
# PHASE 2: Scenario-Driven Behavior Tests
# Named scenarios that override default outcomes
# ────────────────────────────────────────────────────────────────────────

PHASE2_SCENARIO_DRIVEN: list[TestCase] = [
    TestCase(
        case_id="P2_ambiguous_cup",
        scenario_id="ambiguous_cup",
        global_mode="scenario",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "ambiguous"},
        description="ambiguous_cup scenario should make find_object return ambiguous",
    ),
    TestCase(
        case_id="P2_path_blocked",
        scenario_id="path_blocked",
        global_mode="scenario",
        goal_text="navigate to the cup",
        normalized_intents=("move_to",),
        scene_targets=("cup",),
        skills_to_execute=("navigate_to",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "path_blocked"},
        description="path_blocked scenario should make navigate_to fail with path_blocked",
    ),
    TestCase(
        case_id="P2_motion_unavailable",
        scenario_id="motion_unavailable",
        global_mode="scenario",
        goal_text="perform a motion",
        normalized_intents=("perform_motion",),
        scene_targets=(),
        skills_to_execute=("perform_motion",),
        skill_args={"target": "wave"},
        expected_outcomes={"result_mode": "motion_unavailable"},
        description="motion_unavailable scenario should make perform_motion fail",
    ),
    TestCase(
        case_id="P2_area_person_found",
        scenario_id="area_person_found",
        global_mode="scenario",
        goal_text="inspect the area",
        normalized_intents=("inspect_area",),
        scene_targets=("area",),
        skills_to_execute=("inspect_area",),
        skill_args={"target": "area"},
        expected_outcomes={"result_mode": "person_found"},
        description="area_person_found scenario should make inspect_area return person_found",
    ),
]


# ────────────────────────────────────────────────────────────────────────
# PHASE 3: Global Policy Mode Tests
# Testing always_success, always_fail, every_other, random_seeded modes
# ────────────────────────────────────────────────────────────────────────

PHASE3_GLOBAL_POLICY: list[TestCase] = [
    TestCase(
        case_id="P3_always_success",
        global_mode="always_success",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "found"},
        description="always_success mode should force success for all skills",
    ),
    TestCase(
        case_id="P3_always_fail",
        global_mode="always_fail",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "not_found"},
        description="always_fail mode should force failure for all skills",
    ),
    TestCase(
        case_id="P3_every_other_first",
        global_mode="every_other",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "found"},
        description="every_other: first call should succeed (call_count=0, even)",
    ),
    TestCase(
        case_id="P3_random_seeded_low_fail",
        global_mode="random_seeded",
        random_failure_prob=0.01,
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "found"},
        description="random_seeded with 1% failure should almost always succeed",
    ),
    TestCase(
        case_id="P3_random_seeded_high_fail",
        global_mode="random_seeded",
        random_failure_prob=0.99,
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "not_found"},
        description="random_seeded with 99% failure should almost always fail",
    ),
]


# ────────────────────────────────────────────────────────────────────────
# PHASE 4: Planner Contract Validation Tests
# Multi-step plans, intent normalization, grounded context handling
# ────────────────────────────────────────────────────────────────────────

PHASE4_PLANNER_CONTRACTS: list[TestCase] = [
    TestCase(
        case_id="P4_multi_skill_plan",
        global_mode="always_success",
        goal_text="find the cup and bring it to me",
        normalized_intents=("find_object", "navigate_to"),
        scene_targets=("cup",),
        skills_to_execute=("find_object", "navigate_to"),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "success"},
        description="Multi-skill plan: find_object then navigate_to, both should succeed in always_success mode",
    ),
    TestCase(
        case_id="P4_unsupported_skill",
        global_mode="scenario",
        goal_text="do something impossible",
        normalized_intents=("unknown_intent",),
        scene_targets=(),
        skills_to_execute=("nonexistent_skill",),
        skill_args={"target": "nothing"},
        expected_outcomes={"result_mode": "unsupported_skill"},
        description="Unknown skill should return unsupported_skill with recoverable=False",
    ),
    TestCase(
        case_id="P4_convergence_timeout",
        scenario_id="motion_convergence_timeout",
        global_mode="scenario",
        goal_text="perform a complex motion",
        normalized_intents=("perform_motion",),
        scene_targets=(),
        skills_to_execute=("perform_motion",),
        skill_args={"target": "complex_motion"},
        expected_outcomes={"result_mode": "convergence_timeout"},
        description="motion_convergence_timeout scenario should make perform_motion timeout",
    ),
]


# ────────────────────────────────────────────────────────────────────────
# PHASE 5: Integration / Closed-Loop Tests
# Trace recording, stack observation, event correlation
# ────────────────────────────────────────────────────────────────────────

PHASE5_INTEGRATION: list[TestCase] = [
    TestCase(
        case_id="P5_trace_correlation",
        global_mode="scenario",
        goal_text="find the cup",
        normalized_intents=("find_object",),
        scene_targets=("cup",),
        skills_to_execute=("find_object",),
        skill_args={"target": "cup"},
        expected_outcomes={"result_mode": "found"},
        description="Verify trace events are recorded with correct goal_id correlation",
    ),
    TestCase(
        case_id="P5_stack_observer_events",
        global_mode="scenario",
        goal_text="navigate to the kitchen and inspect",
        normalized_intents=("move_to", "inspect_area"),
        scene_targets=("kitchen",),
        skills_to_execute=("navigate_to", "inspect_area"),
        skill_args={"target": "kitchen"},
        expected_outcomes={"result_mode": "clear"},
        description="Verify stack observer captures events from both navigate_to and inspect_area",
    ),
]


# ────────────────────────────────────────────────────────────────────────
# Full suite — all phases combined
# ────────────────────────────────────────────────────────────────────────

ALL_TEST_CASES: list[TestCase] = (
    PHASE1_SKILL_ISOLATION
    + PHASE2_SCENARIO_DRIVEN
    + PHASE3_GLOBAL_POLICY
    + PHASE4_PLANNER_CONTRACTS
    + PHASE5_INTEGRATION
)

PHASE_MAP: dict[str, list[TestCase]] = {
    "phase1": PHASE1_SKILL_ISOLATION,
    "phase2": PHASE2_SCENARIO_DRIVEN,
    "phase3": PHASE3_GLOBAL_POLICY,
    "phase4": PHASE4_PLANNER_CONTRACTS,
    "phase5": PHASE5_INTEGRATION,
}


def get_test_cases(*, phase: str = "all") -> list[TestCase]:
    """Return test cases for a given phase or all phases."""
    if phase == "all":
        return ALL_TEST_CASES
    return PHASE_MAP.get(phase, [])


def get_case_by_id(case_id: str) -> TestCase | None:
    """Look up a single test case by ID."""
    for case in ALL_TEST_CASES:
        if case.case_id == case_id:
            return case
    return None
