"""Core validation engine — integrates FakeSkillEngine, trace recording, and stack observation.

This module provides the offline validation harness that simulates the full
planner → orchestrator → fake skill execution loop without requiring a ROS runtime.
It mirrors the live probe script's flow but runs entirely in-process.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Import from the existing fake_skills package (same repo)
import sys
_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "src" / "fake_skills"))
sys.path.insert(0, str(_REPO_ROOT / "src" / "interaction_trace_viewer"))

from fake_skills.engine import FakeSkillEngine
from fake_skills.scenario_store import ScenarioStore
from interaction_trace_viewer.trace_model import (
    InteractionEvent,
    TraceRecorder,
    JsonlTraceWriter,
)


# ── Data models ────────────────────────────────────────────────────────

@dataclass(frozen=True)
class TestCase:
    """One validation test case."""
    case_id: str
    scenario_id: str = ""
    global_mode: str = "scenario"
    goal_text: str = ""
    normalized_intents: tuple[str, ...] = ()
    scene_targets: tuple[str, ...] = ()
    random_failure_prob: float = 0.5
    skills_to_execute: tuple[str, ...] = ()
    skill_args: dict = field(default_factory=dict)
    expected_outcomes: dict = field(default_factory=dict)
    description: str = ""


@dataclass
class ValidationResult:
    """Result of running one test case."""
    case_id: str
    passed: bool
    assertions: list[dict] = field(default_factory=list)
    skill_results: list[dict] = field(default_factory=list)
    trace_events: list[dict] = field(default_factory=list)
    duration_sec: float = 0.0
    error: str = ""


@dataclass
class StackObservation:
    """Snapshot of stack state from the observer."""
    timestamp: float
    node_name: str
    event_type: str
    channel: str
    payload: dict = field(default_factory=dict)


# ── Validation Engine ─────────────────────────────────────────────────

class ValidationEngine:
    """Offline validation engine that simulates the full stack.

    Mirrors the live probe flow:
      1. Set fake skill policy (global_mode, scenario_id, failure_prob)
      2. Execute skills through FakeSkillEngine
      3. Record trace events via TraceRecorder
      4. Capture stack observations
      5. Run assertions against expected outcomes
    """

    def __init__(self, *, scenario_file: str = "", output_dir: str = "") -> None:
        # Load scenario store
        if scenario_file and Path(scenario_file).exists():
            self._scenario_store = ScenarioStore.load_file(scenario_file)
        else:
            self._scenario_store = ScenarioStore({})

        # Create engine with default settings
        self._engine = FakeSkillEngine(
            scenario_store=self._scenario_store,
            default_delay_sec=0.0,  # offline — no actual delay
            deterministic_seed=42,
            global_mode="scenario",
            random_failure_prob=0.5,
        )

        # Trace infrastructure
        self._trace_recorder = TraceRecorder()
        self._trace_writer: JsonlTraceWriter | None = None
        if output_dir:
            self._trace_writer = JsonlTraceWriter(output_dir)

        # Stack observations
        self._observations: list[StackObservation] = []

    def _set_policy(
        self,
        *,
        global_mode: str = "scenario",
        random_failure_prob: float = 0.5,
        active_scenario_id: str = "",
    ) -> None:
        """Update fake skill policy before running a test case."""
        self._engine.update_policy(
            global_mode=global_mode,
            random_failure_prob=random_failure_prob,
            mode_overrides=None,
        )

    def execute_skill(
        self,
        *,
        skill: str,
        args: dict,
        scenario_id: str = "",
        scenario_override: dict | None = None,
    ) -> tuple[dict, float]:
        """Execute one fake skill and record the trace event."""
        payload, delay_sec = self._engine.execute(
            skill=skill,
            args=args,
            scenario_id=scenario_id,
            scenario_override=scenario_override,
        )

        # Record as interaction event
        event = InteractionEvent(
            timestamp=time.time(),
            trace_id=None,
            source_node="fake_skill_server",
            channel="/fake_skills/events",
            event_type="skill_result",
            ab_object_id=skill,
            ab_level=0,
            summary=f"fake skill '{skill}' executed: {payload.get('status', 'unknown')}",
            payload=payload,
        )
        enriched = self._trace_recorder.add(event)

        # Write to JSONL if configured
        if self._trace_writer:
            self._trace_writer.write(enriched)

        # Record stack observation
        obs = StackObservation(
            timestamp=enriched.timestamp,
            node_name="fake_skill_server",
            event_type="skill_result",
            channel="/fake_skills/events",
            payload=enriched.to_dict(),
        )
        self._observations.append(obs)

        return payload, delay_sec

    def simulate_planner_request(
        self,
        *,
        goal_text: str,
        normalized_intents: list[str],
        scene_targets: list[str] | None = None,
        turn_id: str = "",
        goal_id: str = "",
    ) -> InteractionEvent:
        """Simulate a planner request being published and recorded."""
        if not turn_id:
            turn_id = f"test_turn_{int(time.time() * 1000)}"
        if not goal_id:
            goal_id = f"test_goal_{int(time.time() * 1000)}"

        payload = {
            "request_id": turn_id,
            "goal_id": goal_id,
            "goal_text": goal_text,
            "normalized_intents": normalized_intents,
            "scene_targets": scene_targets or [],
            "planner_mode": "default",
            "interaction_mode": "speech",
        }

        event = InteractionEvent(
            timestamp=time.time(),
            trace_id=goal_id,
            source_node="planner_llm",
            channel="/planner/request",
            event_type="planner_request",
            ab_object_id=None,
            ab_level=1,
            summary=f"planner request: {goal_text}",
            payload=payload,
        )
        enriched = self._trace_recorder.add(event)

        if self._trace_writer:
            self._trace_writer.write(enriched)

        obs = StackObservation(
            timestamp=enriched.timestamp,
            node_name="planner_llm",
            event_type="planner_request",
            channel="/planner/request",
            payload=enriched.to_dict(),
        )
        self._observations.append(obs)

        return enriched

    def get_trace_events(self) -> list[dict]:
        """Return all recorded trace events as dicts."""
        return [e.to_dict() for e in self._trace_recorder.events()]

    def get_observations(self) -> list[dict]:
        """Return all stack observations as dicts."""
        return [
            {
                "timestamp": o.timestamp,
                "node_name": o.node_name,
                "event_type": o.event_type,
                "channel": o.channel,
                "payload": o.payload,
            }
            for o in self._observations
        ]

    def run_case(self, case: TestCase) -> ValidationResult:
        """Run one test case through the full validation pipeline."""
        start = time.time()
        result = ValidationResult(case_id=case.case_id, passed=True)

        try:
            # 1. Set policy
            self._set_policy(
                global_mode=case.global_mode,
                random_failure_prob=case.random_failure_prob,
                active_scenario_id=case.scenario_id,
            )

            # 2. Simulate planner request
            self.simulate_planner_request(
                goal_text=case.goal_text,
                normalized_intents=list(case.normalized_intents),
                scene_targets=list(case.scene_targets),
            )

            # 3. Execute skills
            for skill_name in case.skills_to_execute:
                args = dict(case.skill_args)
                if "target" not in args and case.scene_targets:
                    args["target"] = case.scene_targets[0]
                payload, _delay = self.execute_skill(
                    skill=skill_name,
                    args=args,
                    scenario_id=case.scenario_id,
                )
                result.skill_results.append(payload)

            # 4. Check expected outcomes
            for key, expected in case.expected_outcomes.items():
                actual = None
                found = False
                for sr in result.skill_results:
                    if key in sr:
                        actual = sr[key]
                        found = True
                        break
                    # Check metadata
                    meta = sr.get("metadata", {})
                    if key in meta:
                        actual = meta[key]
                        found = True
                        break

                passed = self._check_assertion(key, expected, actual)
                result.assertions.append({
                    "key": key,
                    "expected": str(expected),
                    "actual": str(actual),
                    "passed": passed,
                })
                if not passed:
                    result.passed = False

        except Exception as e:
            result.passed = False
            result.error = str(e)

        result.duration_sec = time.time() - start
        result.trace_events = self.get_trace_events()

        return result

    @staticmethod
    def _check_assertion(key: str, expected: Any, actual: Any) -> bool:
        """Check a single assertion."""
        if isinstance(expected, str) and actual is not None:
            return str(actual).lower() == expected.lower()
        if isinstance(expected, bool):
            return bool(actual) == expected
        if isinstance(expected, (int, float)):
            return actual == expected
        # Default: equality check
        return actual == expected

    def close(self) -> None:
        """Close trace writer."""
        if self._trace_writer and not self._trace_writer._handle.closed:
            self._trace_writer.close()
