"""Internal contracts for the fake skills engine."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FakeSkillRequest:
    """Normalized fake skill request routed from an action goal."""

    skill: str
    args: dict
    scenario_id: str = ''
    scenario_override: dict | None = None
