"""Shared timing budget for asynchronous ROS action calls."""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Callable


@dataclass(frozen=True, slots=True)
class ActionDeadline:
    """Bound goal acceptance and execution to one result-timeout budget."""

    expires_at: float

    @classmethod
    def start(
        cls,
        timeout_sec: float,
        *,
        clock: Callable[[], float] = monotonic,
    ) -> 'ActionDeadline':
        return cls(clock() + max(float(timeout_sec), 0.1))

    def remaining(
        self,
        *,
        clock: Callable[[], float] = monotonic,
    ) -> float:
        return max(self.expires_at - clock(), 0.1)
