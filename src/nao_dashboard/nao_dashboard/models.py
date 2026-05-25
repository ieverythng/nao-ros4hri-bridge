"""Shared data models for the NAO dashboard backend."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import time


@dataclass(frozen=True)
class DashboardEvent:
    """Normalized event shape shared by trace viewer and dashboard UI."""

    timestamp: float
    event_id: str
    run_id: str
    trace_id: str | None
    source: str
    event_type: str
    channel: str
    ab_object_id: str | None
    ab_level: int | None
    payload_summary: str
    payload: dict

    def to_dict(self) -> dict:
        return {
            'timestamp': float(self.timestamp),
            'event_id': self.event_id,
            'run_id': self.run_id,
            'trace_id': self.trace_id,
            'source': self.source,
            'event_type': self.event_type,
            'channel': self.channel,
            'ab_object_id': self.ab_object_id,
            'ab_level': self.ab_level,
            'payload_summary': self.payload_summary,
            'payload': dict(self.payload),
        }


@dataclass(frozen=True)
class RosGraphSnapshot:
    """Current ROS graph summary for dashboard health panels."""

    timestamp: float
    nodes: list[dict]
    topics: list[dict]
    services: list[dict]
    actions: list[dict]
    edges: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            'timestamp': float(self.timestamp),
            'nodes': list(self.nodes),
            'topics': list(self.topics),
            'services': list(self.services),
            'actions': list(self.actions),
            'edges': list(self.edges),
        }


@dataclass(frozen=True)
class ABRegistrySnapshot:
    """AB registry view projected for dashboard tables/graphs."""

    timestamp: float
    objects: list[dict]
    edges: list[dict]
    validation_errors: list[str]

    def to_dict(self) -> dict:
        return {
            'timestamp': float(self.timestamp),
            'objects': list(self.objects),
            'edges': list(self.edges),
            'validation_errors': list(self.validation_errors),
        }


def make_run_id() -> str:
    return time.strftime('run_%Y%m%d_%H%M%S', time.localtime())
