"""Portable contracts for an AB-grounded agent interaction."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class AbstractionFrame:
    frame_id: str
    substrate: str
    atomicity_rule: str
    registry_version: str


@dataclass(frozen=True)
class ABControlBand:
    min_direct_level: int
    preferred_level: int
    max_direct_level: int
    inspect_down_to_level: int = 0

    def __post_init__(self) -> None:
        if not (
            self.inspect_down_to_level
            <= self.min_direct_level
            <= self.preferred_level
            <= self.max_direct_level
        ):
            raise ValueError('invalid AB control band ordering')

    def admits_direct(self, level: int) -> bool:
        return self.min_direct_level <= level <= self.max_direct_level

    def admits_inspection(self, level: int) -> bool:
        return self.inspect_down_to_level <= level <= self.max_direct_level


@dataclass(frozen=True)
class AgentRoleSpec:
    role_id: str
    allowed_output_types: tuple[str, ...]
    control_band: ABControlBand
    may_claim_effects: bool = False


@dataclass(frozen=True)
class ABObjectView:
    object_id: str
    ab_level: int
    kind: str
    category: str
    owner_package: str
    expected_effects: tuple[str, ...] = ()
    observable_success: tuple[str, ...] = ()
    decomposes_to: tuple[str, ...] = ()
    runtime_callable: bool = False


@dataclass(frozen=True)
class InteractionModuleSpec:
    task_id: str
    role: AgentRoleSpec
    frame: AbstractionFrame
    objects: tuple[ABObjectView, ...]
    registry_source: str

    @property
    def object_ids(self) -> frozenset[str]:
        return frozenset(item.object_id for item in self.objects)

    def object_for(self, object_id: str) -> ABObjectView | None:
        return next((item for item in self.objects if item.object_id == object_id), None)


@dataclass(frozen=True)
class AgentOutput:
    output_type: str
    payload: dict[str, Any]
    referenced_objects: tuple[str, ...] = ()
    claimed_effects: tuple[str, ...] = ()


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class HarnessTrace:
    trace_id: str
    task_id: str
    role_id: str
    frame_id: str
    registry_version: str
    projected_object_ids: tuple[str, ...]
    output_type: str
    referenced_objects: tuple[str, ...]
    gate: GateDecision
    evidence_refs: tuple[str, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> 'HarnessTrace':
        gate = payload.get('gate', {})
        return cls(
            trace_id=str(payload['trace_id']),
            task_id=str(payload['task_id']),
            role_id=str(payload['role_id']),
            frame_id=str(payload['frame_id']),
            registry_version=str(payload['registry_version']),
            projected_object_ids=tuple(payload.get('projected_object_ids', ())),
            output_type=str(payload['output_type']),
            referenced_objects=tuple(payload.get('referenced_objects', ())),
            gate=GateDecision(bool(gate.get('accepted')), tuple(gate.get('reasons', ()))),
            evidence_refs=tuple(payload.get('evidence_refs', ())),
            metadata=dict(payload.get('metadata', {})),
        )
