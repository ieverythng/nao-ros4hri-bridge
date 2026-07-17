"""Task and role-scoped AB interaction projection."""

from __future__ import annotations

from ab_harness.contracts import AbstractionFrame, AgentRoleSpec, InteractionModuleSpec
from ab_harness.registry import RegistrySnapshot


class InteractionProjector:
    def __init__(self, registry: RegistrySnapshot) -> None:
        self._registry = registry

    def compile(
        self,
        *,
        task_id: str,
        role: AgentRoleSpec,
        frame: AbstractionFrame,
        requested_object_ids: tuple[str, ...],
        include_decomposition: bool = True,
    ) -> InteractionModuleSpec:
        selected: dict[str, object] = {}
        pending = list(requested_object_ids)
        while pending:
            object_id = pending.pop(0)
            if object_id in selected:
                continue
            item = self._registry.get(object_id)
            if item is None:
                raise ValueError('unknown AB object: %s' % object_id)
            if not role.control_band.admits_inspection(item.ab_level):
                raise ValueError('AB object outside inspectable control band: %s' % object_id)
            selected[object_id] = item
            if include_decomposition:
                pending.extend(item.decomposes_to)

        objects = tuple(sorted(selected.values(), key=lambda item: (item.ab_level, item.object_id)))
        return InteractionModuleSpec(
            task_id=task_id,
            role=role,
            frame=frame,
            objects=objects,
            registry_source=self._registry.source,
        )
