"""Deterministic reachability and role gate for model outputs."""

from __future__ import annotations

from ab_harness.contracts import AgentOutput, GateDecision, InteractionModuleSpec


class OutputGate:
    def evaluate(self, output: AgentOutput, module: InteractionModuleSpec) -> GateDecision:
        reasons: list[str] = []
        if output.output_type not in module.role.allowed_output_types:
            reasons.append('output type is not owned by role: %s' % output.output_type)

        for object_id in output.referenced_objects:
            item = module.object_for(object_id)
            if item is None:
                reasons.append('referenced object is outside task projection: %s' % object_id)
                continue
            if not module.role.control_band.admits_direct(item.ab_level):
                reasons.append('referenced object is inspection-only: %s' % object_id)

        if output.claimed_effects and not module.role.may_claim_effects:
            reasons.append('role may not claim execution effects')

        return GateDecision(accepted=not reasons, reasons=tuple(reasons))
