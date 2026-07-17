from pathlib import Path

import pytest

from ab_harness import AgentOutput
from ab_harness import HarnessTrace
from ab_harness import InteractionProjector
from ab_harness import JsonlHarnessTraceStore
from ab_harness import OutputGate
from ab_harness import RegistrySnapshot
from ab_harness.nao_h0 import CHATBOT_ROLE
from ab_harness.nao_h0 import PLANNER_ROLE
from ab_harness.nao_h0 import chatbot_output
from ab_harness.nao_h0 import nao_frame
from ab_harness.nao_h0 import planner_output


ROOT = Path(__file__).resolve().parents[3]
REGISTRY = ROOT / 'src/Neural-Wokbench/src/skill_common/skill_common/defaults/ab_registry.json'


def _registry() -> RegistrySnapshot:
    return RegistrySnapshot.from_json_file(REGISTRY)


def _module(role, *objects):
    registry = _registry()
    return InteractionProjector(registry).compile(
        task_id='task_h0',
        role=role,
        frame=nao_frame(registry),
        requested_object_ids=tuple(objects),
    )


def test_projection_uses_canonical_registry_and_includes_inspectable_decomposition():
    module = _module(PLANNER_ROLE, 'find_object', 'report_result')

    assert 'find_object' in module.object_ids
    assert 'resolve_target_reference' in module.object_ids
    assert module.object_for('resolve_target_reference').ab_level == 0
    assert module.frame.registry_version.startswith('sha256:')
    assert module.registry_source == str(REGISTRY)


def test_planner_output_accepts_only_projected_ab1_skill_references():
    module = _module(PLANNER_ROLE, 'find_object', 'report_result')
    output = planner_output(
        {
            'plan': {
                'steps': [
                    {'id': 'step_1', 'type': 'skill', 'name': 'find_object', 'args': {'label': 'cup'}},
                    {'id': 'step_2', 'type': 'skill', 'name': 'report_result', 'args': {}},
                ]
            }
        }
    )

    decision = OutputGate().evaluate(output, module)

    assert decision.accepted is True
    assert decision.reasons == ()


def test_gate_rejects_unprojected_and_inspection_only_objects():
    module = _module(PLANNER_ROLE, 'find_object')

    outside = OutputGate().evaluate(
        AgentOutput('executable_plan', {}, referenced_objects=('walk_to',)),
        module,
    )
    primitive = OutputGate().evaluate(
        AgentOutput('executable_plan', {}, referenced_objects=('resolve_target_reference',)),
        module,
    )

    assert outside.accepted is False
    assert 'outside task projection' in outside.reasons[0]
    assert primitive.accepted is False
    assert 'inspection-only' in primitive.reasons[0]


def test_chatbot_handoff_can_reference_admitted_skill_but_cannot_emit_plan():
    module = _module(CHATBOT_ROLE, 'find_object')
    handoff = chatbot_output(
        {'route': 'execution', 'user_intent': {'type': 'find_object', 'goal_text': 'find the cup'}}
    )
    plan = AgentOutput('executable_plan', {}, referenced_objects=('find_object',))

    assert OutputGate().evaluate(handoff, module).accepted is True
    rejected = OutputGate().evaluate(plan, module)
    assert rejected.accepted is False
    assert 'not owned by role' in rejected.reasons[0]


def test_model_roles_cannot_claim_execution_effects():
    module = _module(PLANNER_ROLE, 'find_object')
    decision = OutputGate().evaluate(
        AgentOutput(
            'executable_plan',
            {},
            referenced_objects=('find_object',),
            claimed_effects=('cup located',),
        ),
        module,
    )

    assert decision.accepted is False
    assert any('may not claim execution effects' in reason for reason in decision.reasons)


def test_unknown_registry_object_fails_projection_closed():
    with pytest.raises(ValueError, match='unknown AB object'):
        _module(PLANNER_ROLE, 'invented_skill')


def test_h0_trace_round_trip_reconstructs_projection_and_gate(tmp_path):
    module = _module(PLANNER_ROLE, 'find_object')
    output = AgentOutput('executable_plan', {}, referenced_objects=('find_object',))
    decision = OutputGate().evaluate(output, module)
    trace = HarnessTrace(
        trace_id='trace_h0_001',
        task_id=module.task_id,
        role_id=module.role.role_id,
        frame_id=module.frame.frame_id,
        registry_version=module.frame.registry_version,
        projected_object_ids=tuple(sorted(module.object_ids)),
        output_type=output.output_type,
        referenced_objects=output.referenced_objects,
        gate=decision,
        evidence_refs=('planner_output:fixture_001',),
        metadata={'mode': 'parent_repo_h0'},
    )
    store = JsonlHarnessTraceStore(tmp_path / 'h0.jsonl')

    store.append(trace)
    loaded = store.load_all()

    assert loaded == (trace,)
    assert loaded[0].gate.accepted is True
    assert 'resolve_target_reference' in loaded[0].projected_object_ids
