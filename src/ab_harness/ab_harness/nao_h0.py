"""Read-only NAO H0 role contracts and output adapters."""

from __future__ import annotations

from ab_harness.contracts import ABControlBand, AbstractionFrame, AgentOutput, AgentRoleSpec
from ab_harness.registry import RegistrySnapshot


CHATBOT_ROLE = AgentRoleSpec(
    role_id='chatbot_llm',
    allowed_output_types=('dialogue_response', 'knowledge_query', 'planner_handoff'),
    control_band=ABControlBand(1, 1, 2, inspect_down_to_level=0),
    may_claim_effects=False,
)
PLANNER_ROLE = AgentRoleSpec(
    role_id='planner_llm',
    allowed_output_types=('executable_plan', 'supervision_decision', 'planner_dialogue_act'),
    control_band=ABControlBand(1, 1, 2, inspect_down_to_level=0),
    may_claim_effects=False,
)


def nao_frame(registry: RegistrySnapshot) -> AbstractionFrame:
    return AbstractionFrame(
        frame_id='nao_ros4hri',
        substrate='ROS4HRI planner and deterministic skill runtime',
        atomicity_rule='AB1 is planner-callable skill; AB0 is inspectable effect decomposition',
        registry_version=registry.version,
    )


def planner_output(payload: dict) -> AgentOutput:
    plan = payload.get('plan', {}) if isinstance(payload.get('plan'), dict) else {}
    references = []
    for step in plan.get('steps', ()):
        if isinstance(step, dict) and step.get('type') == 'skill':
            name = str(step.get('name', '')).strip()
            if name:
                references.append(name)
    return AgentOutput('executable_plan', dict(payload), tuple(references))


def chatbot_output(payload: dict) -> AgentOutput:
    route = str(payload.get('route', '')).strip()
    output_type = {
        'dialogue': 'dialogue_response',
        'knowledge_query': 'knowledge_query',
        'execution': 'planner_handoff',
    }.get(route, 'dialogue_response')
    user_intent = payload.get('user_intent', {})
    references = ()
    if isinstance(user_intent, dict):
        skill_name = str(user_intent.get('type', '')).strip()
        if skill_name:
            references = (skill_name,)
    return AgentOutput(output_type, dict(payload), references)
