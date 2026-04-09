from planner_common import ExecutionFeedback
from planner_common import PlannerRequest

from planner_llm.planner_engine import PlannerEngine
from planner_llm.providers import PlannerProviderConfig
from planner_llm.providers import build_provider
from planner_llm.skill_registry import SkillRegistry


class _FakeProvider:
    def __init__(self, response_text: str) -> None:
        self._response_text = response_text
        self.messages = []

    def generate(self, messages):
        self.messages = list(messages)
        return self._response_text


def _engine_for_response(response_text: str, *, retry_budget: int = 1) -> PlannerEngine:
    return PlannerEngine(
        _FakeProvider(response_text),
        SkillRegistry.load(),
        default_retry_budget=retry_budget,
    )


def test_planner_engine_builds_rule_plan_for_motion_intent() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r1',
            'goal_id': 'goal_1',
            'user_text': 'look to the left for the cup',
            'normalized_intents': ['head_look_left'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_1', plan_version=1)
    assert decision.mode == 'rule'
    assert decision.payload['plan']['goal_id'] == 'goal_1'
    assert decision.payload['plan']['plan_version'] == 1
    assert decision.payload['plan']['steps'][0]['args']['object'] == 'head_look_left'
    assert decision.payload['plan']['retry_budget'] == 2
    assert provider.messages == []


def test_planner_engine_uses_provider_for_non_rule_request() -> None:
    provider = _FakeProvider(
        '```json\n{"ack_text":"I will inspect the scene.","steps":[{"type":"look_at","name":"look_at","args":{"target_frame":"cup_frame"},"requires":[],"on_failure":"replan","retry_budget":0}],"retry_budget":1}\n```'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r2',
            'goal_id': 'goal_2',
            'user_text': 'look at the cup',
            'normalized_intents': ['inspect_scene'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(
        request,
        world_model_text='Current world model context: cup visible',
        world_model_snapshot={'scene_targets': ['cup']},
        goal_id='goal_2',
        plan_version=2,
        status='replanning',
    )
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['type'] == 'look_at'
    assert decision.payload['plan']['scene_targets'] == ['cup']
    assert decision.payload['plan']['plan_version'] == 2
    assert provider.messages[0]['role'] == 'system'


def test_planner_engine_filters_unsupported_model_skills() -> None:
    provider = _FakeProvider(
        '{"ack_text":"Trying a custom action.","steps":[{"type":"skill","name":"dance","args":{"style":"wave"},"requires":[],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_bad',
            'goal_id': 'goal_bad',
            'user_text': 'do something unknown',
            'normalized_intents': ['custom_action'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_bad', plan_version=1)
    assert decision.mode == 'clarify'
    assert decision.payload['plan']['replan_hint'] == 'clarify_user'


def test_planner_engine_clarifies_when_retry_budget_is_exhausted() -> None:
    engine = _engine_for_response('{}', retry_budget=2)
    request = PlannerRequest.from_payload(
        {'request_id': 'r3', 'goal_id': 'goal_3', 'user_text': 'bring me the cup'}
    )
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_3',
            'plan_id': 'plan_1',
            'plan_version': 1,
            'status': 'failed',
            'reason': 'path blocked',
            'retry_budget': 0,
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(request, feedback=feedback, goal_id='goal_3', plan_version=2)
    assert decision.mode == 'clarify'
    assert decision.payload['plan']['steps'][0]['type'] == 'say'
    assert decision.payload['plan']['replan_hint'] == 'clarify_user'


def test_planner_engine_uses_provider_for_multi_step_requests_even_with_rule_intent() -> None:
    provider = _FakeProvider(
        '{"ack_text":"Moving my head up, then sitting down.","steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_up"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"perform_motion","args":{"object":"sit"},"requires":[],"on_failure":"replan","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_multi',
            'goal_id': 'goal_multi',
            'user_text': 'move your head up and then sit down',
            'normalized_intents': ['head_look_up'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_multi', plan_version=1)
    assert decision.mode == 'plan'
    assert len(decision.payload['plan']['steps']) == 2
    assert provider.messages[0]['role'] == 'system'


def test_provider_factory_selects_openai_compatible_adapter() -> None:
    provider = build_provider(
        PlannerProviderConfig(provider='openai', model='qwen', base_url='http://localhost:8080')
    )
    assert provider.__class__.__name__ == 'OpenAICompatiblePlannerProvider'
