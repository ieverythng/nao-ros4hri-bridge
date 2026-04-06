from planner_common import ExecutionFeedback
from planner_common import PlannerRequest

from planner_llm.planner_engine import PlannerEngine
from planner_llm.providers import PlannerProviderConfig
from planner_llm.providers import build_provider


class _FakeProvider:
    def __init__(self, response_text: str) -> None:
        self._response_text = response_text
        self.messages = []

    def generate(self, messages):
        self.messages = list(messages)
        return self._response_text


def test_planner_engine_builds_plan_from_model_json() -> None:
    provider = _FakeProvider(
        '{"ack_text":"I will check the cup.","ack_mode":"auto","validation_status":"draft","steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},"requires":[],"on_failure":"replan","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r1',
            'user_text': 'look to the left for the cup',
            'normalized_intents': ['head_look_left'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(request, world_model_text='Current world model context: cup visible')
    assert decision.mode == 'rule'
    assert decision.payload['plan']['steps'][0]['args']['object'] == 'head_look_left'
    assert decision.payload['plan']['retry_budget'] == 2
    assert provider.messages == []


def test_planner_engine_uses_provider_for_non_rule_request() -> None:
    provider = _FakeProvider(
        '```json\n{"ack_text":"I will inspect the scene.","steps":[{"type":"look_at","name":"look_at","args":{"target_frame":"cup_frame"},"requires":[],"on_failure":"replan","retry_budget":0}],"retry_budget":1}\n```'
    )
    engine = PlannerEngine(provider, default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r2',
            'user_text': 'look at the cup',
            'normalized_intents': ['inspect_scene'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(
        request,
        world_model_text='Current world model context: cup visible',
        world_model_snapshot={'scene_targets': ['cup']},
    )
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['type'] == 'look_at'
    assert decision.payload['plan']['scene_targets'] == ['cup']
    assert provider.messages[0]['role'] == 'system'


def test_planner_engine_clarifies_when_retry_budget_is_exhausted() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, default_retry_budget=2)
    request = PlannerRequest.from_payload({'request_id': 'r3', 'user_text': 'bring me the cup'})
    feedback = ExecutionFeedback.from_payload(
        {
            'plan_id': 'plan_1',
            'status': 'failed',
            'reason': 'path blocked',
            'retry_budget': 0,
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(request, feedback=feedback)
    assert decision.mode == 'clarify'
    assert decision.payload['plan']['steps'][0]['type'] == 'say'
    assert decision.payload['plan']['replan_hint'] == 'clarify_user'
    assert provider.messages == []


def test_provider_factory_selects_openai_compatible_adapter() -> None:
    provider = build_provider(
        PlannerProviderConfig(provider='openai', model='qwen', base_url='http://localhost:8080')
    )
    assert provider.__class__.__name__ == 'OpenAICompatiblePlannerProvider'
