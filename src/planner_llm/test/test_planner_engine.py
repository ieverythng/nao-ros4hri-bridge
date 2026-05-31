from planner_common import ExecutionFeedback
from planner_common import PlannerRequest

from planner_llm.planner_engine import PlannerEngine
from planner_llm.providers import PlannerProviderConfig
from planner_llm.providers import PlannerProviderError
from planner_llm.providers import build_provider
from planner_llm.skill_registry import SkillRegistry


class _FakeProvider:
    def __init__(self, response_text: str) -> None:
        self._response_text = response_text
        self.messages = []

    def generate(self, messages):
        self.messages = list(messages)
        return self._response_text


class _SequenceProvider:
    def __init__(self, responses: list[str]) -> None:
        self._responses = list(responses)
        self.messages = []

    def generate(self, messages):
        self.messages.append(list(messages))
        if not self._responses:
            return '{}'
        return self._responses.pop(0)


class _FailingProvider:
    def generate(self, messages):
        raise PlannerProviderError('timed out')


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
            'goal_text': 'inspect the visible cup',
            'user_text': 'look at the cup',
            'normalized_intents': ['inspect_scene'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_2',
        plan_version=2,
        status='replanning',
    )
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['type'] == 'look_at'
    assert decision.payload['plan']['scene_targets'] == ['cup']
    assert decision.payload['plan']['plan_version'] == 2
    assert provider.messages[0]['role'] == 'system'
    assert '"goal_text": "inspect the visible cup"' in provider.messages[1]['content']
    assert '"user_text"' not in provider.messages[1]['content']
    assert '"invalid_examples": [{"name": "say", "type": "skill"}' in provider.messages[1]['content']


def test_planner_engine_canonicalizes_look_at_target_alias_from_model() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"look_at","name":"look_at","args":{"target":"anonymous person bcbhb"}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r2b',
            'goal_id': 'goal_2b',
            'goal_text': 'look at that human',
            'scene_targets': ['anonymous person bcbhb'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_2b', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['args']['target_frame'] == 'anonymous person bcbhb'


def test_planner_engine_reports_invalid_model_output_as_failure() -> None:
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
    assert decision.mode == 'fail'
    assert decision.payload['plan']['status'] == 'failed'
    assert decision.payload['plan']['replan_hint'] == 'planner_invalid_output'
    assert 'unsupported skill step name "dance"' in decision.payload['plan']['failure_reason']


def test_planner_engine_retries_invalid_model_plan_with_validation_feedback() -> None:
    provider = _SequenceProvider(
        [
            '{"steps":[{"type":"skill","name":"scan","args":{"target_kind":"scene"}},'
            '{"type":"skill","name":"say","args":{"text":"I looked around."}}]}',
            '{"steps":[{"type":"skill","name":"scan","args":{"target_kind":"scene"}}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_retry',
            'goal_id': 'goal_retry',
            'goal_text': 'scan the room and tell me what you see',
            'normalized_intents': ['inspect_scene'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_retry', plan_version=1)

    assert decision.mode == 'plan'
    assert len(provider.messages) == 2
    retry_prompt = provider.messages[1][1]['content']
    assert 'validation_retry' in retry_prompt
    assert 'do not mix speech steps into executable plans' in retry_prompt
    assert [step['type'] for step in decision.payload['plan']['steps']] == ['skill']


def test_planner_engine_rejects_mixed_say_and_executable_steps() -> None:
    provider = _FakeProvider(
        '{"ack_text":"Sure, I will move my head up and down for you.",'
        '"steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_up"}},'
        '{"type":"skill","name":"perform_motion","args":{"object":"head_look_down"}},'
        '{"type":"say","name":"say","args":{"text":"Sure, I will move my head up and down for you."}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_dedupe',
            'goal_id': 'goal_dedupe',
            'goal_text': 'move your head up and down',
            'normalized_intents': ['head_nod', 'head_look_up'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_dedupe', plan_version=1)

    assert decision.mode == 'fail'
    assert decision.payload['plan']['status'] == 'failed'
    assert 'say steps cannot be mixed with executable steps' in decision.payload['plan']['failure_reason']


def test_planner_engine_marks_provider_timeout_as_backend_unavailable() -> None:
    engine = PlannerEngine(_FailingProvider(), SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_timeout',
            'goal_id': 'goal_timeout',
            'goal_text': 'bring me the cup',
            'normalized_intents': ['inspect_scene'],
            'requested_plan': [],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_timeout', plan_version=1)

    assert decision.mode == 'backend_unavailable'
    assert decision.payload['plan']['status'] == 'failed'
    assert decision.payload['plan']['replan_hint'] == 'planner_backend_unavailable'
    assert 'timed out' in decision.raw_model_output


def test_planner_engine_rejects_partial_model_plans_when_one_step_is_unsupported() -> None:
    provider = _FakeProvider(
        '{"ack_text":"Trying two actions.","steps":[{"type":"skill","name":"perform_motion","args":{"object":"stand"},"requires":[],"on_failure":"fail","retry_budget":0},{"type":"skill","name":"dance","args":{"style":"wave"},"requires":[],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_partial',
            'goal_id': 'goal_partial',
            'goal_text': 'stand up and dance',
            'normalized_intents': ['multi_step_task'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_partial', plan_version=1)

    assert decision.mode == 'fail'
    assert decision.payload['plan']['steps'][0]['type'] == 'say'


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


def test_planner_engine_caps_model_retry_budget_to_remaining_feedback_budget() -> None:
    provider = _FakeProvider(
        '{"ack_text":"Retrying.","retry_budget":3,"steps":[{"type":"skill","name":"find_object","args":{"target":"cup"},"requires":[],"on_failure":"replan","retry_budget":3}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {'request_id': 'r3b', 'goal_id': 'goal_3b', 'user_text': 'find the cup'}
    )
    feedback = ExecutionFeedback.from_payload(
        {
            'goal_id': 'goal_3b',
            'plan_id': 'plan_1',
            'plan_version': 1,
            'status': 'failed',
            'reason': 'target moved',
            'retry_budget': 2,
        }
    )

    decision = engine.plan_request(request, feedback=feedback, goal_id='goal_3b', plan_version=2)
    assert decision.mode == 'replan'
    assert decision.payload['plan']['retry_budget'] == 1


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


def test_planner_engine_falls_back_to_requested_plan_when_model_output_is_invalid() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_hint',
            'goal_id': 'goal_hint',
            'user_text': 'look up and then sit down',
            'normalized_intents': ['head_look_up'],
            'requested_plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'head_look_up'},
                },
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'sit'},
                },
            ],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_hint', plan_version=1)

    assert decision.mode == 'hint'
    assert [step['args']['object'] for step in decision.payload['plan']['steps']] == [
        'head_look_up',
        'sit',
    ]
    assert provider.messages[1]['content'].find('"requested_plan"') != -1


def test_planner_engine_does_not_use_partial_requested_plan_hints() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_hint_partial',
            'goal_id': 'goal_hint_partial',
            'goal_text': 'stand up and dance',
            'requested_plan': [
                {
                    'type': 'skill',
                    'name': 'perform_motion',
                    'args': {'object': 'stand'},
                },
                {
                    'type': 'skill',
                    'name': 'dance',
                    'args': {'style': 'wave'},
                },
            ],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_hint_partial', plan_version=1)

    assert decision.mode == 'fail'


def test_provider_factory_selects_openai_compatible_adapter() -> None:
    provider = build_provider(
        PlannerProviderConfig(provider='openai', model='qwen', base_url='http://localhost:8080')
    )
    assert provider.__class__.__name__ == 'OpenAICompatiblePlannerProvider'


def test_ollama_provider_requires_configured_model() -> None:
    provider = build_provider(PlannerProviderConfig(provider='ollama', model=''))

    try:
        provider.generate([{'role': 'user', 'content': 'plan'}])
    except PlannerProviderError as err:
        assert 'model is not configured' in str(err)
    else:  # pragma: no cover - defensive assertion
        raise AssertionError('expected PlannerProviderError')


def test_openai_provider_warns_when_think_is_enabled() -> None:
    import warnings

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        provider = build_provider(
            PlannerProviderConfig(
                provider='openai',
                model='qwen',
                base_url='http://localhost:8080',
                think=True,
            )
        )

    assert provider.__class__.__name__ == 'OpenAICompatiblePlannerProvider'
    assert any('think=True is ignored' in str(item.message) for item in caught)


def test_ollama_provider_payload_disables_thinking_by_default(monkeypatch) -> None:
    captured = {}

    def fake_post_json(url, payload, *, timeout_sec, headers):
        captured['url'] = url
        captured['payload'] = payload
        return {'message': {'content': '{"steps":[]}'}}

    monkeypatch.setattr('planner_llm.providers._post_json', fake_post_json)
    provider = build_provider(
        PlannerProviderConfig(provider='ollama', model='qwen3.5:397b-cloud')
    )

    assert provider.generate([{'role': 'user', 'content': 'plan'}]) == '{"steps":[]}'
    assert captured['payload']['think'] is False


def test_ollama_provider_uses_thinking_when_content_is_empty(monkeypatch) -> None:
    captured = {}

    def fake_post_json(url, payload, *, timeout_sec, headers):
        captured['payload'] = payload
        return {'message': {'content': '', 'thinking': '{"steps":[]}'}}

    monkeypatch.setattr('planner_llm.providers._post_json', fake_post_json)
    provider = build_provider(
        PlannerProviderConfig(provider='ollama', model='qwen3.5:397b-cloud')
    )

    assert provider.generate([{'role': 'system', 'content': 'Return JSON only.'}]) == '{"steps":[]}'
    assert captured['payload']['messages'][0]['content'].startswith('/no_think')


def test_planner_engine_accepts_scan_steps_from_provider() -> None:
    provider = _FakeProvider(
        '{"ack_text":"I will look around and report what I find.","steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"perform_motion","args":{"object":"head_look_right"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"scan","args":{"target":"people","target_kind":"people","max_sweeps":2},"requires":[],"on_failure":"replan","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan',
            'goal_id': 'goal_scan',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene'],
            'scene_targets': ['people'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_scan', plan_version=1)

    assert decision.mode == 'plan'
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'perform_motion',
        'perform_motion',
        'scan',
    ]


def test_planner_engine_retries_scan_result_wording_outside_plan() -> None:
    provider = _SequenceProvider(
        [
            '{"ack_text":"I will look around and report what I find.",'
            '"steps":[{"type":"skill","name":"scan","args":{"target":"people"}},'
            '{"type":"say","name":"say","args":{"text":"I found one person."}}]}',
            '{"ack_text":"I will look around and report what I find.",'
            '"steps":[{"type":"skill","name":"scan","args":{"target":"people","target_kind":"people"}}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan_result',
            'goal_id': 'goal_scan_result',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_scan_result', plan_version=1)

    assert len(provider.messages) == 2
    assert 'say steps cannot be mixed with executable steps' in provider.messages[1][1]['content']
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'scan',
    ]
