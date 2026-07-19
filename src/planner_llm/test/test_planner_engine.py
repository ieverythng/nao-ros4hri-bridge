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


def _assert_invalid_planner_failure(decision) -> None:
    plan = decision.payload['plan']
    assert decision.mode == 'fail'
    assert plan['status'] == 'failed'
    assert plan['validation_status'] == 'invalid'
    assert plan['steps'] == []
    assert plan['replan_hint'] == 'planner_invalid_output'


def _assert_structured_clarification(decision, reason_fragment: str = '') -> None:
    plan = decision.payload['plan']
    assert decision.mode == 'clarify'
    assert plan['status'] == 'waiting_user'
    assert plan['steps'] == []
    assert plan['replan_hint'] == 'clarify_user'
    if reason_fragment:
        assert reason_fragment in plan['user_facing_reason']


def _assert_target_selection_recovery(
    decision,
    *,
    skill_names: list[str],
    scene_targets: list[str],
) -> None:
    plan = decision.payload['plan']
    assert decision.mode == 'validated_target_selection_recovery'
    assert plan['validation_status'] == 'valid'
    assert [step['name'] for step in plan['steps']] == skill_names
    assert plan['scene_targets'] == scene_targets


def _step_requirements(decision) -> list[list[str]]:
    return [step['requires'] for step in decision.payload['plan']['steps']]


def test_planner_engine_fails_when_model_cannot_plan_motion() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
            {
                'request_id': 'r1',
                'goal_id': 'goal_1',
                'goal_text': 'look left',
                'normalized_intents': ['head_look_left'],
            }
        )

    decision = engine.plan_request(request, goal_id='goal_1', plan_version=1)
    assert decision.mode == 'fail'
    assert decision.payload['plan']['goal_id'] == 'goal_1'
    assert decision.payload['plan']['plan_version'] == 1
    assert decision.payload['plan']['steps'] == []
    assert decision.payload['plan']['status'] == 'failed'
    assert 'valid executable plan' in decision.payload['plan']['failure_reason']
    assert provider.messages


def test_planner_engine_does_not_rule_fallback_targeted_motion_request() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_targeted_motion',
            'goal_id': 'goal_targeted_motion',
            'goal_text': 'look to the left for the cup',
            'normalized_intents': ['head_look_left'],
            'scene_targets': ['cup'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_targeted_motion', plan_version=1)

    assert decision.mode == 'fail'
    assert decision.payload['plan']['status'] == 'failed'


def test_planner_engine_prefers_provider_over_rule_for_simple_motion_intent() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"perform_motion",'
        '"args":{"object":"head_look_up"},"requires":[],"on_failure":"replan",'
        '"retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_provider_motion',
            'goal_id': 'goal_provider_motion',
            'goal_text': 'move your head up',
            'normalized_intents': ['head_look_up'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_provider_motion', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['args']['object'] == 'head_look_up'
    assert provider.messages[0]['role'] == 'system'


def test_planner_engine_uses_second_retry_after_validation_progress() -> None:
    provider = _SequenceProvider(
        [
            'not json',
            '{"steps":[{"type":"skill","name":"perform_motion","args":{}}]}',
            (
                '{"steps":[{"type":"skill","name":"perform_motion",'
                '"args":{"object":"head_look_up"},"requires":[],"on_failure":"fail"}]}'
            ),
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_progressive_validation_retry',
            'goal_id': 'goal_progressive_validation_retry',
            'goal_text': 'look up',
            'normalized_intents': ['head_look_up'],
        }
    )

    decision = engine.plan_request(request)

    assert len(provider.messages) == 3
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['args']['object'] == 'head_look_up'
    second_retry_prompt = provider.messages[2][1]['content']
    assert 'validation_retry' in second_retry_prompt
    assert 'previous_model_output' in second_retry_prompt
    assert '\\"name\\":\\"perform_motion\\"' in second_retry_prompt


def test_planner_engine_stops_retrying_repeated_invalid_output() -> None:
    provider = _SequenceProvider(
        [
            'not json',
            'not json',
            (
                '{"steps":[{"type":"skill","name":"perform_motion",'
                '"args":{"object":"head_look_up"}}]}'
            ),
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_repeated_validation_retry',
            'goal_id': 'goal_repeated_validation_retry',
            'goal_text': 'look up',
            'normalized_intents': ['head_look_up'],
        }
    )

    decision = engine.plan_request(request)

    assert len(provider.messages) == 2
    _assert_invalid_planner_failure(decision)


def test_planner_engine_retries_function_style_kb_statements_before_execution() -> None:
    provider = _SequenceProvider(
        [
            (
                '{"steps":[{"type":"skill","name":"kb_add",'
                '"args":{"statements":["rdf:type(cup, dbp:RedCup)"]},'
                '"requires":[],"on_failure":"replan"}]}'
            ),
            (
                '{"steps":[{"type":"skill","name":"kb_add",'
                '"args":{"statements":["red_cup rdf:type Cup",'
                '"red_cup dbp:color red"]},"requires":[],"on_failure":"replan"}]}'
            ),
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_kb_statement_retry',
            'goal_id': 'goal_kb_statement_retry',
            'goal_text': 'Add a red cup to your KB.',
            'normalized_intents': ['kb_add'],
        }
    )

    decision = engine.plan_request(request)

    assert len(provider.messages) == 2
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['args']['statements'] == [
        'red_cup rdf:type Cup',
        'red_cup dbp:color red',
    ]
    assert 'subject predicate object' in provider.messages[1][1]['content']
    assert 'namespace-qualified predicate' in provider.messages[1][1]['content']
    assert 'subject first' in provider.messages[1][1]['content']


def test_planner_engine_does_not_rule_fallback_composite_motion_goal_text() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_all_direction_hint',
            'goal_id': 'goal_all_direction_hint',
            'goal_text': 'move your head in all directions',
            'normalized_intents': ['head_look_up'],
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_all_direction_hint',
        plan_version=1,
    )

    assert decision.mode == 'fail'
    assert decision.payload['plan']['status'] == 'failed'
    assert 'planner output did not contain a valid executable plan' in (
        decision.payload['plan']['failure_reason']
    )
    assert provider.messages


def test_planner_engine_uses_provider_for_motion_report_request() -> None:
    provider = _FakeProvider(
        '{"steps":['
        '{"type":"skill","name":"perform_motion","args":{"object":"head_look_right"},'
        '"requires":[],"on_failure":"replan","retry_budget":0},'
        '{"type":"skill","name":"scan","args":{},'
        '"requires":["step_1"],"on_failure":"replan","retry_budget":0},'
        '{"type":"skill","name":"report_result","args":{},'
        '"requires":["step_2"],"on_failure":"fail","retry_budget":0}'
        ']}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_motion_report',
            'goal_id': 'goal_motion_report',
            'goal_text': 'move your head to the right and report what you see',
            'normalized_intents': ['head_look_right', 'inspect_scene', 'report_result'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_motion_report', plan_version=1)

    assert decision.mode == 'plan'
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'perform_motion',
        'scan',
        'report_result',
    ]
    assert provider.messages[0]['role'] == 'system'
    assert 'Keep emit_progress=false for short plans' in provider.messages[0]['content']
    assert 'Routine internal step transitions' in provider.messages[0]['content']


def test_planner_engine_keeps_composite_motion_report_progress_quiet_by_default() -> None:
    provider = _FakeProvider(
        '{"communication_policy":{"emit_progress":false,"emit_completion":true},'
        '"steps":['
        '{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},'
        '"requires":[],"on_failure":"replan","retry_budget":0},'
        '{"type":"skill","name":"perform_motion","args":{"object":"head_look_right"},'
        '"requires":["step_1"],"on_failure":"replan","retry_budget":0},'
        '{"type":"skill","name":"wave_greet","args":{},'
        '"requires":["step_2"],"on_failure":"replan","retry_budget":0},'
        '{"type":"skill","name":"report_result","args":{},'
        '"requires":["step_3"],"on_failure":"fail","retry_budget":0}'
        ']}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=2)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_motion_wave_report',
            'goal_id': 'goal_motion_wave_report',
            'goal_text': 'move your head in all directions and then wave at me',
            'normalized_intents': ['perform_motion', 'wave_greet', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_motion_wave_report',
        plan_version=1,
    )

    policy = decision.payload['plan']['communication_policy']
    assert policy['emit_progress'] is False
    assert policy['emit_completion'] is False
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'perform_motion',
        'perform_motion',
        'wave_greet',
        'report_result',
    ]


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


def test_planner_engine_limits_scene_targets_to_executable_step_scope() -> None:
    provider = _FakeProvider(
        '{"steps":['
        '{"type":"skill","name":"perform_motion","args":{"object":"standinit"}},'
        '{"type":"skill","name":"wave_greet",'
        '"args":{"target_frame":"anonymous_person_current"}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_bounded_person_scope',
            'goal_id': 'goal_bounded_person_scope',
            'goal_text': 'stand up and wave at the nearest person',
            'normalized_intents': ['posture_stand', 'wave_greet'],
            'scene_targets': [
                'anonymous_person_stale_1',
                'anonymous_person_stale_2',
                'anonymous_person_current',
            ],
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.payload['plan']['scene_targets'] == ['anonymous_person_current']


def test_planner_engine_rejects_model_scene_target_for_targetless_motion() -> None:
    provider = _FakeProvider(
        '{"plan":{"scene_targets":["anonymous_person_visible"]},'
        '"steps":['
        '{"type":"skill","name":"perform_motion","args":{"object":"sit"}},'
        '{"type":"skill","name":"report_result","args":{}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_targetless_motion',
            'goal_id': 'goal_targetless_motion',
            'goal_text': 'sit down and report when sitting',
            'normalized_intents': ['posture_sit', 'report_result'],
            'scene_targets': [],
            'grounded_context': {
                'entities': [
                    {
                        'id': 'anonymous_person_visible',
                        'kind': 'person',
                        'class': 'Human',
                    }
                ]
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['scene_targets'] == []


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
            '{"steps":[{"type":"skill","name":"scan","args":{"target_kind":"scene"}},'
            '{"type":"skill","name":"report_result","args":{},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_retry',
            'goal_id': 'goal_retry',
            'goal_text': 'scan the room and tell me what you see',
            'normalized_intents': ['inspect_scene', 'report_result'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_retry', plan_version=1)

    assert decision.mode == 'plan'
    assert len(provider.messages) == 2
    retry_prompt = provider.messages[1][1]['content']
    assert 'validation_retry' in retry_prompt
    assert 'do not mix speech steps into executable plans' in retry_prompt
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'scan',
        'report_result',
    ]


def test_planner_engine_retries_place_object_without_manipulated_object() -> None:
    provider = _SequenceProvider(
        [
            '{"steps":[{"type":"skill","name":"place_object",'
            '"args":{"target":"table_1"},"on_failure":"replan"}]}',
            '{"steps":[{"type":"skill","name":"place_object",'
            '"args":{"object":"apple_1","target":"table_1"},"on_failure":"replan"}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_place',
            'goal_id': 'goal_place',
            'goal_text': 'place the apple on the table',
            'normalized_intents': ['place_object'],
            'scene_targets': ['apple_1', 'table_1'],
        }
    )

    decision = engine.plan_request(request, goal_id='goal_place', plan_version=1)

    assert len(provider.messages) == 2
    assert 'place_object requires target and destination' in provider.messages[1][1]['content']
    assert decision.payload['plan']['steps'][0]['args'] == {
        'target': 'apple_1',
        'destination': 'table_1',
    }


def test_planner_engine_rejects_place_object_without_destination() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"place_object",'
        '"args":{"object_id":"apple_1"},"on_failure":"replan"}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_place_missing_destination',
            'goal_id': 'goal_place_missing_destination',
            'goal_text': 'place the apple',
            'normalized_intents': ['place_object'],
            'scene_targets': ['apple_1'],
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_place_missing_destination',
        plan_version=1,
    )

    assert decision.mode == 'fail'
    assert 'place_object requires target and destination' in (
        decision.payload['plan']['failure_reason']
    )


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
        }
    )

    decision = engine.plan_request(request, goal_id='goal_timeout', plan_version=1)

    assert decision.mode == 'backend_unavailable'
    assert decision.payload['plan']['status'] == 'failed'
    assert decision.payload['plan']['replan_hint'] == 'planner_backend_unavailable'
    assert 'timed out' in decision.raw_model_output


def test_planner_engine_allows_one_grounded_delivery_target_without_selection() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"bring_object",'
        '"args":{"target":"pear_xiuwe","recipient":"myself","source":"Lab"}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_single_grounded_delivery',
            'goal_id': 'goal_single_grounded_delivery',
            'goal_text': 'bring the pear with ID xiuwe',
            'normalized_intents': ['bring_object'],
            'scene_targets': ['pear xiuwe'],
            'grounded_context': {
                'entities': [
                    {
                        'id': 'pear_xiuwe',
                        'label': 'pear',
                        'kind': 'object',
                        'class': 'Fruit',
                        'relations': [{'predicate': 'oro:isAt', 'object': 'Lab'}],
                    },
                    {
                        'id': 'Lab',
                        'label': 'Lab',
                        'kind': 'location',
                        'class': 'Room',
                    },
                ],
                'locations': [
                    {'id': 'Lab', 'label': 'Lab', 'role': 'navigation_target'}
                ],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_single_grounded_delivery',
        plan_version=1,
    )

    assert provider.messages
    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][0]['args']['target'] == 'pear_xiuwe'


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

    _assert_invalid_planner_failure(decision)


def test_planner_engine_recovers_validated_grounded_location_group_delivery() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_kitchen_delivery',
            'goal_id': 'goal_kitchen_delivery',
            'goal_text': 'bring every object from the kitchen to ALEX and report what happened',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'codex_kitchen',
                'member_ids': ['book_1', 'cup_1'],
                'recipient_id': 'person_1',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {
                        'id': 'codex_kitchen',
                        'label': 'kitchen',
                        'kind': 'object',
                        'class': 'Room',
                    },
                    {
                        'id': 'cup_1',
                        'label': 'cup',
                        'kind': 'object',
                        'class': 'Cup',
                        'relations': [{'predicate': 'oro:isIn', 'object': 'codex_kitchen'}],
                    },
                    {
                        'id': 'book_1',
                        'label': 'book',
                        'kind': 'object',
                        'class': 'Book',
                        'relations': [{'predicate': 'oro:isIn', 'object': 'codex_kitchen'}],
                    },
                    {
                        'id': 'person_1',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ],
                'locations': [
                    {
                        'id': 'codex_kitchen',
                        'label': 'kitchen',
                        'class': 'Room',
                        'contains': [
                            {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                            {'id': 'book_1', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                        ],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_kitchen_delivery', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['bring_object', 'bring_object', 'report_result'],
        scene_targets=['book_1', 'cup_1', 'person_1'],
    )
    assert _step_requirements(decision) == [[], [], ['step_1', 'step_2']]


def test_planner_engine_rejects_container_as_delivery_recipient() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_apple_house',
            'goal_id': 'goal_apple_house',
            'goal_text': 'bring the apple to the house',
            'normalized_intents': ['bring_object'],
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'deliver',
                'member_ids': ['apple_1'],
                'recipient_id': 'house_1',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'apple_1', 'label': 'apple', 'kind': 'object', 'class': 'Apple'},
                    {
                        'id': 'house_1',
                        'label': 'house',
                        'kind': 'object',
                        'class': 'cyc:SpatialThing-Localized',
                        'relations': [{'predicate': 'oro:contains', 'object': 'phone_1'}],
                    },
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_apple_house', plan_version=1)

    _assert_structured_clarification(decision, 'grounded person')
    assert provider.messages == []


def test_planner_engine_recovers_validated_location_group_alias_after_invalid_output() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_work_table_delivery',
            'goal_id': 'goal_work_table_delivery',
            'goal_text': 'Bring every object from the work table to ALEX and report what happened.',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'codex_lab_table_section',
                'member_ids': ['cup_1'],
                'recipient_id': 'codex_lab_alex',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {
                        'id': 'codex_lab_room',
                        'label': 'lab',
                        'kind': 'object',
                        'class': 'Room',
                    },
                    {
                        'id': 'codex_lab_handoff_area',
                        'label': 'handoff area',
                        'kind': 'object',
                        'class': 'Place',
                        'relations': [{'predicate': 'oro:isIn', 'object': 'codex_lab_room'}],
                    },
                    {
                        'id': 'codex_gold_recipient',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [
                            {'predicate': 'dbp:name', 'object': 'ALEX'},
                            {'predicate': 'oro:isIn', 'object': 'codex_gold_handoff_area'},
                        ],
                    },
                    {
                        'id': 'codex_lab_alex',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [
                            {'predicate': 'dbp:name', 'object': 'ALEX'},
                            {'predicate': 'oro:isIn', 'object': 'codex_lab_handoff_area'},
                        ],
                    },
                ],
                'locations': [
                    {
                        'id': 'codex_gold_table',
                        'label': 'table',
                        'class': 'Table',
                        'contains': [{'id': 'gold_apple', 'label': 'apple', 'kind': 'object'}],
                    },
                    {
                        'id': 'table',
                        'label': 'table',
                        'contains': [{'id': 'phone_1', 'label': 'phone', 'kind': 'object'}],
                    },
                    {
                        'id': 'codex_lab_table_section',
                        'label': 'work_table',
                        'class': 'Table',
                        'relations': [{'predicate': 'oro:isIn', 'object': 'codex_lab_room'}],
                        'contains': [{'id': 'cup_1', 'label': 'cup', 'kind': 'object'}],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_work_table_delivery', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['bring_object', 'report_result'],
        scene_targets=['cup_1', 'codex_lab_alex'],
    )


def test_planner_engine_recovers_validated_grouped_delivery_after_invalid_output() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_structured_delivery',
            'goal_id': 'goal_structured_delivery',
            'goal_text': 'handle the requested collection',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'work_table',
                'member_ids': ['book_1', 'cup_1'],
                'recipient_id': 'person_1',
                'ordering': 'none',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'book_1', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                    {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'person_1', 'label': 'ALEX', 'kind': 'person', 'class': 'Human'},
                ],
                'locations': [
                    {
                        'id': 'work_table',
                        'label': 'work table',
                        'role': 'support_group',
                        'contains': [
                            {'id': 'book_1', 'label': 'book', 'kind': 'object'},
                            {'id': 'cup_1', 'label': 'cup', 'kind': 'object'},
                        ],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['bring_object', 'bring_object', 'report_result'],
        scene_targets=['book_1', 'cup_1', 'person_1'],
    )


def test_planner_engine_does_not_bypass_invalid_selection_with_phrase_fallback() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_invalid_structured_delivery',
            'goal_id': 'goal_invalid_structured_delivery',
            'goal_text': 'bring every object from the kitchen to ALEX',
            'normalized_intents': ['bring_object'],
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'kitchen',
                'member_ids': ['cup_1', 'person_1'],
                'recipient_id': 'person_1',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'person_1', 'label': 'ALEX', 'kind': 'person', 'class': 'Human'},
                ],
                'locations': [
                    {
                        'id': 'kitchen',
                        'label': 'kitchen',
                        'contains': [{'id': 'cup_1', 'kind': 'object'}],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'clarify'
    assert all(
        step.get('name') != 'bring_object'
        for step in decision.payload['plan']['steps']
    )
    assert provider.messages == []


def test_planner_engine_rejects_empty_delivery_selection_before_model() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"bring_object",'
        '"args":{"target":"cup_1","recipient":"kitchen"}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_empty_delivery_selection',
            'goal_id': 'goal_empty_delivery_selection',
            'goal_text': 'bring every visible object to ALEX',
            'normalized_intents': ['bring_object'],
            'target_selection': {
                'selection_kind': 'visible_objects',
                'operation': 'deliver',
                'member_ids': [],
                'recipient_id': 'person_1',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'person_1', 'label': 'ALEX', 'kind': 'person'},
                ],
                'locations': [
                    {'id': 'kitchen', 'label': 'kitchen', 'kind': 'location'},
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'clarify'
    assert provider.messages == []


def test_planner_engine_stops_selection_required_request_before_model_without_selection() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_unbounded_visit',
            'goal_id': 'goal_unbounded_visit',
            'goal_text': 'perform the requested object sequence',
            'normalized_intents': ['navigate_to'],
            'scene_targets': ['cup_1', 'person_1'],
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'person_1', 'kind': 'person', 'class': 'Human'},
                ]
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    _assert_structured_clarification(decision, 'complete grounded target selection')
    assert provider.messages == []


def test_planner_engine_does_not_require_object_selection_for_person_navigation() -> None:
    provider = _FakeProvider(
        '{"steps":['
        '{"type":"skill","name":"navigate_to","args":{"target":"person_1"}},'
        '{"type":"skill","name":"wave_greet","args":{"target":"person_1"}},'
        '{"type":"skill","name":"report_result","args":{}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_person_wave',
            'goal_id': 'goal_person_wave',
            'goal_text': 'go to ALEX, wave, and report',
            'normalized_intents': ['navigate_to', 'wave_greet', 'report_result'],
            'scene_targets': ['person_1'],
            'grounded_context': {
                'entities': [
                    {'id': 'person_1', 'label': 'ALEX', 'kind': 'person', 'class': 'Human'},
                ]
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'plan'
    assert provider.messages


def test_planner_engine_recovers_validated_ordered_visit_after_invalid_output() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_structured_visit',
            'goal_id': 'goal_structured_visit',
            'goal_text': 'perform the requested sequence',
            'normalized_intents': ['navigate_to', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'visible_objects',
                'operation': 'visit',
                'member_ids': ['apple_1', 'book_1'],
                'ordering': 'sequential',
                'report_policy': 'per_target',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'apple_1', 'label': 'apple', 'kind': 'object', 'class': 'Apple'},
                    {'id': 'book_1', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                ]
            },
        }
    )

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['navigate_to', 'report_result', 'navigate_to', 'report_result'],
        scene_targets=['apple_1', 'book_1'],
    )


def test_planner_engine_recovers_validated_compact_room_after_invalid_output() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_iiia_kitchen_delivery',
            'goal_id': 'goal_iiia_kitchen_delivery',
            'goal_text': 'Bring every object from the kitchen to ALEX and report what happened.',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'codex_iiia_kitchen',
                'member_ids': ['codex_iiia_cup'],
                'recipient_id': 'codex_iiia_alex',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'codex_iiia_cup', 'label': 'IIIA_CUP', 'kind': 'object', 'class': 'Cup'},
                    {
                        'id': 'codex_iiia_alex',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ],
                'locations': [
                    {
                        'id': 'codex_iiia_kitchen',
                        'label': 'codex_iiia_kitchen',
                        'class': 'Room',
                        'role': 'navigation_target',
                        'contains': [
                            {
                                'id': 'codex_iiia_cup',
                                'label': 'IIIA_CUP',
                                'kind': 'object',
                                'class': 'Cup',
                                'relation': 'oro:isIn',
                            }
                        ],
                    },
                    {
                        'id': 'codex_iiia_kitchen_table',
                        'label': 'codex_iiia_kitchen',
                        'class': 'Table',
                        'role': 'support_group',
                        'contains': [],
                    },
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_iiia_kitchen_delivery', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['bring_object', 'report_result'],
        scene_targets=['codex_iiia_cup', 'codex_iiia_alex'],
    )


def test_planner_engine_recovers_members_after_model_targets_container() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"bring_object","args":{"target":"codex_lab_table_section","recipient":"person_1","source":"codex_lab_table_section"}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_work_table_container_plan',
            'goal_id': 'goal_work_table_container_plan',
            'goal_text': 'Bring every object from the work table to ALEX and report what happened.',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'deliver',
                'source_location_id': 'codex_lab_table_section',
                'member_ids': ['codex_lab_cup', 'codex_lab_phone'],
                'recipient_id': 'person_1',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'codex_lab_cup', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'codex_lab_phone', 'label': 'phone', 'kind': 'object', 'class': 'Phone'},
                    {
                        'id': 'person_1',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ],
                'locations': [
                    {
                        'id': 'codex_lab_table_section',
                        'label': 'codex_lab_table_section',
                        'aliases': ['work_table'],
                        'class': 'Table',
                        'contains': [
                            {'id': 'codex_lab_cup', 'label': 'cup', 'kind': 'object'},
                            {'id': 'codex_lab_phone', 'label': 'phone', 'kind': 'object'},
                        ],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_work_table_container_plan',
        plan_version=1,
    )

    _assert_target_selection_recovery(
        decision,
        skill_names=['bring_object', 'bring_object', 'report_result'],
        scene_targets=['codex_lab_cup', 'codex_lab_phone', 'person_1'],
    )


def test_planner_engine_recovers_validated_ordered_walk_after_invalid_json() -> None:
    provider = _FakeProvider('I cannot format a plan right now.')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_ordered_walk',
            'goal_id': 'goal_ordered_walk',
            'goal_text': 'Walk to every object on the table and let me know when you get to each one.',
            'normalized_intents': ['navigate_to', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'visit',
                'source_location_id': 'table_1',
                'member_ids': ['apple_1', 'book_1'],
                'ordering': 'sequential',
                'report_policy': 'per_target',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'apple_1', 'label': 'apple', 'kind': 'object', 'class': 'Apple'},
                    {'id': 'book_1', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                ],
                'locations': [
                    {
                        'id': 'table_1',
                        'label': 'table',
                        'class': 'Table',
                        'contains': [
                            {'id': 'apple_1', 'label': 'apple', 'kind': 'object'},
                            {'id': 'book_1', 'label': 'book', 'kind': 'object'},
                        ],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_ordered_walk', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['navigate_to', 'report_result', 'navigate_to', 'report_result'],
        scene_targets=['apple_1', 'book_1'],
    )


def test_ordered_walk_recovery_publishes_members_without_support_locations() -> None:
    provider = _FakeProvider('not json')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_ordered_walk_support_groups',
            'goal_id': 'goal_ordered_walk_support_groups',
            'goal_text': (
                'Walk to every object on the table and let me know when you get to each one.'
            ),
            'normalized_intents': ['navigate_to', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'location_members',
                'operation': 'visit',
                'source_location_id': 'codex_base_table',
                'member_ids': [
                    'codex_probe_book',
                    'codex_probe_cup',
                    'codex_probe_phone',
                ],
                'ordering': 'sequential',
                'report_policy': 'per_target',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'codex_base_table', 'label': 'table', 'kind': 'object', 'class': 'Table'},
                    {'id': 'codex_lab_table', 'label': 'table', 'kind': 'object', 'class': 'Table'},
                    {'id': 'codex_probe_book', 'label': 'book', 'kind': 'object', 'class': 'Book'},
                    {'id': 'codex_probe_cup', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'codex_probe_phone', 'label': 'phone', 'kind': 'object', 'class': 'Phone'},
                ],
                'locations': [
                    {
                        'id': 'codex_base_table',
                        'label': 'table',
                        'class': 'Table',
                        'role': 'support_group',
                        'contains': [
                            {
                                'id': 'codex_probe_book',
                                'label': 'book',
                                'kind': 'object',
                                'class': 'Book',
                            },
                            {
                                'id': 'codex_probe_cup',
                                'label': 'cup',
                                'kind': 'object',
                                'class': 'Cup',
                            },
                            {
                                'id': 'codex_probe_phone',
                                'label': 'phone',
                                'kind': 'object',
                                'class': 'Phone',
                            },
                        ],
                    },
                    {
                        'id': 'codex_lab_table',
                        'label': 'work table',
                        'class': 'Table',
                        'role': 'support_group',
                        'contains': [],
                    },
                ],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_ordered_walk_support_groups',
        plan_version=1,
    )

    _assert_target_selection_recovery(
        decision,
        skill_names=[
            'navigate_to',
            'report_result',
            'navigate_to',
            'report_result',
            'navigate_to',
            'report_result',
        ],
        scene_targets=['codex_probe_book', 'codex_probe_cup', 'codex_probe_phone'],
    )


def test_planner_engine_clarifies_location_group_delivery_without_recipient() -> None:
    engine = _engine_for_response('{}', retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_kitchen_delivery_missing_recipient',
            'goal_id': 'goal_kitchen_delivery_missing_recipient',
            'goal_text': 'bring every object from the kitchen',
            'normalized_intents': ['bring_object'],
            'planner_mode': 'multi_step',
            'grounded_context': {
                'locations': [
                    {
                        'id': 'codex_kitchen',
                        'label': 'kitchen',
                        'contains': [{'id': 'cup_1', 'label': 'cup'}],
                    }
                ],
                'entities': [{'id': 'cup_1', 'label': 'cup', 'kind': 'object'}],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_kitchen_delivery_missing_recipient',
        plan_version=1,
    )

    _assert_structured_clarification(decision, 'complete grounded target selection')


def test_planner_engine_clarifies_location_group_delivery_when_named_person_is_absent() -> None:
    engine = _engine_for_response('{}', retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_table_delivery_missing_named_person',
            'goal_id': 'goal_table_delivery_missing_named_person',
            'goal_text': 'Bring every object from the work table to the person named BLAKE and report what happened.',
            'normalized_intents': ['bring_object', 'report_result'],
            'planner_mode': 'multi_step',
            'grounded_context': {
                'locations': [
                    {
                        'id': 'codex_lab_table_section',
                        'label': 'work_table',
                        'class': 'Table',
                        'contains': [
                            {
                                'id': 'codex_lab_book',
                                'label': 'book',
                                'kind': 'object',
                                'class': 'Book',
                            },
                            {
                                'id': 'codex_lab_cup',
                                'label': 'cup',
                                'kind': 'object',
                                'class': 'Cup',
                            },
                        ],
                    }
                ],
                'entities': [
                    {'id': 'codex_lab_book', 'label': 'book', 'kind': 'object'},
                    {'id': 'codex_lab_cup', 'label': 'cup', 'kind': 'object'},
                    {
                        'id': 'codex_lab_alex',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_table_delivery_missing_named_person',
        plan_version=1,
    )

    _assert_structured_clarification(decision, 'complete grounded target selection')


def test_planner_engine_does_not_compile_grounded_look_after_invalid_output() -> None:
    engine = _engine_for_response('{}', retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_look_report',
            'goal_id': 'goal_look_report',
            'goal_text': 'Look at the probe cup and tell me what you did.',
            'normalized_intents': [],
            'planner_mode': 'single_step',
            'grounded_context': {
                'entities': [
                    {
                        'id': 'codex_probe_cup',
                        'label': 'cup',
                        'kind': 'object',
                        'class': 'Cup',
                        'relations': [
                            {'predicate': 'dbp:name', 'object': 'TITAS'},
                            {'predicate': 'dbp:color', 'object': 'gold'},
                        ],
                    }
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_look_report', plan_version=1)

    _assert_invalid_planner_failure(decision)


def test_planner_engine_clarifies_grounded_look_when_target_is_ambiguous() -> None:
    engine = _engine_for_response('{}', retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_look_report_ambiguous',
            'goal_id': 'goal_look_report_ambiguous',
            'goal_text': 'Look at the cup and tell me what you did.',
            'normalized_intents': [],
            'planner_mode': 'single_step',
            'grounded_context': {
                'entities': [
                    {'id': 'cup_1', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'cup_2', 'label': 'cup', 'kind': 'object', 'class': 'Cup'},
                ],
            },
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_look_report_ambiguous',
        plan_version=1,
    )

    _assert_invalid_planner_failure(decision)


def test_planner_engine_repairs_unsupported_composite_motion_object() -> None:
    provider = _SequenceProvider(
        [
            '{"steps":[{"type":"skill","name":"perform_motion",'
            '"args":{"object":"head_look_all"},"requires":[],'
            '"on_failure":"replan","retry_budget":0}]}',
            '{"steps":['
            '{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},'
            '"requires":[],"on_failure":"replan","retry_budget":0},'
            '{"type":"skill","name":"perform_motion","args":{"object":"head_look_right"},'
            '"requires":[],"on_failure":"replan","retry_budget":0},'
            '{"type":"skill","name":"perform_motion","args":{"object":"head_look_up"},'
            '"requires":[],"on_failure":"replan","retry_budget":0},'
            '{"type":"skill","name":"perform_motion","args":{"object":"head_look_down"},'
            '"requires":[],"on_failure":"replan","retry_budget":0}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_all_directions',
            'goal_id': 'goal_all_directions',
            'goal_text': 'move your head in all directions',
            'normalized_intents': ['perform_motion'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_all_directions', plan_version=1)

    assert decision.mode == 'plan'
    assert len(provider.messages) == 2
    retry_prompt = provider.messages[1][1]['content']
    assert 'unsupported perform_motion args.object \\"head_look_all\\"' in retry_prompt
    assert [step['args']['object'] for step in decision.payload['plan']['steps']] == [
        'head_look_left',
        'head_look_right',
        'head_look_up',
        'head_look_down',
    ]


def test_planner_engine_recovers_normalized_motion_sequence_after_retry_exhaustion() -> None:
    invalid_output = (
        '{"steps":[{"type":"skill","name":"perform_motion",'
        '"args":{"object":"head_look_all"}}]}'
    )
    provider = _SequenceProvider([invalid_output, invalid_output, invalid_output])
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_all_directions_recovery',
            'goal_id': 'goal_all_directions_recovery',
            'goal_text': 'move your head in all directions',
            'normalized_intents': [
                'head_look_left',
                'head_look_right',
                'head_look_up',
                'head_look_down',
            ],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(
        request,
        goal_id='goal_all_directions_recovery',
        plan_version=1,
    )

    assert decision.mode == 'validated_motion_sequence_recovery'
    assert len(provider.messages) == 2
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'perform_motion',
        'perform_motion',
        'perform_motion',
        'perform_motion',
        'report_result',
    ]
    assert [
        step['args'].get('object')
        for step in decision.payload['plan']['steps'][:4]
    ] == [
        'head_look_left',
        'head_look_right',
        'head_look_up',
        'head_look_down',
    ]
    assert decision.payload['plan']['steps'][-1]['requires'] == [
        'step_1',
        'step_2',
        'step_3',
        'step_4',
    ]


def test_planner_engine_strips_prefilled_report_summary_after_scan() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"scan","args":{},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"report_result","args":{"summary_text":"I can see a cup."},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan_report',
            'goal_id': 'goal_scan_report',
            'goal_text': 'look around and report what is visible',
            'normalized_intents': ['inspect_scene', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_scan_report', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][1]['args'] == {}


def test_planner_engine_strips_prefilled_report_summary_after_motion() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"report_result","args":{"summary_text":"I have moved my head in all directions as requested."},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_motion_report',
            'goal_id': 'goal_motion_report',
            'goal_text': 'move your head in all directions and report when done',
            'normalized_intents': ['perform_motion', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_motion_report', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][1]['args'] == {}


def test_planner_engine_strips_prefilled_report_summary_after_navigation() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"navigate_to","args":{"target":"apple_1"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"report_result","args":{"summary_text":"I navigated to the apple."},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_nav_report',
            'goal_id': 'goal_nav_report',
            'goal_text': 'navigate to the apple and report when done',
            'normalized_intents': ['navigate_to', 'report_result'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['apple_1'],
                'ordering': 'sequential',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'apple_1', 'label': 'apple', 'kind': 'object', 'class': 'Apple'},
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_nav_report', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][1]['args'] == {}


def test_planner_engine_accepts_report_result_after_scan_without_summary_text() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"scan","args":{},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"report_result","args":{},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan_report_ok',
            'goal_id': 'goal_scan_report_ok',
            'goal_text': 'look around and report what is visible',
            'normalized_intents': ['inspect_scene', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_scan_report_ok', plan_version=1)

    assert decision.mode == 'plan'
    assert decision.payload['plan']['steps'][1]['args'] == {}


def test_planner_engine_retries_when_requested_report_is_missing() -> None:
    provider = _SequenceProvider(
        [
            '{"steps":[{"type":"skill","name":"scan","args":{},'
            '"requires":[],"on_failure":"replan","retry_budget":0}]}',
            '{"steps":[{"type":"skill","name":"scan","args":{},'
            '"requires":[],"on_failure":"replan","retry_budget":0},'
            '{"type":"skill","name":"report_result","args":{},'
            '"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_missing_report',
            'goal_id': 'goal_missing_report',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_missing_report', plan_version=1)

    assert len(provider.messages) == 2
    assert 'request asks for a user-facing report' in provider.messages[1][1]['content']
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'scan',
        'report_result',
    ]


def test_planner_engine_does_not_infer_report_requirement_from_goal_text() -> None:
    provider = _FakeProvider(
        '{"steps":[{"type":"skill","name":"scan","args":{},'
        '"requires":[],"on_failure":"replan","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_no_report_intent',
            'goal_id': 'goal_no_report_intent',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_no_report_intent', plan_version=1)

    assert decision.mode == 'plan'
    assert [step['name'] for step in decision.payload['plan']['steps']] == ['scan']


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
    _assert_structured_clarification(decision)


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


def test_planner_engine_fails_when_model_output_is_invalid() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_invalid',
            'goal_id': 'goal_invalid',
            'user_text': 'look up and then sit down',
            'goal_text': 'look up and then sit down',
            'normalized_intents': ['inspect_scene'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_invalid', plan_version=1)

    assert decision.mode == 'fail'


def test_planner_engine_rejects_invalid_model_output_without_hints() -> None:
    provider = _FakeProvider('{}')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_invalid_partial',
            'goal_id': 'goal_invalid_partial',
            'goal_text': 'stand up and dance',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_invalid_partial', plan_version=1)

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


def test_openai_provider_disables_thinking_for_namespaced_qwen_model(monkeypatch) -> None:
    captured = {}

    def fake_post_json(url, payload, *, timeout_sec, headers):
        captured['url'] = url
        captured['payload'] = payload
        return {'choices': [{'message': {'content': '{"steps":[]}'}}]}

    monkeypatch.setattr('planner_llm.providers._post_json', fake_post_json)
    provider = build_provider(
        PlannerProviderConfig(
            provider='openai_compatible',
            model='cyankiwi/Qwen3.5-35B-A3B-AWQ-4bit',
            base_url='http://10.7.138.215:8004',
            top_p=0.8,
            top_k=20,
            min_p=0.0,
            presence_penalty=1.5,
            repetition_penalty=1.0,
        )
    )

    assert provider.generate([{'role': 'user', 'content': 'plan'}]) == '{"steps":[]}'
    assert captured['payload']['chat_template_kwargs'] == {'enable_thinking': False}
    assert captured['payload']['top_p'] == 0.8
    assert captured['payload']['top_k'] == 20
    assert captured['payload']['min_p'] == 0.0
    assert captured['payload']['presence_penalty'] == 1.5
    assert captured['payload']['repetition_penalty'] == 1.0


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
        '{"ack_text":"I will look around and report what I find.","steps":[{"type":"skill","name":"perform_motion","args":{"object":"head_look_left"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"perform_motion","args":{"object":"head_look_right"},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"scan","args":{"target":"people","target_kind":"people","max_sweeps":2},"requires":[],"on_failure":"replan","retry_budget":0},{"type":"skill","name":"report_result","args":{},"requires":["step_3"],"on_failure":"fail","retry_budget":0}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan',
            'goal_id': 'goal_scan',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene', 'report_result'],
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
        'report_result',
    ]


def test_planner_engine_retries_scan_result_wording_outside_plan() -> None:
    provider = _SequenceProvider(
        [
            '{"ack_text":"I will look around and report what I find.",'
            '"steps":[{"type":"skill","name":"scan","args":{"target":"people"}},'
            '{"type":"say","name":"say","args":{"text":"I found one person."}}]}',
            '{"ack_text":"I will look around and report what I find.",'
            '"steps":[{"type":"skill","name":"scan","args":{"target":"people","target_kind":"people"}},'
            '{"type":"skill","name":"report_result","args":{},"requires":["step_1"],"on_failure":"fail","retry_budget":0}]}',
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_scan_result',
            'goal_id': 'goal_scan_result',
            'goal_text': 'look around and tell me what you see',
            'normalized_intents': ['inspect_scene', 'report_result'],
            'planner_mode': 'multi_step',
        }
    )

    decision = engine.plan_request(request, goal_id='goal_scan_result', plan_version=1)

    assert len(provider.messages) == 2
    assert 'say steps cannot be mixed with executable steps' in provider.messages[1][1]['content']
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'scan',
        'report_result',
    ]


def test_invalid_model_recovers_validated_navigation_to_person() -> None:
    provider = _FakeProvider('not json')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_nav_person',
            'goal_id': 'goal_nav_person',
            'goal_text': 'walk to person edcca',
            'normalized_intents': ['navigate_to'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['anonymous_person_edcca'],
                'ordering': 'sequential',
                'report_policy': 'none',
            },
            'grounded_context': {
                'entities': [
                    {
                        'id': 'anonymous_person_edcca',
                        'label': 'anonymous_person_edcca',
                        'kind': 'person',
                        'class': 'Human',
                    }
                ]
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_nav_person', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['navigate_to'],
        scene_targets=['anonymous_person_edcca'],
    )


def test_invalid_model_recovers_validated_navigation_to_location() -> None:
    provider = _FakeProvider('not json')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = PlannerRequest.from_payload(
        {
            'request_id': 'r_nav_location',
            'goal_id': 'goal_nav_location',
            'goal_text': 'walk to the kitchen',
            'normalized_intents': ['navigate_to'],
            'planner_mode': 'multi_step',
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['kitchen'],
                'ordering': 'sequential',
                'report_policy': 'none',
            },
            'grounded_context': {
                'entities': [],
                'locations': [
                    {
                        'id': 'kitchen',
                        'label': 'kitchen',
                        'contains': [
                            {'id': 'cup_1', 'kind': 'object', 'class': 'Cup'},
                        ],
                    },
                ],
            },
        }
    )

    decision = engine.plan_request(request, goal_id='goal_nav_location', plan_version=1)

    _assert_target_selection_recovery(
        decision,
        skill_names=['navigate_to'],
        scene_targets=['kitchen'],
    )


def test_target_selection_recovery_rejects_incomplete_rich_objective() -> None:
    provider = _FakeProvider('not json')
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = _rich_visit_request()

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    _assert_invalid_planner_failure(decision)
    assert 'incomplete requested action coverage' in decision.payload['plan']['failure_reason']


def test_planner_accepts_rich_objective_when_all_action_capabilities_are_ordered() -> None:
    provider = _FakeProvider(
        '{"steps":['
        '{"type":"skill","name":"perform_motion","args":{"object":"stand"}},'
        '{"type":"skill","name":"navigate_to","args":{"target":"book_1"}},'
        '{"type":"skill","name":"look_at","args":{"target_frame":"book_1"}},'
        '{"type":"skill","name":"wave_greet","args":{"target":"person_alex"}},'
        '{"type":"skill","name":"perform_motion","args":{"object":"sit"}},'
        '{"type":"skill","name":"report_result","args":{}}]}'
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = _rich_visit_request()

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'plan'
    assert [step['name'] for step in decision.payload['plan']['steps']] == [
        'perform_motion',
        'navigate_to',
        'look_at',
        'wave_greet',
        'perform_motion',
        'report_result',
    ]


def test_planner_allows_second_validation_retry_after_partial_target_coverage() -> None:
    provider = _SequenceProvider(
        [
            _all_objects_plan_response(['book_1']),
            _all_objects_plan_response(['book_1', 'cup_1']),
            _all_objects_plan_response(['book_1', 'cup_1', 'apple_1']),
        ]
    )
    engine = PlannerEngine(provider, SkillRegistry.load(), default_retry_budget=1)
    request = _rich_all_objects_request()

    decision = engine.plan_request(request, goal_id=request.goal_id, plan_version=1)

    assert decision.mode == 'plan'
    assert len(provider.messages) == 3
    assert [
        step['args']['target_frame']
        for step in decision.payload['plan']['steps']
        if step['name'] == 'look_at'
    ] == ['book_1', 'cup_1', 'apple_1']


def _rich_visit_request() -> PlannerRequest:
    return PlannerRequest.from_payload(
        {
            'request_id': 'r_rich_visit',
            'goal_id': 'goal_rich_visit',
            'goal_text': (
                'stand, walk to the book, look at it, wave to ALEX, sit, and summarize'
            ),
            'normalized_intents': [
                'posture_stand',
                'navigate_to',
                'look_at',
                'wave_greet',
                'posture_sit',
                'report_result',
            ],
            'planner_mode': 'multi_step',
            'scene_targets': ['book_1', 'person_alex'],
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['book_1'],
                'ordering': 'sequential',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'book_1', 'label': 'MIDAS', 'kind': 'object', 'class': 'Book'},
                    {
                        'id': 'person_alex',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ]
            },
        }
    )


def _rich_all_objects_request() -> PlannerRequest:
    return PlannerRequest.from_payload(
        {
            'request_id': 'r_rich_all_objects',
            'goal_id': 'goal_rich_all_objects',
            'goal_text': (
                'stand, walk to each visible object, look at each one, wave to ALEX, '
                'sit, and summarize'
            ),
            'normalized_intents': [
                'posture_stand',
                'navigate_to',
                'look_at',
                'wave_greet',
                'posture_sit',
                'report_result',
            ],
            'planner_mode': 'multi_step',
            'scene_targets': ['book_1', 'cup_1', 'apple_1', 'person_alex'],
            'target_selection': {
                'selection_kind': 'explicit_members',
                'operation': 'visit',
                'member_ids': ['book_1', 'cup_1', 'apple_1'],
                'ordering': 'sequential',
                'report_policy': 'final',
            },
            'grounded_context': {
                'entities': [
                    {'id': 'book_1', 'label': 'MIDAS', 'kind': 'object', 'class': 'Book'},
                    {'id': 'cup_1', 'label': 'TITAS', 'kind': 'object', 'class': 'Cup'},
                    {'id': 'apple_1', 'label': 'ATLAS', 'kind': 'object', 'class': 'Apple'},
                    {
                        'id': 'person_alex',
                        'label': 'ALEX',
                        'kind': 'person',
                        'class': 'Human',
                        'relations': [{'predicate': 'dbp:name', 'object': 'ALEX'}],
                    },
                ]
            },
        }
    )


def _all_objects_plan_response(look_targets: list[str]) -> str:
    steps = [
        '{"type":"skill","name":"perform_motion","args":{"object":"stand"}}',
    ]
    for target in ('book_1', 'cup_1', 'apple_1'):
        steps.append(
            '{"type":"skill","name":"navigate_to","args":{"target":"%s"}}' % target
        )
        if target in look_targets:
            steps.append(
                '{"type":"skill","name":"look_at","args":{"target_frame":"%s"}}'
                % target
            )
    steps.extend(
        [
            '{"type":"skill","name":"wave_greet","args":{"target":"person_alex"}}',
            '{"type":"skill","name":"perform_motion","args":{"object":"sit"}}',
            '{"type":"skill","name":"report_result","args":{}}',
        ]
    )
    return '{"steps":[%s]}' % ','.join(steps)
