from dataclasses import dataclass

from nao_chatbot.chat_config import ChatSkillConfig
from nao_chatbot.chat_turn_engine import ChatTurnEngine


@dataclass
class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _Transport:
    def __init__(self, responses):
        self.responses = list(responses)

    def query(self, **_kwargs):
        if not self.responses:
            return ""
        return self.responses.pop(0)


class _Goal:
    is_cancel_requested = False


def _config(mode: str) -> ChatSkillConfig:
    return ChatSkillConfig(
        action_name="/skill/chat",
        role_name="__default__",
        enabled=True,
        ollama_url="http://localhost:11434/api/chat",
        model="model-a",
        intent_model="model-b",
        request_timeout_sec=20.0,
        first_request_timeout_sec=60.0,
        intent_request_timeout_sec=10.0,
        context_window_tokens=4096,
        temperature=0.2,
        top_p=0.9,
        fallback_response="fallback response",
        max_history_messages=12,
        robot_name="Pop",
        persona_prompt_path="",
        system_prompt="sys",
        response_prompt_addendum="resp addendum",
        intent_prompt_addendum="intent addendum",
        environment_description="env",
        response_schema={"type": "object"},
        intent_schema={"type": "object"},
        identity_reminder_every_n_turns=6,
        intent_detection_mode=mode,
        prompt_pack_path="",
        use_skill_catalog=False,
        skill_catalog_packages=[],
        skill_catalog_max_entries=16,
        skill_catalog_max_chars=3000,
    )


def _feedback(_goal, _status, _progress):
    return None


def test_two_stage_success_llm_mode() -> None:
    engine = ChatTurnEngine(
        config=_config("llm"),
        transport=_Transport(
            [
                '{"verbal_ack":"Sure, I can do that."}',
                '{"user_intent":{"type":"posture_stand"},"intent_confidence":0.8}',
            ]
        ),
        logger=_Logger(),
        skill_catalog_text="",
    )

    result = engine.execute_turn(
        goal_handle=_Goal(),
        user_text="please stand up",
        history=[],
        user_id="u1",
        publish_feedback=_feedback,
    )

    assert result.success is True
    assert result.verbal_ack == "Sure, I can do that."
    assert result.intent == "posture_stand"
    assert result.intent_source == "llm_intent"


def test_stage2_failure_keeps_verbal_and_uses_rules_in_fallback_mode() -> None:
    engine = ChatTurnEngine(
        config=_config("llm_with_rules_fallback"),
        transport=_Transport(['{"verbal_ack":"Okay."}', "not json" ]),
        logger=_Logger(),
        skill_catalog_text="",
    )

    result = engine.execute_turn(
        goal_handle=_Goal(),
        user_text="please sit down",
        history=[],
        user_id="u1",
        publish_feedback=_feedback,
    )

    assert result.success is True
    assert result.verbal_ack == "Okay."
    assert result.intent == "posture_sit"
    assert result.intent_source == "rules_llm_intent_fallback"


def test_stage1_failure_in_llm_mode_returns_failure_response() -> None:
    engine = ChatTurnEngine(
        config=_config("llm"),
        transport=_Transport([""]),
        logger=_Logger(),
        skill_catalog_text="",
    )

    result = engine.execute_turn(
        goal_handle=_Goal(),
        user_text="hello",
        history=[],
        user_id="u1",
        publish_feedback=_feedback,
    )

    assert result.success is False
    assert result.verbal_ack == "fallback response"
    assert result.intent == "fallback"
    assert result.intent_source == "llm_response_failed"
