#!/usr/bin/env python3
"""Two-stage turn execution policy for chat skill server."""

from __future__ import annotations

from dataclasses import dataclass
import json

from nao_chatbot.chat_config import ChatSkillConfig
from nao_chatbot.chat_config import coerce_float
from nao_chatbot.chat_goal_codec import coerce_user_intent
from nao_chatbot.chat_goal_codec import extract_json_object
from nao_chatbot.chat_history import history_to_messages
from nao_chatbot.chat_history import messages_to_history
from nao_chatbot.chat_history import trim_messages
from nao_chatbot.chat_prompts import build_intent_prompt
from nao_chatbot.chat_prompts import build_response_prompt
from nao_chatbot.chat_prompts import load_persona_prompt
from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import normalize_intent


@dataclass(frozen=True)
class TurnExecutionResult:
    """Result tuple returned by ChatTurnEngine.execute_turn."""

    success: bool
    verbal_ack: str
    updated_history: list[str]
    intent: str
    intent_source: str
    intent_confidence: float
    user_intent: dict


class ChatTurnEngine:
    """Implement rules/LLM policies and two-stage model flow."""

    def __init__(
        self,
        config: ChatSkillConfig,
        transport,
        logger,
        skill_catalog_text: str,
    ) -> None:
        self._config = config
        self._transport = transport
        self._logger = logger
        self._skill_catalog_text = str(skill_catalog_text or "").strip()
        self._persona_prompt = load_persona_prompt(config.persona_prompt_path, logger=logger)
        self._handled_requests = 0

    def execute_turn(
        self,
        goal_handle,
        user_text: str,
        history: list[str],
        user_id: str,
        publish_feedback,
    ) -> TurnExecutionResult:
        """Execute one full turn with policy/fallback logic."""
        publish_feedback(goal_handle, "thinking", 0.1)
        if goal_handle.is_cancel_requested:
            return self._cancelled_result(history)

        if self._config.intent_detection_mode == "rules":
            result = self._execute_rule_turn(user_text=user_text, history=history, source="rules")
            publish_feedback(goal_handle, "complete", 1.0)
            return result

        if not self._config.enabled:
            if self._config.intent_detection_mode == "llm_with_rules_fallback":
                result = self._execute_rule_turn(
                    user_text=user_text,
                    history=history,
                    source="rules_llm_disabled",
                )
                publish_feedback(goal_handle, "complete", 1.0)
                return result
            result = self._execute_disabled_turn(user_text=user_text, history=history)
            publish_feedback(goal_handle, "complete", 1.0)
            return result

        history_messages = history_to_messages(
            history,
            max_history_messages=self._config.max_history_messages,
        )
        history_messages = self._inject_identity_reminder(history_messages)

        publish_feedback(goal_handle, "generating_response", 0.35)
        if goal_handle.is_cancel_requested:
            return self._cancelled_result(history)

        stage1_timeout_sec = (
            self._config.first_request_timeout_sec
            if self._handled_requests == 0
            else self._config.request_timeout_sec
        )
        response_payload = self._query_response(
            history_messages=history_messages,
            user_text=user_text,
            user_id=user_id,
            timeout_sec=stage1_timeout_sec,
        )
        verbal_ack = str(response_payload.get("verbal_ack", "")).strip()
        if not verbal_ack:
            if self._config.intent_detection_mode == "llm_with_rules_fallback":
                result = self._execute_rule_turn(
                    user_text=user_text,
                    history=history,
                    source="rules_llm_response_fallback",
                )
                publish_feedback(goal_handle, "complete", 1.0)
                return result
            result = self._execute_llm_failure_turn(user_text=user_text, history=history)
            publish_feedback(goal_handle, "complete", 1.0)
            return result

        publish_feedback(goal_handle, "extracting_intent", 0.7)
        if goal_handle.is_cancel_requested:
            return self._cancelled_result(history)

        intent_payload = self._query_intent(
            history_messages=history_messages,
            user_text=user_text,
            assistant_response=verbal_ack,
            user_id=user_id,
            timeout_sec=self._config.intent_request_timeout_sec,
        )

        (
            resolved_intent,
            intent_source,
            intent_confidence,
            user_intent,
        ) = self._resolve_intent(
            user_text=user_text,
            verbal_ack=verbal_ack,
            intent_payload=intent_payload,
        )

        updated_history = messages_to_history(
            history_messages
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ],
            max_history_messages=self._config.max_history_messages,
        )

        self._handled_requests += 1
        publish_feedback(goal_handle, "complete", 1.0)
        return TurnExecutionResult(
            success=True,
            verbal_ack=verbal_ack,
            updated_history=updated_history,
            intent=resolved_intent,
            intent_source=intent_source,
            intent_confidence=intent_confidence,
            user_intent=user_intent,
        )

    def _resolve_intent(
        self,
        user_text: str,
        verbal_ack: str,
        intent_payload: dict,
    ) -> tuple[str, str, float, dict]:
        if intent_payload:
            raw_user_intent = intent_payload.get("user_intent", intent_payload)
            user_intent = coerce_user_intent(raw_user_intent)
            hint_text = " ".join(
                [
                    user_intent.get("object", ""),
                    user_intent.get("goal", ""),
                    user_intent.get("input", ""),
                    verbal_ack,
                ]
            ).strip()
            resolved = normalize_intent(user_intent.get("type", ""), default="", hint_text=hint_text)
            confidence = coerce_float(
                intent_payload.get(
                    "intent_confidence",
                    intent_payload.get("confidence", 0.0),
                )
            )
            if resolved:
                return resolved, "llm_intent", confidence, user_intent

        if self._config.intent_detection_mode == "llm_with_rules_fallback":
            fallback_intent = detect_intent(user_text)
            fallback_user_intent = {"type": fallback_intent} if fallback_intent != "fallback" else {}
            fallback_confidence = 1.0 if fallback_intent != "fallback" else 0.0
            return (
                fallback_intent,
                "rules_llm_intent_fallback",
                fallback_confidence,
                fallback_user_intent,
            )

        return "fallback", "llm_intent_failed", 0.0, {}

    def _execute_rule_turn(
        self,
        user_text: str,
        history: list[str],
        source: str,
    ) -> TurnExecutionResult:
        intent = detect_intent(user_text)
        verbal_ack = build_rule_response(intent)
        user_intent = {"type": intent} if intent != "fallback" else {}
        updated_history = messages_to_history(
            history_to_messages(history, max_history_messages=self._config.max_history_messages)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ],
            max_history_messages=self._config.max_history_messages,
        )
        self._handled_requests += 1
        return TurnExecutionResult(
            success=True,
            verbal_ack=verbal_ack,
            updated_history=updated_history,
            intent=intent,
            intent_source=source,
            intent_confidence=1.0 if intent != "fallback" else 0.0,
            user_intent=user_intent,
        )

    def _execute_disabled_turn(self, user_text: str, history: list[str]) -> TurnExecutionResult:
        verbal_ack = self._config.fallback_response
        updated_history = messages_to_history(
            history_to_messages(history, max_history_messages=self._config.max_history_messages)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ],
            max_history_messages=self._config.max_history_messages,
        )
        self._handled_requests += 1
        return TurnExecutionResult(
            success=False,
            verbal_ack=verbal_ack,
            updated_history=updated_history,
            intent="fallback",
            intent_source="llm_disabled",
            intent_confidence=0.0,
            user_intent={},
        )

    def _execute_llm_failure_turn(self, user_text: str, history: list[str]) -> TurnExecutionResult:
        verbal_ack = self._config.fallback_response
        updated_history = messages_to_history(
            history_to_messages(history, max_history_messages=self._config.max_history_messages)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ],
            max_history_messages=self._config.max_history_messages,
        )
        self._handled_requests += 1
        return TurnExecutionResult(
            success=False,
            verbal_ack=verbal_ack,
            updated_history=updated_history,
            intent="fallback",
            intent_source="llm_response_failed",
            intent_confidence=0.0,
            user_intent={},
        )

    def _cancelled_result(self, history: list[str]) -> TurnExecutionResult:
        return TurnExecutionResult(
            success=False,
            verbal_ack="",
            updated_history=list(history),
            intent="fallback",
            intent_source="cancelled",
            intent_confidence=0.0,
            user_intent={},
        )

    def _query_response(
        self,
        history_messages: list[dict],
        user_text: str,
        user_id: str,
        timeout_sec: float,
    ) -> dict:
        prompt = build_response_prompt(
            robot_name=self._config.robot_name,
            user_id=user_id,
            system_prompt=self._config.system_prompt,
            environment_description=self._config.environment_description,
            response_prompt_addendum=self._config.response_prompt_addendum,
            skill_catalog_text=self._skill_catalog_text,
            persona_prompt=self._persona_prompt,
        )
        messages = [{"role": "system", "content": prompt}]
        messages.extend(history_messages)
        messages.append({"role": "user", "content": user_text})
        messages = trim_messages(messages, max_history_messages=self._config.max_history_messages)

        raw_response = self._transport.query(
            messages=messages,
            timeout_sec=timeout_sec,
            model=self._config.model,
            temperature=self._config.temperature,
            top_p=self._config.top_p,
            response_format=self._config.response_schema,
        )
        if not raw_response:
            return {}

        parsed = extract_json_object(raw_response)
        if parsed:
            verbal_ack = str(parsed.get("verbal_ack", "")).strip()
            if verbal_ack:
                return {"verbal_ack": verbal_ack}
        return {"verbal_ack": str(raw_response).strip()}

    def _query_intent(
        self,
        history_messages: list[dict],
        user_text: str,
        assistant_response: str,
        user_id: str,
        timeout_sec: float,
    ) -> dict:
        prompt = build_intent_prompt(
            robot_name=self._config.robot_name,
            user_id=user_id,
            system_prompt=self._config.system_prompt,
            environment_description=self._config.environment_description,
            intent_prompt_addendum=self._config.intent_prompt_addendum,
            skill_catalog_text=self._skill_catalog_text,
            persona_prompt=self._persona_prompt,
        )
        messages = [{"role": "system", "content": prompt}]
        messages.extend(history_messages)
        messages.extend(
            [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": assistant_response},
                {
                    "role": "user",
                    "content": json.dumps(
                        {
                            "task": "Extract user intent in canonical JSON form",
                            "user_text": user_text,
                            "assistant_response": assistant_response,
                        },
                        separators=(",", ":"),
                    ),
                },
            ]
        )
        messages = trim_messages(messages, max_history_messages=self._config.max_history_messages)

        raw_response = self._transport.query(
            messages=messages,
            timeout_sec=timeout_sec,
            model=self._config.intent_model,
            temperature=self._config.temperature,
            top_p=self._config.top_p,
            response_format=self._config.intent_schema,
        )
        if not raw_response:
            return {}

        parsed = extract_json_object(raw_response)
        if not parsed:
            self._logger.warn("Intent extraction response was not valid JSON")
            return {}
        return parsed

    def _inject_identity_reminder(self, history_messages: list[dict]) -> list[dict]:
        if self._config.identity_reminder_every_n_turns <= 0:
            return list(history_messages)
        if self._handled_requests <= 0:
            return list(history_messages)
        if self._handled_requests % self._config.identity_reminder_every_n_turns != 0:
            return list(history_messages)

        reminder = {
            "role": "system",
            "content": (
                f"Reminder: You are {self._config.robot_name}. "
                "Keep your personality and stay concise for spoken responses."
            ),
        }
        return list(history_messages) + [reminder]
