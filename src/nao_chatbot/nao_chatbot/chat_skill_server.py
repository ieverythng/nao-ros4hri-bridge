#!/usr/bin/env python3
"""Action server for `/skill/chat` backed by Ollama."""

import json
from pathlib import Path
import socket
from string import Template
import threading
import urllib.error
import urllib.request

from chatbot_msgs.msg import DialogueRole
from communication_skills.action import Chat
from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import normalize_intent
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_skills.msg import Result as SkillResult


INTENT_DETECTION_MODES = {"rules", "llm", "llm_with_rules_fallback"}
CHATBOT_RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "verbal_ack": {"type": "string"},
        "user_intent": {
            "type": "object",
            "properties": {
                "type": {"type": "string"},
                "object": {"type": "string"},
                "recipient": {"type": "string"},
                "input": {"type": "string"},
                "goal": {"type": "string"},
            },
            "required": ["type"],
        },
        "confidence": {"type": "number"},
        "intent_confidence": {"type": "number"},
    },
}

CHATBOT_PROMPT = Template(
    """
You are a friendly robot called $robot_name. You try to help the user to the best of your abilities.
You are always helpful, and ask further questions if the desires of the user are unclear.
Your answers are always polite yet concise and to-the-point.

Your aim is to extract the user goal while replying naturally.

Your response must be a JSON object with the following fields (both optional):
- verbal_ack: a short acknowledgement sentence or a concise answer.
- user_intent: the user overall goal with optional fields:
  - type: one of these canonical intents:
    - posture_stand
    - posture_sit
    - posture_kneel
    - greet
    - identity
    - wellbeing
    - help
    - fallback
  - object
  - recipient
  - input
  - goal

Rules:
- Return only valid JSON (no markdown or extra text).
- For posture requests, set user_intent.type to posture_stand/posture_sit/posture_kneel.
- For greetings, set user_intent.type to greet.
- For "who are you"/name, set user_intent.type to identity.
- For "how are you", set user_intent.type to wellbeing.
- For help requests, set user_intent.type to help.
- If no action is required, you can omit user_intent and answer in verbal_ack.
- If uncertain, keep verbal_ack short and set user_intent.type to fallback.

The user_id of the person you are talking to is $user_id.
Always use this ID when referring to the person in your response.

This is a description of the environment:
$environment
"""
)


class ChatSkillServer(Node):
    """Serve Ollama-backed conversational turns over `/skill/chat`."""

    def __init__(self) -> None:
        super().__init__("ollama_chatbot")

        self.declare_parameter("action_name", "/skill/chat")
        self.declare_parameter("role_name", DialogueRole.DEFAULT_ROLE)
        self.declare_parameter(
            "enabled",
            True,
            ParameterDescriptor(
                description="Enable chat skill backend",
                dynamic_typing=True,
            ),
        )
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model", "llama3.2:1b")
        self.declare_parameter("intent_model", "")
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("first_request_timeout_sec", 60.0)
        self.declare_parameter("intent_request_timeout_sec", 10.0)
        self.declare_parameter("context_window_tokens", 4096)
        self.declare_parameter("temperature", 0.2)
        self.declare_parameter("top_p", 0.9)
        self.declare_parameter(
            "fallback_response",
            "I am having trouble reaching my language model right now.",
        )
        self.declare_parameter("max_history_messages", 12)
        self.declare_parameter("robot_name", "Pop")
        self.declare_parameter("persona_prompt_path", "")
        self.declare_parameter("system_prompt", "")
        self.declare_parameter(
            "prompt_addendum",
            (
                "Use concise speech suitable for text-to-speech. "
                "Posture requests should map to posture_stand/posture_sit/posture_kneel."
            ),
        )
        self.declare_parameter("intent_prompt_addendum", "")
        self.declare_parameter("environment_description", "No specific objects described.")
        self.declare_parameter("identity_reminder_every_n_turns", 6)
        self.declare_parameter("intent_detection_mode", "llm_with_rules_fallback")

        self.action_name = self.get_parameter("action_name").value
        self.role_name = self.get_parameter("role_name").value
        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.ollama_url = self.get_parameter("ollama_url").value
        self.model = self.get_parameter("model").value
        self.intent_model = str(self.get_parameter("intent_model").value).strip()
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.first_request_timeout_sec = float(
            self.get_parameter("first_request_timeout_sec").value
        )
        self.intent_request_timeout_sec = max(
            0.5,
            float(self.get_parameter("intent_request_timeout_sec").value),
        )
        self.context_window_tokens = int(self.get_parameter("context_window_tokens").value)
        self.temperature = float(self.get_parameter("temperature").value)
        self.top_p = float(self.get_parameter("top_p").value)
        self.fallback_response = self.get_parameter("fallback_response").value
        self.max_history_messages = int(self.get_parameter("max_history_messages").value)
        self.robot_name = self.get_parameter("robot_name").value
        self.persona_prompt_path = self.get_parameter("persona_prompt_path").value
        self.system_prompt_template = self.get_parameter("system_prompt").value
        self.prompt_addendum = self.get_parameter("prompt_addendum").value
        self.intent_prompt_addendum = self.get_parameter("intent_prompt_addendum").value
        self.environment_description = self.get_parameter("environment_description").value
        self.identity_reminder_every_n_turns = int(
            self.get_parameter("identity_reminder_every_n_turns").value
        )
        self.intent_detection_mode = (
            str(self.get_parameter("intent_detection_mode").value).strip().lower()
        )
        if self.intent_detection_mode not in INTENT_DETECTION_MODES:
            self.get_logger().warn(
                "Unsupported intent_detection_mode=%s, defaulting to llm_with_rules_fallback"
                % self.intent_detection_mode
            )
            self.intent_detection_mode = "llm_with_rules_fallback"

        if not self.intent_model:
            self.intent_model = self.model

        self._handled_requests = 0
        self._execution_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Chat,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "ollama_chatbot ready | action:%s canonical_type:communication_skills/action/Chat "
            "backend_enabled:%s model:%s mode:%s"
            % (
                self.action_name,
                self.enabled,
                self.intent_model,
                self.intent_detection_mode,
            )
        )
        self._log_model_inventory()

    def goal_callback(self, goal_request: Chat.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            self.get_logger().warn("Rejected chat goal because another goal is running")
            return GoalResponse.REJECT
        user_message, _history, _user_id = self._extract_canonical_goal(goal_request)
        if not user_message:
            self.get_logger().warn("Rejected chat goal with empty user message")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for chat goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._canonical_result(
                False,
                "Another chat goal is already executing",
                [],
                "",
                SkillResult.ROS_ECANCELED,
                intent="fallback",
                intent_source="server_busy",
                intent_confidence=0.0,
                user_intent={},
            )

        try:
            user_text, history, user_id = self._extract_canonical_goal(goal_handle.request)
            if not user_text:
                goal_handle.abort()
                return self._canonical_result(
                    False,
                    "Goal user message is empty",
                    history,
                    "",
                    SkillResult.ROS_EBADMSG,
                    intent="fallback",
                    intent_source="bad_goal",
                    intent_confidence=0.0,
                    user_intent={},
                )

            (
                success,
                verbal_ack,
                updated_history,
                detected_intent,
                intent_source,
                intent_confidence,
                user_intent,
            ) = self._execute_turn(
                goal_handle=goal_handle,
                user_text=user_text,
                history=history,
                user_id=user_id,
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._canonical_result(
                    False,
                    "Cancelled while executing chat goal",
                    history,
                    "",
                    SkillResult.ROS_ECANCELED,
                    intent="fallback",
                    intent_source="cancelled",
                    intent_confidence=0.0,
                    user_intent={},
                )

            goal_handle.succeed()
            error_code = SkillResult.ROS_ENOERR if success else SkillResult.ROS_EOTHER
            message = "" if success else "No assistant response generated"
            return self._canonical_result(
                success,
                message,
                updated_history,
                verbal_ack,
                error_code,
                intent=detected_intent,
                intent_source=intent_source,
                intent_confidence=intent_confidence,
                user_intent=user_intent,
            )
        finally:
            self._execution_lock.release()

    def _execute_turn(
        self,
        goal_handle,
        user_text: str,
        history: list[str],
        user_id: str,
    ) -> tuple[bool, str, list[str], str, str, float, dict]:
        self._publish_feedback(goal_handle, "thinking", 0.1)
        if goal_handle.is_cancel_requested:
            return False, "", list(history), "fallback", "cancelled", 0.0, {}

        if self.intent_detection_mode == "rules":
            result = self._execute_rule_turn(
                user_text=user_text,
                history=history,
                source="rules",
            )
            self._publish_feedback(goal_handle, "complete", 1.0)
            return result

        if not self.enabled:
            if self.intent_detection_mode == "llm_with_rules_fallback":
                result = self._execute_rule_turn(
                    user_text=user_text,
                    history=history,
                    source="rules_llm_disabled",
                )
                self._publish_feedback(goal_handle, "complete", 1.0)
                return result
            result = self._execute_disabled_turn(
                user_text=user_text,
                history=history,
            )
            self._publish_feedback(goal_handle, "complete", 1.0)
            return result

        history_messages = self._history_to_messages(history)
        if (
            self.identity_reminder_every_n_turns > 0
            and self._handled_requests > 0
            and self._handled_requests % self.identity_reminder_every_n_turns == 0
        ):
            history_messages.append(
                {
                    "role": "system",
                    "content": (
                        f"Reminder: You are {self.robot_name}. Keep your personality "
                        "and stay concise for spoken responses."
                    ),
                }
            )

        self._publish_feedback(goal_handle, "generating", 0.4)
        if goal_handle.is_cancel_requested:
            return False, "", list(history), "fallback", "cancelled", 0.0, {}

        timeout_sec = (
            self.first_request_timeout_sec
            if self._handled_requests == 0
            else self.request_timeout_sec
        )
        timeout_sec = min(timeout_sec, self.intent_request_timeout_sec)
        parsed_response = self._query_structured_chatbot(
            history_messages=history_messages,
            user_text=user_text,
            user_id=user_id,
            timeout_sec=timeout_sec,
        )
        if not parsed_response:
            if self.intent_detection_mode == "llm_with_rules_fallback":
                result = self._execute_rule_turn(
                    user_text=user_text,
                    history=history,
                    source="rules_llm_parse_fallback",
                )
                self._publish_feedback(goal_handle, "complete", 1.0)
                return result
            result = self._execute_llm_failure_turn(
                user_text=user_text,
                history=history,
            )
            self._publish_feedback(goal_handle, "complete", 1.0)
            return result

        verbal_ack = str(parsed_response.get("verbal_ack", "")).strip()
        user_intent = self._coerce_user_intent(parsed_response.get("user_intent"))
        hinted_text = " ".join(
            [
                user_intent.get("object", ""),
                user_intent.get("goal", ""),
                user_intent.get("input", ""),
                verbal_ack,
            ]
        ).strip()
        resolved_intent = normalize_intent(
            user_intent.get("type", ""),
            default="",
            hint_text=hinted_text,
        )
        source = "llm"
        confidence = self._coerce_float(
            parsed_response.get(
                "intent_confidence",
                parsed_response.get("confidence", 0.0),
            )
        )

        if not resolved_intent:
            if self.intent_detection_mode == "llm_with_rules_fallback":
                resolved_intent = detect_intent(user_text)
                source = "rules_llm_no_intent_fallback"
                confidence = 0.0
            else:
                resolved_intent = "fallback"
                source = "llm_no_intent"

        if not verbal_ack:
            if resolved_intent != "fallback":
                verbal_ack = build_rule_response(resolved_intent)
            elif self.intent_detection_mode == "llm_with_rules_fallback":
                verbal_ack = build_rule_response("fallback")
            else:
                verbal_ack = self.fallback_response

        updated_history = self._messages_to_history(
            history_messages
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ]
        )
        self._handled_requests += 1
        self._publish_feedback(goal_handle, "complete", 1.0)
        return (
            True,
            verbal_ack,
            updated_history,
            resolved_intent,
            source,
            confidence,
            user_intent,
        )

    def _execute_rule_turn(
        self,
        user_text: str,
        history: list[str],
        source: str,
    ) -> tuple[bool, str, list[str], str, str, float, dict]:
        intent = detect_intent(user_text)
        verbal_ack = build_rule_response(intent)
        user_intent = {"type": intent} if intent != "fallback" else {}
        updated_history = self._messages_to_history(
            self._history_to_messages(history)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ]
        )
        self._handled_requests += 1
        return (
            True,
            verbal_ack,
            updated_history,
            intent,
            source,
            1.0 if intent != "fallback" else 0.0,
            user_intent,
        )

    def _execute_disabled_turn(
        self,
        user_text: str,
        history: list[str],
    ) -> tuple[bool, str, list[str], str, str, float, dict]:
        verbal_ack = self.fallback_response
        updated_history = self._messages_to_history(
            self._history_to_messages(history)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ]
        )
        self._handled_requests += 1
        return (
            False,
            verbal_ack,
            updated_history,
            "fallback",
            "llm_disabled",
            0.0,
            {},
        )

    def _execute_llm_failure_turn(
        self,
        user_text: str,
        history: list[str],
    ) -> tuple[bool, str, list[str], str, str, float, dict]:
        verbal_ack = self.fallback_response
        updated_history = self._messages_to_history(
            self._history_to_messages(history)
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": verbal_ack},
            ]
        )
        self._handled_requests += 1
        return (
            False,
            verbal_ack,
            updated_history,
            "fallback",
            "llm_failed",
            0.0,
            {},
        )

    def _query_structured_chatbot(
        self,
        history_messages: list[dict],
        user_text: str,
        user_id: str,
        timeout_sec: float,
    ) -> dict:
        messages = [
            {"role": "system", "content": self._build_chatbot_prompt(user_id=user_id)},
        ]
        messages.extend(history_messages)
        messages.append({"role": "user", "content": user_text})
        messages = self._trim_messages(messages)

        raw_response = self._query_ollama(
            messages=messages,
            timeout_sec=timeout_sec,
            model=self.intent_model,
            temperature=self.temperature,
            top_p=self.top_p,
            response_format=CHATBOT_RESPONSE_SCHEMA,
        )
        if not raw_response:
            return {}
        parsed = self._extract_json_object(raw_response)
        if not parsed:
            self.get_logger().warn("Structured chatbot response was not valid JSON")
            return {}
        return parsed

    def _build_chatbot_prompt(self, user_id: str) -> str:
        parts = []

        if self.persona_prompt_path:
            prompt_path = Path(self.persona_prompt_path)
            if prompt_path.exists():
                try:
                    file_prompt = prompt_path.read_text(encoding="utf-8").strip()
                    if file_prompt:
                        parts.append(file_prompt)
                except Exception as err:
                    self.get_logger().warn(f"Could not read persona_prompt_path: {err}")
            else:
                self.get_logger().warn(
                    f'persona_prompt_path does not exist: "{self.persona_prompt_path}"'
                )

        if self.system_prompt_template.strip():
            try:
                system_prompt = self.system_prompt_template.format(
                    robot_name=self.robot_name
                ).strip()
            except Exception:
                system_prompt = self.system_prompt_template.strip()
            if system_prompt:
                parts.append(system_prompt)

        template_prompt = CHATBOT_PROMPT.safe_substitute(
            robot_name=self.robot_name,
            user_id=user_id or "user1",
            environment=self.environment_description or "No specific objects described.",
        ).strip()
        parts.append(template_prompt)

        if self.prompt_addendum.strip():
            parts.append(self.prompt_addendum.strip())
        if self.intent_prompt_addendum.strip():
            parts.append(self.intent_prompt_addendum.strip())
        return "\n\n".join(parts).strip()

    def _extract_canonical_goal(self, goal: Chat.Goal) -> tuple[str, list[str], str]:
        payload = self._parse_json_dict(goal.role.configuration)
        user_message = str(payload.get("user_message", "")).strip()
        if not user_message and goal.initial_input.strip():
            user_message = goal.initial_input.strip()
        if not user_message:
            user_message = str(payload.get("input", "")).strip()

        history_raw = payload.get("conversation_history", payload.get("history", []))
        history = self._coerce_history(history_raw)

        user_id = str(payload.get("user_id", "")).strip()
        if not user_id:
            user_id = str(goal.person_id).strip()
        if not user_id:
            user_id = "user1"
        return user_message, history, user_id

    @staticmethod
    def _coerce_history(raw_history) -> list[str]:
        if isinstance(raw_history, list):
            return [str(entry).strip() for entry in raw_history if str(entry).strip()]
        if isinstance(raw_history, str) and raw_history.strip():
            return [raw_history.strip()]
        return []

    @staticmethod
    def _parse_json_dict(payload: str) -> dict:
        if not payload:
            return {}
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            return {}
        if not isinstance(parsed, dict):
            return {}
        return parsed

    @staticmethod
    def _extract_json_object(payload: str) -> dict:
        parsed = ChatSkillServer._parse_json_dict(payload)
        if parsed:
            return parsed

        decoder = json.JSONDecoder()
        for start in range(len(payload)):
            if payload[start] != "{":
                continue
            try:
                maybe_obj, _ = decoder.raw_decode(payload[start:])
            except json.JSONDecodeError:
                continue
            if isinstance(maybe_obj, dict):
                return maybe_obj
        return {}

    @staticmethod
    def _coerce_user_intent(user_intent) -> dict:
        if isinstance(user_intent, dict):
            cleaned = {}
            for key in ("type", "object", "recipient", "input", "goal"):
                value = str(user_intent.get(key, "")).strip()
                if value:
                    cleaned[key] = value
            return cleaned
        if isinstance(user_intent, str) and user_intent.strip():
            return {"type": user_intent.strip()}
        return {}

    def _query_ollama(
        self,
        messages: list[dict],
        timeout_sec: float,
        model: str,
        temperature: float | None = None,
        top_p: float | None = None,
        response_format: dict | None = None,
    ) -> str:
        payload = {
            "model": model,
            "messages": messages,
            "stream": False,
            "options": {
                "num_ctx": self.context_window_tokens,
                "temperature": (
                    self.temperature if temperature is None else float(temperature)
                ),
                "top_p": self.top_p if top_p is None else float(top_p),
            },
        }
        if response_format is not None:
            payload["format"] = response_format

        request = urllib.request.Request(
            self.ollama_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=timeout_sec) as response:
                parsed = json.loads(response.read().decode("utf-8"))
            message = parsed.get("message", {})
            text = message.get("content", "").strip()
            if not text:
                text = parsed.get("response", "").strip()
            if not text:
                self.get_logger().warn("Ollama response did not include message content")
            return text
        except urllib.error.HTTPError as err:
            error_body = err.read().decode("utf-8", errors="replace")
            self.get_logger().error(
                f"Ollama HTTP error {err.code} for model {model} at "
                f"{self.ollama_url}: {error_body}"
            )
            if err.code == 404:
                self._log_model_inventory()
            return ""
        except urllib.error.URLError as err:
            self.get_logger().error(f"Ollama request failed: {err}")
            return ""
        except TimeoutError:
            self.get_logger().error("Ollama request timed out")
            return ""
        except socket.timeout:
            self.get_logger().error("Ollama socket timeout")
            return ""
        except json.JSONDecodeError as err:
            self.get_logger().error(f"Ollama response decode failed: {err}")
            return ""
        except Exception as err:
            self.get_logger().error(f"Ollama unexpected error: {err}")
            return ""

    def _log_model_inventory(self) -> None:
        try:
            tags_url = self.ollama_url.replace("/api/chat", "/api/tags")
            with urllib.request.urlopen(tags_url, timeout=5.0) as response:
                payload = json.loads(response.read().decode("utf-8"))
            models = [m.get("name", "") for m in payload.get("models", [])]
            if models:
                self.get_logger().info("Ollama available models: %s" % ", ".join(models))
            else:
                self.get_logger().warn("Ollama tags endpoint returned no models")
        except Exception as err:
            self.get_logger().warn(f"Could not query Ollama model inventory: {err}")

    def _history_to_messages(self, history_entries) -> list[dict]:
        messages = []
        for index, entry in enumerate(history_entries):
            clean = str(entry).strip()
            if not clean:
                continue
            role = ""
            content = ""
            if ":" in clean:
                role_part, content_part = clean.split(":", 1)
                maybe_role = role_part.strip().lower()
                if maybe_role in {"system", "user", "assistant"}:
                    role = maybe_role
                    content = content_part.strip()
            if not role:
                role = "user" if index % 2 == 0 else "assistant"
                content = clean
            if content:
                messages.append({"role": role, "content": content})
        return self._trim_messages(messages)

    def _messages_to_history(self, messages: list[dict]) -> list[str]:
        serialized = []
        for message in self._trim_messages(messages):
            role = message.get("role", "").strip().lower()
            content = message.get("content", "").strip()
            if role not in {"user", "assistant"}:
                continue
            if not content:
                continue
            serialized.append(f"{role}:{content}")
        return serialized

    def _trim_messages(self, messages: list[dict]) -> list[dict]:
        if self.max_history_messages <= 0:
            return messages
        if not messages:
            return messages

        system_prefix = []
        non_system_messages = []
        for message in messages:
            if (
                not system_prefix
                and message.get("role", "") == "system"
                and message.get("content", "").strip()
            ):
                system_prefix = [message]
                continue
            non_system_messages.append(message)

        if len(non_system_messages) > self.max_history_messages:
            non_system_messages = non_system_messages[-self.max_history_messages :]
        return system_prefix + non_system_messages

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = Chat.Feedback()
        feedback.feedback.data_str = status
        feedback.feedback.data_float = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _canonical_result(
        success: bool,
        message: str,
        updated_history: list[str],
        assistant_response: str,
        error_code: int,
        intent: str,
        intent_source: str,
        intent_confidence: float,
        user_intent: dict,
    ):
        result = Chat.Result()
        result.result.error_code = int(error_code)
        result.result.error_msg = message
        result.role_results = json.dumps(
            {
                "success": bool(success),
                "assistant_response": assistant_response,
                "verbal_ack": assistant_response,
                "updated_history": list(updated_history),
                "message": message,
                "intent": intent,
                "intent_source": intent_source,
                "intent_confidence": float(intent_confidence),
                "user_intent": dict(user_intent),
            },
            separators=(",", ":"),
        )
        return result

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    @staticmethod
    def _coerce_float(value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChatSkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
