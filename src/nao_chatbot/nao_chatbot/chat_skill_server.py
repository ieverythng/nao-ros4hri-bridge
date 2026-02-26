#!/usr/bin/env python3
"""Action server for `/skill/chat` backed by Ollama."""

import json
from pathlib import Path
import socket
import threading
import urllib.error
import urllib.request

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nao_skills.action import Chat


class ChatSkillServer(Node):
    """Serve conversational turns over `/skill/chat`."""

    def __init__(self) -> None:
        super().__init__("chat_skill_server")

        self.declare_parameter("action_name", "/skill/chat")
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
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("first_request_timeout_sec", 60.0)
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
        self.declare_parameter(
            "system_prompt",
            (
                "You are {robot_name}, a social robot assistant in a human-robot "
                "interaction lab. Be concise, polite, and action-oriented. Keep "
                "responses short enough for text-to-speech."
            ),
        )
        self.declare_parameter(
            "prompt_addendum",
            "When the user asks for posture changes, answer clearly using verbs like stand, sit, or kneel.",
        )
        self.declare_parameter("identity_reminder_every_n_turns", 6)

        self.action_name = self.get_parameter("action_name").value
        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.ollama_url = self.get_parameter("ollama_url").value
        self.model = self.get_parameter("model").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.first_request_timeout_sec = float(
            self.get_parameter("first_request_timeout_sec").value
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
        self.identity_reminder_every_n_turns = int(
            self.get_parameter("identity_reminder_every_n_turns").value
        )

        self._handled_requests = 0
        self._resolved_system_prompt = self._build_system_prompt()
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
            "chat_skill_server ready | action:%s enabled:%s model:%s"
            % (self.action_name, self.enabled, self.model)
        )
        self._log_model_inventory()

    def goal_callback(self, goal_request: Chat.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            self.get_logger().warn("Rejected chat goal because another goal is running")
            return GoalResponse.REJECT
        if not goal_request.user_message.strip():
            self.get_logger().warn("Rejected chat goal with empty user_message")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for chat goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._result(False, "", [])

        try:
            return self._execute_locked(goal_handle)
        finally:
            self._execution_lock.release()

    def _execute_locked(self, goal_handle):
        goal = goal_handle.request
        user_text = goal.user_message.strip()

        self._publish_feedback(goal_handle, "thinking", 0.1)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "", list(goal.conversation_history))

        history_messages = self._history_to_messages(goal.conversation_history)
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

        messages = []
        if self._resolved_system_prompt:
            messages.append({"role": "system", "content": self._resolved_system_prompt})
        messages.extend(history_messages)
        messages.append({"role": "user", "content": user_text})
        messages = self._trim_messages(messages)

        self._publish_feedback(goal_handle, "generating", 0.4)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "", list(goal.conversation_history))

        timeout_sec = (
            self.first_request_timeout_sec
            if self._handled_requests == 0
            else self.request_timeout_sec
        )
        if self.enabled:
            assistant_text = self._query_ollama(
                messages=messages,
                timeout_sec=timeout_sec,
            )
        else:
            assistant_text = ""
            self.get_logger().warn("Chat skill is disabled; returning fallback response")

        success = bool(assistant_text)
        if not assistant_text:
            assistant_text = self.fallback_response

        updated_history = self._messages_to_history(
            history_messages
            + [
                {"role": "user", "content": user_text},
                {"role": "assistant", "content": assistant_text},
            ]
        )

        self._handled_requests += 1
        self._publish_feedback(goal_handle, "complete", 1.0)
        goal_handle.succeed()
        return self._result(success, assistant_text, updated_history)

    def _query_ollama(self, messages: list[dict], timeout_sec: float) -> str:
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "options": {
                "num_ctx": self.context_window_tokens,
                "temperature": self.temperature,
                "top_p": self.top_p,
            },
        }

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
                f"Ollama HTTP error {err.code} at {self.ollama_url}: {error_body}"
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

    def _build_system_prompt(self) -> str:
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

        try:
            template_prompt = self.system_prompt_template.format(
                robot_name=self.robot_name
            ).strip()
        except Exception:
            template_prompt = self.system_prompt_template.strip()
        if template_prompt:
            parts.append(template_prompt)
        if self.prompt_addendum.strip():
            parts.append(self.prompt_addendum.strip())

        return "\n\n".join(parts).strip()

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = Chat.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(success: bool, assistant_response: str, updated_history: list[str]):
        result = Chat.Result()
        result.success = bool(success)
        result.assistant_response = assistant_response
        result.updated_history = list(updated_history)
        return result

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)


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
