import json
from pathlib import Path
import socket
import time
import urllib.error
import urllib.request

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class OllamaResponder(Node):
    """Backend node that turns user text into LLM responses via Ollama."""

    def __init__(self) -> None:
        super().__init__("ollama_responder")

        self.declare_parameter(
            "enabled",
            False,
            ParameterDescriptor(
                description="Enable Ollama backend",
                dynamic_typing=True,
            ),
        )
        self.declare_parameter("backend_request_topic", "/chatbot/backend/request")
        self.declare_parameter("backend_response_topic", "/chatbot/backend/response")
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
        self.declare_parameter("dedupe_window_sec", 0.8)
        self.declare_parameter("disabled_warn_period_sec", 5.0)
        self.declare_parameter("robot_name", "Pop")
        self.declare_parameter("persona_prompt_path", "")
        self.declare_parameter(
            "system_prompt",
            (
                "You are {robot_name}, a social robot assistant in a human-robot interaction lab. "
                "Be concise, polite, and action-oriented. Keep responses short enough for text-to-speech."
            ),
        )
        self.declare_parameter(
            "prompt_addendum",
            "When the user asks for posture changes, answer clearly using verbs like stand, sit, or kneel.",
        )
        self.declare_parameter("identity_reminder_every_n_turns", 6)

        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.backend_request_topic = self.get_parameter("backend_request_topic").value
        self.backend_response_topic = self.get_parameter("backend_response_topic").value
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
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self.disabled_warn_period_sec = float(
            self.get_parameter("disabled_warn_period_sec").value
        )
        self.robot_name = self.get_parameter("robot_name").value
        self.persona_prompt_path = self.get_parameter("persona_prompt_path").value
        self.system_prompt_template = self.get_parameter("system_prompt").value
        self.prompt_addendum = self.get_parameter("prompt_addendum").value
        self.identity_reminder_every_n_turns = int(
            self.get_parameter("identity_reminder_every_n_turns").value
        )
        self._last_request_text = ""
        self._last_request_ts = 0.0
        self._last_disabled_warn_ts = 0.0
        self._handled_requests = 0
        self._resolved_system_prompt = self._build_system_prompt()

        self.response_publisher = self.create_publisher(
            String, self.backend_response_topic, 10
        )
        self.create_subscription(
            String, self.backend_request_topic, self._on_backend_request, 10
        )

        self.history = []
        if self._resolved_system_prompt:
            self.history.append({"role": "system", "content": self._resolved_system_prompt})

        self.get_logger().info(
            "ollama_responder ready | enabled:%s model:%s ctx:%s req:%s resp:%s"
            % (
                self.enabled,
                self.model,
                self.context_window_tokens,
                self.backend_request_topic,
                self.backend_response_topic,
            )
        )
        self._log_model_inventory()

    def _on_backend_request(self, msg: String) -> None:
        if not self.enabled:
            now = time.monotonic()
            if now - self._last_disabled_warn_ts >= self.disabled_warn_period_sec:
                self.get_logger().warn(
                    "Ollama is disabled; enable with launch arg ollama_enabled:=true"
                )
                self._last_disabled_warn_ts = now
            return

        user_text = msg.data.strip()
        if not user_text:
            return
        now = time.monotonic()
        if (
            user_text == self._last_request_text
            and now - self._last_request_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(
                f'Ignored duplicate backend request within {self.dedupe_window_sec}s'
            )
            return
        self._last_request_text = user_text
        self._last_request_ts = now

        # Periodically reinforce personality/instructions without rebuilding full history.
        if (
            self.identity_reminder_every_n_turns > 0
            and self._handled_requests > 0
            and self._handled_requests % self.identity_reminder_every_n_turns == 0
        ):
            self.history.append(
                {
                    "role": "system",
                    "content": (
                        f"Reminder: You are {self.robot_name}. Keep your personality and "
                        "stay concise for spoken responses."
                    ),
                }
            )

        self.history.append({"role": "user", "content": user_text})
        self._trim_history()
        timeout_sec = (
            self.first_request_timeout_sec
            if self._handled_requests == 0
            else self.request_timeout_sec
        )
        response_text = self._query_ollama(timeout_sec=timeout_sec)
        if not response_text:
            response_text = self.fallback_response
            self.get_logger().warn("Using fallback response because Ollama returned no text")

        self.history.append({"role": "assistant", "content": response_text})
        self._trim_history()
        self._handled_requests += 1
        out_msg = String()
        out_msg.data = response_text
        self.response_publisher.publish(out_msg)
        self.get_logger().info(
            f'Published Ollama response to "{self.backend_response_topic}"'
        )

    def _query_ollama(self, timeout_sec: float) -> str:
        payload = {
            "model": self.model,
            "messages": self.history,
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
            with urllib.request.urlopen(
                request, timeout=timeout_sec
            ) as response:
                parsed = json.loads(response.read().decode("utf-8"))
            message = parsed.get("message", {})
            text = message.get("content", "").strip()
            if not text:
                # Compatibility fallback in case endpoint returns generate-style payload.
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

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _trim_history(self) -> None:
        if self.max_history_messages <= 0:
            return
        if not self.history:
            return

        system_prefix = []
        start_index = 0
        if self.history[0].get("role") == "system":
            system_prefix = [self.history[0]]
            start_index = 1

        non_system_messages = self.history[start_index:]
        if len(non_system_messages) > self.max_history_messages:
            non_system_messages = non_system_messages[-self.max_history_messages :]
        self.history = system_prefix + non_system_messages

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

def main(args=None) -> None:
    rclpy.init(args=args)
    node = OllamaResponder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
