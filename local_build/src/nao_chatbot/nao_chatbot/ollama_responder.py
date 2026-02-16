import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OllamaResponder(Node):
    """Backend node that turns user text into LLM responses via Ollama."""

    def __init__(self) -> None:
        super().__init__("ollama_responder")

        self.declare_parameter("enabled", False)
        self.declare_parameter("backend_request_topic", "/chatbot/backend/request")
        self.declare_parameter("backend_response_topic", "/chatbot/backend/response")
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model", "llama3.1")
        self.declare_parameter("request_timeout_sec", 12.0)
        self.declare_parameter(
            "system_prompt",
            "You are a concise, polite robot assistant helping users through voice and chat.",
        )

        self.enabled = self.get_parameter("enabled").value
        self.backend_request_topic = self.get_parameter("backend_request_topic").value
        self.backend_response_topic = self.get_parameter("backend_response_topic").value
        self.ollama_url = self.get_parameter("ollama_url").value
        self.model = self.get_parameter("model").value
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.system_prompt = self.get_parameter("system_prompt").value

        self.response_publisher = self.create_publisher(
            String, self.backend_response_topic, 10
        )
        self.create_subscription(
            String, self.backend_request_topic, self._on_backend_request, 10
        )

        self.history = []
        if self.system_prompt:
            self.history.append({"role": "system", "content": self.system_prompt})

        self.get_logger().info(
            "ollama_responder ready | enabled:%s model:%s req:%s resp:%s"
            % (
                self.enabled,
                self.model,
                self.backend_request_topic,
                self.backend_response_topic,
            )
        )

    def _on_backend_request(self, msg: String) -> None:
        if not self.enabled:
            self.get_logger().info("Ollama disabled; ignoring backend request")
            return

        user_text = msg.data.strip()
        if not user_text:
            return

        self.history.append({"role": "user", "content": user_text})
        response_text = self._query_ollama()
        if not response_text:
            return

        self.history.append({"role": "assistant", "content": response_text})
        out_msg = String()
        out_msg.data = response_text
        self.response_publisher.publish(out_msg)
        self.get_logger().info(
            f'Published Ollama response to "{self.backend_response_topic}"'
        )

    def _query_ollama(self) -> str:
        payload = {
            "model": self.model,
            "messages": self.history,
            "stream": False,
        }

        request = urllib.request.Request(
            self.ollama_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(
                request, timeout=self.request_timeout_sec
            ) as response:
                parsed = json.loads(response.read().decode("utf-8"))
            message = parsed.get("message", {})
            text = message.get("content", "").strip()
            if not text:
                self.get_logger().warn("Ollama response did not include message content")
            return text
        except urllib.error.URLError as err:
            self.get_logger().error(f"Ollama request failed: {err}")
            return ""
        except json.JSONDecodeError as err:
            self.get_logger().error(f"Ollama response decode failed: {err}")
            return ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OllamaResponder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
