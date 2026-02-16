import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionController(Node):
    """Intent extraction and response routing for chatbot flows."""

    def __init__(self) -> None:
        super().__init__("mission_controller")

        self.declare_parameter("mode", "rules")
        self.declare_parameter("user_text_topic", "/chatbot/user_text")
        self.declare_parameter("assistant_text_topic", "/chatbot/assistant_text")
        self.declare_parameter("intent_topic", "/chatbot/intent")
        self.declare_parameter("backend_request_topic", "/chatbot/backend/request")
        self.declare_parameter("backend_response_topic", "/chatbot/backend/response")

        self.mode = self.get_parameter("mode").value
        self.user_text_topic = self.get_parameter("user_text_topic").value
        self.assistant_text_topic = self.get_parameter("assistant_text_topic").value
        self.intent_topic = self.get_parameter("intent_topic").value
        self.backend_request_topic = self.get_parameter("backend_request_topic").value
        self.backend_response_topic = self.get_parameter("backend_response_topic").value

        self.assistant_publisher = self.create_publisher(
            String, self.assistant_text_topic, 10
        )
        self.intent_publisher = self.create_publisher(String, self.intent_topic, 10)
        self.backend_request_publisher = self.create_publisher(
            String, self.backend_request_topic, 10
        )

        self.create_subscription(String, self.user_text_topic, self._on_user_text, 10)
        self.create_subscription(
            String, self.backend_response_topic, self._on_backend_response, 10
        )

        self.get_logger().info(
            "mission_controller ready | mode:%s user:%s assistant:%s intent:%s backend_req:%s backend_resp:%s"
            % (
                self.mode,
                self.user_text_topic,
                self.assistant_text_topic,
                self.intent_topic,
                self.backend_request_topic,
                self.backend_response_topic,
            )
        )

    def _on_user_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        intent = self._detect_intent(text)
        self._publish(self.intent_publisher, intent)

        if self.mode == "backend":
            self._publish(self.backend_request_publisher, text)
            self.get_logger().info(
                f'Forwarded request to backend "{self.backend_request_topic}" | intent={intent}'
            )
            return

        response = self._build_rule_response(intent)
        self._publish(self.assistant_publisher, response)
        self.get_logger().info(
            f'Published rule response to "{self.assistant_text_topic}" | intent={intent}'
        )

    def _on_backend_response(self, msg: String) -> None:
        if self.mode != "backend":
            return
        text = msg.data.strip()
        if not text:
            return
        self._publish(self.assistant_publisher, text)
        self.get_logger().info(
            f'Forwarded backend response to "{self.assistant_text_topic}"'
        )

    @staticmethod
    def _publish(publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)

    @staticmethod
    def _detect_intent(text: str) -> str:
        lowered = text.lower()
        if any(token in lowered for token in ("hello", "hi", "hey")):
            return "greet"
        if "how are you" in lowered:
            return "wellbeing"
        if "your name" in lowered or "who are you" in lowered:
            return "identity"
        if "help" in lowered:
            return "help"
        return "fallback"

    @staticmethod
    def _build_rule_response(intent: str) -> str:
        if intent == "greet":
            return "Hello! Nice to meet you."
        if intent == "wellbeing":
            return "I am doing well. Thank you for asking."
        if intent == "identity":
            return "I am your Nao mission controller."
        if intent == "help":
            return "You can greet me, ask my name, or ask how I am."
        return "I heard you. We are testing the chat to speech pipeline."


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
