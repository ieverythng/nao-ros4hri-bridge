import re
import time

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
        self.declare_parameter("posture_command_topic", "/chatbot/posture_command")
        self.declare_parameter("backend_request_topic", "/chatbot/backend/request")
        self.declare_parameter("backend_response_topic", "/chatbot/backend/response")
        self.declare_parameter("backend_fallback_to_rules", False)
        self.declare_parameter("backend_response_timeout_sec", 30.0)
        self.declare_parameter("dedupe_window_sec", 0.8)

        self.mode = self.get_parameter("mode").value
        self.user_text_topic = self.get_parameter("user_text_topic").value
        self.assistant_text_topic = self.get_parameter("assistant_text_topic").value
        self.intent_topic = self.get_parameter("intent_topic").value
        self.posture_command_topic = self.get_parameter("posture_command_topic").value
        self.backend_request_topic = self.get_parameter("backend_request_topic").value
        self.backend_response_topic = self.get_parameter("backend_response_topic").value
        self.backend_fallback_to_rules = self._as_bool(
            self.get_parameter("backend_fallback_to_rules").value
        )
        self.backend_response_timeout_sec = float(
            self.get_parameter("backend_response_timeout_sec").value
        )
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self.pending_backend_fallback_intent = None
        self.pending_backend_timer = None
        self.waiting_backend_response = False
        self.ignore_late_backend_response = False
        self._last_user_text = ""
        self._last_user_text_ts = 0.0

        self.assistant_publisher = self.create_publisher(
            String, self.assistant_text_topic, 10
        )
        self.intent_publisher = self.create_publisher(String, self.intent_topic, 10)
        self.posture_command_publisher = self.create_publisher(
            String, self.posture_command_topic, 10
        )
        self.backend_request_publisher = self.create_publisher(
            String, self.backend_request_topic, 10
        )

        self.create_subscription(String, self.user_text_topic, self._on_user_text, 10)
        self.create_subscription(
            String, self.backend_response_topic, self._on_backend_response, 10
        )

        self.get_logger().info(
            "mission_controller ready | mode:%s user:%s assistant:%s intent:%s posture:%s backend_req:%s backend_resp:%s"
            % (
                self.mode,
                self.user_text_topic,
                self.assistant_text_topic,
                self.intent_topic,
                self.posture_command_topic,
                self.backend_request_topic,
                self.backend_response_topic,
            )
        )

    def _on_user_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        now = time.monotonic()
        if (
            text == self._last_user_text
            and now - self._last_user_text_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(f'Ignored duplicate user request within {self.dedupe_window_sec}s')
            return
        self._last_user_text = text
        self._last_user_text_ts = now

        intent = self._detect_intent(text)
        self._publish(self.intent_publisher, intent)
        self._handle_posture_intent(intent)

        if self.mode == "backend":
            self.ignore_late_backend_response = False
            self.waiting_backend_response = True
            self._publish(self.backend_request_publisher, text)
            self._schedule_backend_fallback(intent)
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
        if not self.waiting_backend_response and self.ignore_late_backend_response:
            self.get_logger().warn("Ignored late backend response after fallback")
            return
        text = msg.data.strip()
        if not text:
            return
        self._cancel_backend_fallback_timer()
        self.pending_backend_fallback_intent = None
        self.waiting_backend_response = False
        self._publish(self.assistant_publisher, text)
        self.get_logger().info(
            f'Forwarded backend response to "{self.assistant_text_topic}"'
        )
        # Optional: detect posture requests in backend text responses too.
        self._handle_posture_intent(self._detect_intent(text))

    def _schedule_backend_fallback(self, intent: str) -> None:
        self.pending_backend_fallback_intent = intent
        self._cancel_backend_fallback_timer()
        if not self.backend_fallback_to_rules:
            return
        self.pending_backend_timer = self.create_timer(
            self.backend_response_timeout_sec,
            self._on_backend_timeout,
        )

    def _cancel_backend_fallback_timer(self) -> None:
        if self.pending_backend_timer is not None:
            self.pending_backend_timer.cancel()
            self.pending_backend_timer = None

    def _on_backend_timeout(self) -> None:
        self._cancel_backend_fallback_timer()
        if self.mode != "backend":
            return
        if not self.backend_fallback_to_rules:
            return
        intent = self.pending_backend_fallback_intent
        if not intent:
            return
        fallback = self._build_rule_response(intent)
        self._publish(self.assistant_publisher, fallback)
        self.get_logger().warn(
            "Backend response timeout; published rule-based fallback response"
        )
        self.pending_backend_fallback_intent = None
        self.waiting_backend_response = False
        self.ignore_late_backend_response = True

    @staticmethod
    def _publish(publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    @staticmethod
    def _detect_intent(text: str) -> str:
        lowered = text.lower()
        if MissionController._contains_any_phrase(
            lowered,
            (
                "stand up",
                "get up",
                "please stand",
                "can you stand",
                "stand",
            ),
        ):
            return "posture_stand"
        if MissionController._contains_any_phrase(
            lowered,
            (
                "sit down",
                "take a seat",
                "please sit",
                "can you sit",
                "sit",
            ),
        ):
            return "posture_sit"
        if MissionController._contains_any_phrase(
            lowered,
            (
                "kneel down",
                "kneel",
                "crouch",
                "on your knees",
                "seiza",
            ),
        ):
            return "posture_kneel"
        if MissionController._contains_any_phrase(lowered, ("hello", "hi", "hey")):
            return "greet"
        if MissionController._contains_any_phrase(lowered, ("how are you",)):
            return "wellbeing"
        if MissionController._contains_any_phrase(
            lowered, ("your name", "who are you")
        ):
            return "identity"
        if MissionController._contains_any_phrase(lowered, ("help",)):
            return "help"
        return "fallback"

    @staticmethod
    def _contains_any_phrase(text: str, phrases: tuple[str, ...]) -> bool:
        for phrase in phrases:
            # Build regex that matches full words and supports flexible spacing.
            words = [re.escape(part) for part in phrase.split()]
            pattern = r"\b" + r"\s+".join(words) + r"\b"
            if re.search(pattern, text):
                return True
        return False

    @staticmethod
    def _build_rule_response(intent: str) -> str:
        if intent == "posture_stand":
            return "Sure. I am switching to a standing posture."
        if intent == "posture_sit":
            return "Sure. I am switching to a sitting posture."
        if intent == "posture_kneel":
            return "Sure. I am switching to a kneeling posture."
        if intent == "greet":
            return "Hello! Nice to meet you."
        if intent == "wellbeing":
            return "I am doing well. Thank you for asking."
        if intent == "identity":
            return "I am your Nao mission controller."
        if intent == "help":
            return "You can greet me, ask my name, ask how I am, or ask for posture changes like stand, kneel, or sit."
        return "I heard you. We are testing the chat to speech pipeline."

    def _handle_posture_intent(self, intent: str) -> None:
        posture_map = {
            "posture_stand": "stand",
            "posture_sit": "sit",
            "posture_kneel": "kneel",
        }
        command = posture_map.get(intent)
        if not command:
            return
        self._publish(self.posture_command_publisher, command)
        self.get_logger().info(
            f'Published posture command "{command}" to "{self.posture_command_topic}"'
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
