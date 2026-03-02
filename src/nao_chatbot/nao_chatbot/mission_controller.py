import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nao_chatbot.chat_skill_client import ChatSkillClient
from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import posture_command_for_intent
from nao_chatbot.posture_skill_client import PostureSkillClient


class MissionController(Node):
    """Intent extraction and response routing for chatbot flows."""

    def __init__(self) -> None:
        super().__init__("mission_controller")

        self.declare_parameter("mode", "rules")
        self.declare_parameter("user_text_topic", "/chatbot/user_text")
        self.declare_parameter("assistant_text_topic", "/chatbot/assistant_text")
        self.declare_parameter("intent_topic", "/chatbot/intent")
        self.declare_parameter("posture_command_topic", "/chatbot/posture_command")
        self.declare_parameter("backend_fallback_to_rules", False)
        self.declare_parameter("backend_response_timeout_sec", 30.0)
        self.declare_parameter("backend_execute_posture_after_response", True)
        self.declare_parameter("backend_posture_from_response_enabled", False)
        self.declare_parameter("dedupe_window_sec", 0.8)
        self.declare_parameter("use_posture_skill", True)
        self.declare_parameter("posture_skill_action", "/skill/do_posture")
        self.declare_parameter("posture_skill_speed", 0.8)
        self.declare_parameter("posture_skill_dispatch_wait_sec", 1.0)
        self.declare_parameter("use_chat_skill", True)
        self.declare_parameter("chat_skill_action", "/skill/chat")
        self.declare_parameter("chat_skill_dispatch_wait_sec", 1.0)
        self.declare_parameter("chat_history_max_entries", 24)

        self.mode = self.get_parameter("mode").value
        self.user_text_topic = self.get_parameter("user_text_topic").value
        self.assistant_text_topic = self.get_parameter("assistant_text_topic").value
        self.intent_topic = self.get_parameter("intent_topic").value
        self.posture_command_topic = self.get_parameter("posture_command_topic").value
        self.backend_fallback_to_rules = self._as_bool(
            self.get_parameter("backend_fallback_to_rules").value
        )
        self.backend_response_timeout_sec = max(
            0.1,
            float(self.get_parameter("backend_response_timeout_sec").value),
        )
        self.backend_execute_posture_after_response = self._as_bool(
            self.get_parameter("backend_execute_posture_after_response").value
        )
        self.backend_posture_from_response_enabled = self._as_bool(
            self.get_parameter("backend_posture_from_response_enabled").value
        )
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self.use_posture_skill = self._as_bool(
            self.get_parameter("use_posture_skill").value
        )
        self.posture_skill_action = self.get_parameter("posture_skill_action").value
        self.posture_skill_speed = float(
            self.get_parameter("posture_skill_speed").value
        )
        self.posture_skill_dispatch_wait_sec = max(
            0.0,
            float(self.get_parameter("posture_skill_dispatch_wait_sec").value),
        )
        self.use_chat_skill = self._as_bool(
            self.get_parameter("use_chat_skill").value
        )
        self.chat_skill_action = self.get_parameter("chat_skill_action").value
        self.chat_skill_dispatch_wait_sec = max(
            0.0,
            float(self.get_parameter("chat_skill_dispatch_wait_sec").value),
        )
        self.chat_history_max_entries = max(
            0, int(self.get_parameter("chat_history_max_entries").value)
        )

        self.pending_backend_fallback_intent = None
        self.pending_backend_posture_intent = None
        self.pending_backend_user_text = ""
        self.pending_backend_timer = None
        self._last_user_text = ""
        self._last_user_text_ts = 0.0
        self.conversation_history = []
        self.posture_client = None
        self.chat_client = None

        self.assistant_publisher = self.create_publisher(
            String, self.assistant_text_topic, 10
        )
        self.intent_publisher = self.create_publisher(String, self.intent_topic, 10)
        self.posture_command_publisher = self.create_publisher(
            String, self.posture_command_topic, 10
        )

        self.create_subscription(String, self.user_text_topic, self._on_user_text, 10)

        if self.use_posture_skill:
            self.posture_client = PostureSkillClient(
                self,
                action_name=self.posture_skill_action,
                default_speed=self.posture_skill_speed,
            )
            if self.posture_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info(
                    f'Posture skill server connected at "{self.posture_skill_action}"'
                )
            else:
                self.get_logger().warn(
                    "Posture skill server unavailable at startup; will retry and use topic fallback until it appears"
                )
        else:
            self.get_logger().info("Using topic-based posture commands")

        if self.use_chat_skill:
            self.chat_client = ChatSkillClient(self, action_name=self.chat_skill_action)
            if self.chat_client.wait_for_server(timeout_sec=1.5):
                self.get_logger().info(
                    f'Chat skill server connected at "{self.chat_skill_action}"'
                )
            else:
                self.get_logger().warn(
                    "Chat skill server unavailable at startup; backend mode will rely on rules fallback if enabled"
                )

        self.get_logger().info(
            "mission_controller ready | mode:%s user:%s assistant:%s intent:%s "
            "posture:%s use_chat_skill:%s"
            % (
                self.mode,
                self.user_text_topic,
                self.assistant_text_topic,
                self.intent_topic,
                self.posture_command_topic,
                self.use_chat_skill,
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
            self.get_logger().warn(
                f'Ignored duplicate user request within {self.dedupe_window_sec}s'
            )
            return
        self._last_user_text = text
        self._last_user_text_ts = now

        intent = detect_intent(text)
        self._publish(self.intent_publisher, intent)

        if self.mode == "backend":
            if self.backend_execute_posture_after_response:
                self.pending_backend_posture_intent = intent
                if posture_command_for_intent(intent):
                    self.get_logger().info(
                        f"Deferred posture execution until backend response | intent={intent}"
                    )
            else:
                self._handle_posture_intent(intent)

            self.pending_backend_user_text = text

            if self._dispatch_chat_skill_request(text, intent):
                return

            self.pending_backend_fallback_intent = None
            self._cancel_backend_fallback_timer()
            self._handle_backend_no_chat_response(
                intent,
                reason="Chat skill dispatch failed",
                posture_source="chat_skill_dispatch_failure_fallback",
            )
            return

        self._handle_posture_intent(intent)
        response = build_rule_response(intent)
        self._publish(self.assistant_publisher, response)
        self._append_conversation_turn(text, response)
        self.get_logger().info(
            f'Published rule response to "{self.assistant_text_topic}" | intent={intent}'
        )

    def _dispatch_chat_skill_request(self, text: str, intent: str) -> bool:
        if not self.use_chat_skill or self.chat_client is None:
            self.get_logger().warn(
                "Chat skill is disabled/unavailable in backend mode; cannot dispatch"
            )
            return False

        if not self.chat_client.ensure_server(
            wait_timeout_sec=self.chat_skill_dispatch_wait_sec
        ):
            self.get_logger().warn(
                f'Chat skill server "{self.chat_skill_action}" unavailable at dispatch'
            )
            return False

        sent = self.chat_client.send_goal(
            user_message=text,
            conversation_history=list(self.conversation_history),
            result_callback=self._on_chat_skill_result,
        )
        if not sent:
            self.get_logger().warn("Chat skill goal submission failed")
            return False

        self._schedule_backend_fallback(intent)
        self.get_logger().info(
            f'Forwarded request to chat skill "{self.chat_skill_action}" | intent={intent}'
        )
        return True

    def _on_chat_skill_result(self, result) -> None:
        if self.mode != "backend":
            return
        if self.pending_backend_fallback_intent is None:
            self.get_logger().warn("Ignored stale chat skill result")
            return

        request_intent = self.pending_backend_fallback_intent
        self._cancel_backend_fallback_timer()
        self.pending_backend_fallback_intent = None

        response = result.assistant_response.strip()
        fallback_applied = False
        if result.updated_history:
            self.conversation_history = self._trim_chat_history(result.updated_history)

        if not response and self.backend_fallback_to_rules:
            fallback_intent = request_intent or self.pending_backend_posture_intent or "fallback"
            response = build_rule_response(fallback_intent)
            fallback_applied = True
            self.get_logger().warn(
                "Chat skill returned empty response; published rule-based fallback"
            )

        if response:
            self._publish(self.assistant_publisher, response)
            if fallback_applied or not result.updated_history:
                self._append_conversation_turn(self.pending_backend_user_text, response)
            self.get_logger().info(
                f'Forwarded chat skill response to "{self.assistant_text_topic}"'
            )
        else:
            self.get_logger().warn("Chat skill returned no response and no fallback applied")

        self.pending_backend_user_text = ""
        pending_intent = self.pending_backend_posture_intent
        self.pending_backend_posture_intent = None
        if self.backend_execute_posture_after_response and pending_intent:
            self._handle_posture_intent(
                pending_intent,
                source="chat_skill_response_for_user_intent",
            )
            return
        if self.backend_posture_from_response_enabled and response:
            self._handle_posture_intent(detect_intent(response), source="chat_skill_response")

    def _schedule_backend_fallback(self, intent: str) -> None:
        self.pending_backend_fallback_intent = intent
        self._cancel_backend_fallback_timer()
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
        intent = self.pending_backend_fallback_intent
        if not intent:
            return

        self.pending_backend_fallback_intent = None
        self._handle_backend_no_chat_response(
            intent,
            reason="Chat skill response timeout",
            posture_source="chat_skill_timeout_fallback",
        )

    def _handle_backend_no_chat_response(
        self,
        intent: str,
        reason: str,
        posture_source: str,
    ) -> None:
        if self.backend_fallback_to_rules:
            fallback = build_rule_response(intent)
            self._publish(self.assistant_publisher, fallback)
            self._append_conversation_turn(self.pending_backend_user_text, fallback)
            self.get_logger().warn(f"{reason}; published rule-based fallback response")
        else:
            self.get_logger().warn(
                f"{reason}; no fallback published (backend_fallback_to_rules is false)"
            )

        self.pending_backend_user_text = ""
        pending_intent = self.pending_backend_posture_intent
        self.pending_backend_posture_intent = None
        if (
            self.backend_fallback_to_rules
            and self.backend_execute_posture_after_response
            and pending_intent
        ):
            self._handle_posture_intent(
                pending_intent,
                source=posture_source,
            )

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

    def _append_conversation_turn(self, user_text: str, assistant_text: str) -> None:
        if user_text.strip():
            self.conversation_history.append(f"user:{user_text.strip()}")
        if assistant_text.strip():
            self.conversation_history.append(f"assistant:{assistant_text.strip()}")
        self.conversation_history = self._trim_chat_history(self.conversation_history)

    def _trim_chat_history(self, history_entries) -> list[str]:
        cleaned = [str(entry).strip() for entry in history_entries if str(entry).strip()]
        if self.chat_history_max_entries > 0 and len(cleaned) > self.chat_history_max_entries:
            return cleaned[-self.chat_history_max_entries :]
        return cleaned

    def _handle_posture_intent(self, intent: str, source: str = "user_text") -> None:
        """Handle posture intent via skill action or topic fallback."""
        command = posture_command_for_intent(intent)
        if not command:
            return
        if self.use_posture_skill and self.posture_client is not None:
            if not self.posture_client.ensure_server(
                wait_timeout_sec=self.posture_skill_dispatch_wait_sec
            ):
                self.get_logger().warn(
                    f'Posture skill server "{self.posture_skill_action}" unavailable at dispatch; using topic fallback'
                )
            else:
                posture_map = {
                    "stand": "Stand",
                    "sit": "Sit",
                    "kneel": "Crouch",
                }
                posture_name = posture_map.get(command.lower(), "Stand")
                self.get_logger().info(
                    f"Executing posture via skill: {posture_name} (intent: {intent}, source: {source})"
                )
                self.posture_client.send_goal(
                    posture_name=posture_name,
                    speed=self.posture_skill_speed,
                    result_callback=lambda result: self.get_logger().info(
                        f"Posture result: {result.message}"
                    ),
                )
                return

        self._publish(self.posture_command_publisher, command)
        self.get_logger().info(
            f'Published posture command "{command}" to "{self.posture_command_topic}"'
            f" (source: {source})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
