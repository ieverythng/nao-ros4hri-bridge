import json
import time

from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nao_chatbot.chat_skill_client import ChatSkillClient
from nao_chatbot.head_motion_skill_client import HeadMotionSkillClient
from nao_chatbot.intent_rules import build_rule_response
from nao_chatbot.intent_rules import detect_intent
from nao_chatbot.intent_rules import head_motion_goal_for_intent
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
        self.declare_parameter("use_head_motion_skill", True)
        self.declare_parameter("head_motion_skill_action", "/skill/do_head_motion")
        self.declare_parameter("head_motion_skill_speed", 0.25)
        self.declare_parameter("head_motion_skill_dispatch_wait_sec", 1.0)
        self.declare_parameter("head_motion_joint_angles_topic", "/joint_angles")
        self.declare_parameter("head_motion_fallback_to_joint_topic", True)
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
        self.use_head_motion_skill = self._as_bool(
            self.get_parameter("use_head_motion_skill").value
        )
        self.head_motion_skill_action = self.get_parameter("head_motion_skill_action").value
        self.head_motion_skill_speed = float(
            self.get_parameter("head_motion_skill_speed").value
        )
        self.head_motion_skill_dispatch_wait_sec = max(
            0.0,
            float(self.get_parameter("head_motion_skill_dispatch_wait_sec").value),
        )
        self.head_motion_joint_angles_topic = self.get_parameter(
            "head_motion_joint_angles_topic"
        ).value
        self.head_motion_fallback_to_joint_topic = self._as_bool(
            self.get_parameter("head_motion_fallback_to_joint_topic").value
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
        self.pending_backend_user_text = ""
        self.pending_backend_turn_id = ""
        self.pending_backend_timer = None
        self._last_user_text = ""
        self._last_user_text_ts = 0.0
        self._turn_counter = 0
        self.conversation_history = []
        self.posture_client = None
        self.head_motion_client = None
        self.chat_client = None

        self.assistant_publisher = self.create_publisher(
            String, self.assistant_text_topic, 10
        )
        self.intent_publisher = self.create_publisher(String, self.intent_topic, 10)
        self.posture_command_publisher = self.create_publisher(
            String, self.posture_command_topic, 10
        )
        self.head_motion_topic_publisher = self.create_publisher(
            JointAnglesWithSpeed, self.head_motion_joint_angles_topic, 10
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

        if self.use_head_motion_skill:
            self.head_motion_client = HeadMotionSkillClient(
                self,
                action_name=self.head_motion_skill_action,
                default_speed=self.head_motion_skill_speed,
            )
            if self.head_motion_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info(
                    f'Head-motion skill server connected at "{self.head_motion_skill_action}"'
                )
            else:
                self.get_logger().warn(
                    "Head-motion skill server unavailable at startup; will retry and use topic fallback until it appears"
                )
        else:
            self.get_logger().info("Using topic-based head-motion commands")

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
            "posture:%s head:%s use_chat_skill:%s"
            % (
                self.mode,
                self.user_text_topic,
                self.assistant_text_topic,
                self.intent_topic,
                self.posture_command_topic,
                self.head_motion_joint_angles_topic,
                self.use_chat_skill,
            )
        )

    def _on_user_text(self, msg: String) -> None:
        text, incoming_turn_id = self._decode_text_payload(msg.data)
        if not text:
            return
        turn_id = incoming_turn_id or self._next_turn_id()
        self._trace(
            turn_id,
            "TURN_START",
            f'mode={self.mode} user="{self._preview_text(text)}"',
        )
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

        if self.mode == "backend":
            self.pending_backend_user_text = text
            self.pending_backend_turn_id = turn_id
            fallback_intent = (
                detect_intent(text) if self.backend_fallback_to_rules else "fallback"
            )

            if self._dispatch_chat_skill_request(text, fallback_intent, turn_id):
                return

            self.pending_backend_fallback_intent = None
            self.pending_backend_turn_id = ""
            self._cancel_backend_fallback_timer()
            self._handle_backend_no_chat_response(
                fallback_intent,
                reason="Chat skill dispatch failed",
                posture_source="chat_skill_dispatch_failure_fallback",
                turn_id=turn_id,
            )
            return

        intent = detect_intent(text)
        self._publish(self.intent_publisher, intent)
        self._trace(turn_id, "INTENT_PUBLISHED", f"{intent} source=rules")
        self._handle_posture_intent(intent, turn_id=turn_id)
        self._handle_head_motion_intent(intent, turn_id=turn_id)
        response = build_rule_response(intent)
        self._publish(self.assistant_publisher, response)
        self._append_conversation_turn(text, response)
        self._trace(
            turn_id,
            "ASSISTANT_PUBLISHED",
            f'topic="{self.assistant_text_topic}" intent={intent}',
        )
        self._trace(turn_id, "TURN_DONE", "rules path complete")

    def _dispatch_chat_skill_request(
        self,
        text: str,
        fallback_intent: str,
        turn_id: str,
    ) -> bool:
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
            turn_id=turn_id,
            result_callback=self._on_chat_skill_result,
        )
        if not sent:
            self.get_logger().warn("Chat skill goal submission failed")
            return False

        self._schedule_backend_fallback(fallback_intent)
        self._trace(
            turn_id,
            "CHAT_DISPATCH",
            f'action="{self.chat_skill_action}" fallback_intent={fallback_intent}',
        )
        return True

    def _on_chat_skill_result(self, result) -> None:
        turn_id = (
            str(getattr(result, "turn_id", "")).strip()
            or self.pending_backend_turn_id
            or "unknown"
        )
        if self.mode != "backend":
            return
        if self.pending_backend_fallback_intent is None:
            self._trace(turn_id, "CHAT_STALE", "ignored stale chat result", level="warn")
            return

        request_fallback_intent = self.pending_backend_fallback_intent or "fallback"
        self._cancel_backend_fallback_timer()
        self.pending_backend_fallback_intent = None
        self.pending_backend_turn_id = ""

        response = result.assistant_response.strip()
        fallback_applied = False
        if result.updated_history:
            self.conversation_history = self._trim_chat_history(result.updated_history)

        resolved_intent = self._resolve_backend_intent(
            backend_intent=result.intent,
            fallback_intent=request_fallback_intent,
        )

        if not response and self.backend_fallback_to_rules:
            response = build_rule_response(resolved_intent)
            fallback_applied = True
            self.get_logger().warn(
                "Chat skill returned empty response; published rule-based fallback"
            )

        if response:
            self._publish(self.assistant_publisher, response)
            if fallback_applied or not result.updated_history:
                self._append_conversation_turn(self.pending_backend_user_text, response)
            self._trace(
                turn_id,
                "ASSISTANT_PUBLISHED",
                f'topic="{self.assistant_text_topic}" len={len(response)}',
            )
        else:
            self._trace(
                turn_id,
                "ASSISTANT_EMPTY",
                "chat skill returned no response and no fallback applied",
                level="warn",
            )

        if resolved_intent:
            self._publish(self.intent_publisher, resolved_intent)
            self._trace(
                turn_id,
                "INTENT_PUBLISHED",
                "topic=\"%s\" intent=%s source=%s confidence=%.2f"
                % (
                    self.intent_topic,
                    resolved_intent,
                    result.intent_source or "unknown",
                    float(result.intent_confidence),
                ),
            )

        self.pending_backend_user_text = ""
        if self.backend_execute_posture_after_response and resolved_intent:
            self._handle_posture_intent(
                resolved_intent,
                source="chat_skill_result_intent",
                turn_id=turn_id,
            )
            self._handle_head_motion_intent(
                resolved_intent,
                source="chat_skill_result_intent",
                turn_id=turn_id,
            )
            self._trace(turn_id, "TURN_DONE", "backend path complete (motion dispatched)")
            return
        if self.backend_posture_from_response_enabled and response:
            self._handle_posture_intent(
                detect_intent(response),
                source="chat_skill_response",
                turn_id=turn_id,
            )
            self._handle_head_motion_intent(
                detect_intent(response),
                source="chat_skill_response",
                turn_id=turn_id,
            )
        self._trace(turn_id, "TURN_DONE", "backend path complete")

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
            turn_id=self.pending_backend_turn_id or "unknown",
        )
        self.pending_backend_turn_id = ""

    def _handle_backend_no_chat_response(
        self,
        intent: str,
        reason: str,
        posture_source: str,
        turn_id: str = "",
    ) -> None:
        if self.backend_fallback_to_rules:
            fallback = build_rule_response(intent)
            self._publish(self.assistant_publisher, fallback)
            self._publish(self.intent_publisher, intent)
            self._append_conversation_turn(self.pending_backend_user_text, fallback)
            self._trace(
                turn_id,
                "FALLBACK_RESPONSE",
                f"{reason}; published rule-based fallback response",
                level="warn",
            )
        else:
            self._trace(
                turn_id,
                "FALLBACK_SKIPPED",
                f"{reason}; no fallback published (backend_fallback_to_rules is false)",
                level="warn",
            )

        self.pending_backend_user_text = ""
        if self.backend_fallback_to_rules and self.backend_execute_posture_after_response:
            self._handle_posture_intent(
                intent,
                source=posture_source,
                turn_id=turn_id,
            )
            self._handle_head_motion_intent(
                intent,
                source=posture_source,
                turn_id=turn_id,
            )
        self._trace(turn_id, "TURN_DONE", "backend fallback complete")

    def _resolve_backend_intent(
        self,
        backend_intent: str,
        fallback_intent: str,
    ) -> str:
        clean_backend_intent = str(backend_intent).strip()
        if clean_backend_intent:
            return clean_backend_intent
        if self.backend_fallback_to_rules:
            return fallback_intent or "fallback"
        return "fallback"

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
    def _decode_text_payload(payload: str) -> tuple[str, str]:
        raw = str(payload).strip()
        if not raw:
            return "", ""
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return raw, ""
        if isinstance(parsed, str):
            return parsed.strip(), ""
        if not isinstance(parsed, dict):
            return raw, ""
        text = str(
            parsed.get(
                "text",
                parsed.get("user_text", parsed.get("message", parsed.get("input", ""))),
            )
        ).strip()
        turn_id = str(parsed.get("turn_id", parsed.get("trace_id", ""))).strip()
        if not text:
            if turn_id:
                return "", turn_id
            return raw, ""
        return text, turn_id

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

    def _handle_posture_intent(
        self,
        intent: str,
        source: str = "user_text",
        turn_id: str = "",
    ) -> None:
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
                self._trace(
                    turn_id,
                    "POSTURE_DISPATCH",
                    f"{posture_name} intent={intent} source={source}",
                )
                self.posture_client.send_goal(
                    posture_name=posture_name,
                    speed=self.posture_skill_speed,
                    turn_id=turn_id,
                    result_callback=lambda result: self.get_logger().info(
                        "%s POSTURE_RESULT | %s"
                        % (
                            self._turn_label(turn_id),
                            result.message,
                        )
                    ),
                )
                return

        self._publish(self.posture_command_publisher, command)
        self._trace(
            turn_id,
            "POSTURE_TOPIC_FALLBACK",
            'published "%s" to "%s" (source=%s)'
            % (command, self.posture_command_topic, source),
        )

    def _handle_head_motion_intent(
        self,
        intent: str,
        source: str = "user_text",
        turn_id: str = "",
    ) -> None:
        """Handle head-motion intent via skill action or topic fallback."""
        goal = head_motion_goal_for_intent(intent)
        if not goal:
            return

        yaw = float(goal.get("yaw", 0.0))
        pitch = float(goal.get("pitch", 0.0))
        relative = bool(goal.get("relative", False))

        if self.use_head_motion_skill and self.head_motion_client is not None:
            if not self.head_motion_client.ensure_server(
                wait_timeout_sec=self.head_motion_skill_dispatch_wait_sec
            ):
                self.get_logger().warn(
                    f'Head-motion skill server "{self.head_motion_skill_action}" unavailable at dispatch; using topic fallback'
                )
            else:
                self._trace(
                    turn_id,
                    "HEAD_DISPATCH",
                    "yaw=%.4f pitch=%.4f relative=%s intent=%s source=%s"
                    % (yaw, pitch, relative, intent, source),
                )
                self.head_motion_client.send_goal(
                    yaw=yaw,
                    pitch=pitch,
                    speed=self.head_motion_skill_speed,
                    relative=relative,
                    turn_id=turn_id,
                    result_callback=lambda result: self.get_logger().info(
                        "%s HEAD_RESULT | %s"
                        % (
                            self._turn_label(turn_id),
                            result.message,
                        )
                    ),
                )
                return

        if not self.head_motion_fallback_to_joint_topic:
            return

        msg = JointAnglesWithSpeed()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["HeadYaw", "HeadPitch"]
        msg.joint_angles = [yaw, pitch]
        msg.speed = float(self.head_motion_skill_speed)
        msg.relative = 1 if relative else 0
        self.head_motion_topic_publisher.publish(msg)
        self._trace(
            turn_id,
            "HEAD_TOPIC_FALLBACK",
            'published yaw=%.4f pitch=%.4f relative=%s to "%s" (source=%s)'
            % (
                yaw,
                pitch,
                relative,
                self.head_motion_joint_angles_topic,
                source,
            ),
        )

    def _next_turn_id(self) -> str:
        self._turn_counter += 1
        return f"t{self._turn_counter:05d}"

    def _trace(
        self,
        turn_id: str,
        stage: str,
        message: str,
        level: str = "info",
    ) -> None:
        logger = self.get_logger()
        line = f"{self._turn_label(turn_id)} {stage} | {message}"
        if level == "warn":
            logger.warn(line)
            return
        if level == "error":
            logger.error(line)
            return
        logger.info(line)

    @staticmethod
    def _turn_label(turn_id: str) -> str:
        clean_turn_id = str(turn_id).strip()
        if not clean_turn_id:
            return "[turn:unknown]"
        return f"[turn:{clean_turn_id}]"

    @staticmethod
    def _preview_text(text: str, max_len: int = 72) -> str:
        clean = " ".join(str(text).split())
        if len(clean) <= max_len:
            return clean
        return clean[: max_len - 1] + "…"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
