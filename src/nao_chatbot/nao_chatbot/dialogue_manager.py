#!/usr/bin/env python3
"""Dialogue manager node for ROS4HRI conversational orchestration."""

import json
import time

import rclpy
from hri_msgs.msg import LiveSpeech
from rclpy.node import Node
from std_msgs.msg import String

from nao_chatbot.asr_utils import extract_text
from nao_chatbot.say_skill_client import SaySkillClient


class DialogueManager(Node):
    """Coordinate user/assistant turns and speech-skill dispatch."""

    def __init__(self) -> None:
        super().__init__("dialogue_manager")

        self.declare_parameter(
            "input_speech_topic", "/humans/voices/anonymous_speaker/speech"
        )
        self.declare_parameter("user_text_topic", "/chatbot/user_text")
        self.declare_parameter("assistant_text_topic", "/chatbot/assistant_text")
        self.declare_parameter("intent_topic", "/chatbot/intent")
        self.declare_parameter("dialogue_state_topic", "/chatbot/dialogue_state")
        self.declare_parameter("naoqi_speech_topic", "/speech")
        self.declare_parameter("use_say_skill", True)
        self.declare_parameter("say_skill_action", "/skill/say")
        self.declare_parameter("say_skill_language", "en-US")
        self.declare_parameter("say_skill_volume", 1.0)
        self.declare_parameter("say_skill_dispatch_wait_sec", 0.8)
        self.declare_parameter("also_publish_speech_topic", True)
        self.declare_parameter("fallback_publish_speech_topic", True)
        self.declare_parameter("reply_prefix", "")
        self.declare_parameter("dedupe_window_sec", 0.8)

        self.input_speech_topic = self.get_parameter("input_speech_topic").value
        self.user_text_topic = self.get_parameter("user_text_topic").value
        self.assistant_text_topic = self.get_parameter("assistant_text_topic").value
        self.intent_topic = self.get_parameter("intent_topic").value
        self.dialogue_state_topic = self.get_parameter("dialogue_state_topic").value
        self.naoqi_speech_topic = self.get_parameter("naoqi_speech_topic").value
        self.use_say_skill = self._as_bool(self.get_parameter("use_say_skill").value)
        self.say_skill_action = self.get_parameter("say_skill_action").value
        self.say_skill_language = self.get_parameter("say_skill_language").value
        self.say_skill_volume = float(self.get_parameter("say_skill_volume").value)
        self.say_skill_dispatch_wait_sec = max(
            0.0,
            float(self.get_parameter("say_skill_dispatch_wait_sec").value),
        )
        self.also_publish_speech_topic = self._as_bool(
            self.get_parameter("also_publish_speech_topic").value
        )
        self.fallback_publish_speech_topic = self._as_bool(
            self.get_parameter("fallback_publish_speech_topic").value
        )
        self.reply_prefix = self.get_parameter("reply_prefix").value
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)

        self._last_user_text = ""
        self._last_user_text_ts = 0.0
        self._last_assistant_text = ""
        self._last_assistant_text_ts = 0.0
        self._awaiting_assistant_turn = False
        self._latest_intent = ""
        self._last_state_payload = ""
        self._speech_topic_sent_for_current_turn = False
        self._pending_assistant_text = ""

        self.user_text_publisher = self.create_publisher(String, self.user_text_topic, 10)
        self.naoqi_speech_publisher = self.create_publisher(
            String, self.naoqi_speech_topic, 10
        )
        self.dialogue_state_publisher = self.create_publisher(
            String, self.dialogue_state_topic, 10
        )

        self.say_client = None
        if self.use_say_skill:
            self.say_client = SaySkillClient(
                self,
                action_name=self.say_skill_action,
                default_language=self.say_skill_language,
                default_volume=self.say_skill_volume,
            )
            if self.say_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info(
                    f'Say skill server connected at "{self.say_skill_action}"'
                )
            else:
                self.get_logger().warn(
                    "Say skill server unavailable at startup; topic fallback remains active"
                )
        else:
            self.get_logger().info("Say skill disabled; using speech topic publisher only")

        self.create_subscription(
            LiveSpeech, self.input_speech_topic, self._on_live_speech, 10
        )
        self.create_subscription(
            String, self.assistant_text_topic, self._on_assistant_text, 10
        )
        self.create_subscription(String, self.intent_topic, self._on_intent, 10)

        self._set_state("idle", "startup")
        self.get_logger().info(
            "dialogue_manager ready | in:%s user:%s assistant:%s say:%s speech:%s state:%s"
            % (
                self.input_speech_topic,
                self.user_text_topic,
                self.assistant_text_topic,
                self.say_skill_action,
                self.naoqi_speech_topic,
                self.dialogue_state_topic,
            )
        )

    def _on_live_speech(self, msg: LiveSpeech) -> None:
        text = extract_text(msg)
        if not text:
            self.get_logger().warn("LiveSpeech received without usable text")
            return

        now = time.monotonic()
        if (
            text == self._last_user_text
            and now - self._last_user_text_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(
                f'Ignored duplicate user text within {self.dedupe_window_sec}s'
            )
            return

        self._last_user_text = text
        self._last_user_text_ts = now
        self._awaiting_assistant_turn = True
        self._speech_topic_sent_for_current_turn = False
        self._pending_assistant_text = ""

        self._publish_text(self.user_text_publisher, text)
        self._set_state("thinking", "user_turn_forwarded")
        self.get_logger().info(f'Forwarded user text to "{self.user_text_topic}": {text}')

    def _on_assistant_text(self, msg: String) -> None:
        text = f"{self.reply_prefix}{msg.data}".strip()
        if not text:
            return

        now = time.monotonic()
        if (
            text == self._last_assistant_text
            and now - self._last_assistant_text_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(
                f'Ignored duplicate assistant text within {self.dedupe_window_sec}s'
            )
            return

        self._last_assistant_text = text
        self._last_assistant_text_ts = now
        self._awaiting_assistant_turn = False
        self._pending_assistant_text = text

        self._set_state("speaking", "assistant_turn_received")
        self._dispatch_say_turn(text)

    def _on_intent(self, msg: String) -> None:
        self._latest_intent = msg.data.strip()
        if self._awaiting_assistant_turn and self._latest_intent:
            self._set_state("thinking", "intent_detected")

    def _dispatch_say_turn(self, text: str) -> None:
        sent_via_say = False
        if self.use_say_skill and self.say_client is not None:
            if self.say_client.ensure_server(
                wait_timeout_sec=self.say_skill_dispatch_wait_sec
            ):
                sent_via_say = self.say_client.send_goal(
                    text=text,
                    language=self.say_skill_language,
                    volume=self.say_skill_volume,
                    result_callback=self._on_say_result,
                )
            else:
                self.get_logger().warn(
                    f'Say skill server "{self.say_skill_action}" unavailable at dispatch'
                )

        if self.also_publish_speech_topic:
            self._publish_speech_topic(text)
            self._speech_topic_sent_for_current_turn = True

        if sent_via_say:
            self.get_logger().info(f"Assistant turn dispatched to say skill: {text}")
            return

        if self.fallback_publish_speech_topic and not self._speech_topic_sent_for_current_turn:
            self._publish_speech_topic(text)
            self._speech_topic_sent_for_current_turn = True

        self._set_state("idle", "assistant_turn_published_topic_only")

    def _on_say_result(self, result) -> None:
        if result.success:
            self._set_state("idle", "assistant_turn_spoken")
            return

        self.get_logger().warn(f"Say skill failed: {result.message}")
        if self.fallback_publish_speech_topic and not self._speech_topic_sent_for_current_turn:
            if self._pending_assistant_text:
                self._publish_speech_topic(self._pending_assistant_text)
                self._speech_topic_sent_for_current_turn = True
        self._set_state("idle", "assistant_turn_say_failed")

    def _set_state(self, state: str, reason: str) -> None:
        payload = {
            "state": state,
            "reason": reason,
            "awaiting_assistant_turn": self._awaiting_assistant_turn,
            "latest_intent": self._latest_intent,
        }
        encoded = json.dumps(payload, separators=(",", ":"))
        if encoded == self._last_state_payload:
            return
        self._last_state_payload = encoded
        self._publish_text(self.dialogue_state_publisher, encoded)

    def _publish_speech_topic(self, text: str) -> None:
        self._publish_text(self.naoqi_speech_publisher, text)
        self.get_logger().info(
            f'Published robot speech on "{self.naoqi_speech_topic}": {text}'
        )

    @staticmethod
    def _publish_text(publisher, text: str) -> None:
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DialogueManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
