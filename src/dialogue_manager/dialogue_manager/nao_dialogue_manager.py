#!/usr/bin/env python3
"""Dialogue manager node for NAO + ROS4HRI conversational orchestration."""

import json
import time

from hri_msgs.msg import LiveSpeech
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dialogue_manager.nao_asr_utils import extract_text
from dialogue_manager.nao_asr_utils import merge_text_chunks
from dialogue_manager.say_skill_client import SaySkillClient


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
        self.declare_parameter("accept_incremental_speech", False)
        self.declare_parameter("ignore_user_speech_while_busy", True)
        self.declare_parameter("user_turn_holdoff_sec", 0.6)
        self.declare_parameter("user_turn_min_chars", 2)
        self.declare_parameter("user_turn_min_words", 1)

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
        self.accept_incremental_speech = self._as_bool(
            self.get_parameter("accept_incremental_speech").value
        )
        self.ignore_user_speech_while_busy = self._as_bool(
            self.get_parameter("ignore_user_speech_while_busy").value
        )
        self.user_turn_holdoff_sec = max(
            0.0,
            float(self.get_parameter("user_turn_holdoff_sec").value),
        )
        self.user_turn_min_chars = max(
            0,
            int(self.get_parameter("user_turn_min_chars").value),
        )
        self.user_turn_min_words = max(
            1,
            int(self.get_parameter("user_turn_min_words").value),
        )

        self._last_user_text = ""
        self._last_user_text_ts = 0.0
        self._last_assistant_text = ""
        self._last_assistant_text_ts = 0.0
        self._awaiting_assistant_turn = False
        self._latest_intent = ""
        self._last_state_payload = ""
        self._speech_topic_sent_for_current_turn = False
        self._pending_assistant_text = ""
        self._pending_assistant_turn_id = ""
        self._pending_user_text = ""
        self._pending_user_locale = ""
        self._pending_user_confidence = 0.0
        self._pending_user_deadline_ts = 0.0
        self._current_turn_id = ""
        self._turn_counter = 0

        self.user_text_publisher = self.create_publisher(String, self.user_text_topic, 10)
        self.naoqi_speech_publisher = self.create_publisher(
            String, self.naoqi_speech_topic, 10
        )
        self.dialogue_state_publisher = self.create_publisher(
            String, self.dialogue_state_topic, 10
        )
        self._user_turn_timer = self.create_timer(0.1, self._flush_pending_user_turn_if_due)

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
            "dialogue_manager ready | in:%s user:%s assistant:%s say:%s speech:%s state:%s "
            "incremental:%s holdoff:%.2fs busy_guard:%s"
            % (
                self.input_speech_topic,
                self.user_text_topic,
                self.assistant_text_topic,
                self.say_skill_action,
                self.naoqi_speech_topic,
                self.dialogue_state_topic,
                str(self.accept_incremental_speech),
                self.user_turn_holdoff_sec,
                str(self.ignore_user_speech_while_busy),
            )
        )

    def _on_live_speech(self, msg: LiveSpeech) -> None:
        text = extract_text(
            msg,
            allow_incremental=self.accept_incremental_speech,
        )
        if not text:
            self.get_logger().warn("LiveSpeech received without usable final text")
            return

        if not self._should_accept_user_text(text):
            self.get_logger().warn(
                'Ignored short ASR text: "%s"' % self._preview_text(text)
            )
            return

        if self._awaiting_assistant_turn and self.ignore_user_speech_while_busy:
            self._trace(
                self._current_turn_id,
                "USER_IGNORED_BUSY",
                'text="%s"' % self._preview_text(text),
                level="warn",
            )
            return

        now = time.monotonic()
        if self.user_turn_holdoff_sec <= 0.0:
            self._forward_user_text(text, now)
            return

        merged_text = merge_text_chunks(self._pending_user_text, text)
        self._pending_user_text = merged_text
        self._pending_user_locale = str(getattr(msg, "locale", "")).strip()
        self._pending_user_confidence = max(
            float(getattr(msg, "confidence", 0.0)),
            float(self._pending_user_confidence),
        )
        self._pending_user_deadline_ts = now + self.user_turn_holdoff_sec
        self._trace(
            self._current_turn_id,
            "USER_BUFFERED",
            'text="%s" holdoff=%.2fs'
            % (self._preview_text(self._pending_user_text), self.user_turn_holdoff_sec),
        )

    def _flush_pending_user_turn_if_due(self) -> None:
        if not self._pending_user_text:
            return

        now = time.monotonic()
        if now < self._pending_user_deadline_ts:
            return

        text = self._pending_user_text
        self._pending_user_text = ""
        self._pending_user_locale = ""
        self._pending_user_confidence = 0.0
        self._pending_user_deadline_ts = 0.0
        self._forward_user_text(text, now)

    def _forward_user_text(self, text: str, now: float | None = None) -> None:
        current_time = time.monotonic() if now is None else float(now)
        if (
            text == self._last_user_text
            and current_time - self._last_user_text_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(
                f'Ignored duplicate user text within {self.dedupe_window_sec}s'
            )
            return

        turn_id = self._next_turn_id()
        self._last_user_text = text
        self._last_user_text_ts = current_time
        self._awaiting_assistant_turn = True
        self._speech_topic_sent_for_current_turn = False
        self._pending_assistant_text = ""
        self._pending_assistant_turn_id = turn_id
        self._current_turn_id = turn_id

        self._publish_text(
            self.user_text_publisher, self._encode_text_payload(text, turn_id)
        )
        self._set_state("thinking", "user_turn_forwarded")
        self._trace(
            turn_id,
            "USER_FORWARDED",
            'topic="%s" text="%s"'
            % (self.user_text_topic, self._preview_text(text)),
        )

    def _should_accept_user_text(self, text: str) -> bool:
        clean = " ".join(str(text).split())
        if len(clean) < self.user_turn_min_chars:
            return False
        word_count = len([token for token in clean.split(" ") if token])
        if word_count < self.user_turn_min_words:
            return False
        return True

    def _on_assistant_text(self, msg: String) -> None:
        parsed_text, parsed_turn_id = self._decode_text_payload(msg.data)
        text = f"{self.reply_prefix}{parsed_text}".strip()
        if not text:
            return
        turn_id = (
            parsed_turn_id or self._pending_assistant_turn_id or self._current_turn_id
        )

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
        self._pending_assistant_turn_id = turn_id
        if turn_id:
            self._current_turn_id = turn_id

        self._set_state("speaking", "assistant_turn_received")
        self._dispatch_say_turn(text, turn_id)

    def _on_intent(self, msg: String) -> None:
        intent_text, _intent_turn_id = self._decode_text_payload(msg.data)
        self._latest_intent = intent_text
        if self._awaiting_assistant_turn and self._latest_intent:
            self._set_state("thinking", "intent_detected")

    def _dispatch_say_turn(self, text: str, turn_id: str) -> None:
        sent_via_say = False
        if self.use_say_skill and self.say_client is not None:
            if self.say_client.ensure_server(
                wait_timeout_sec=self.say_skill_dispatch_wait_sec
            ):
                sent_via_say = self.say_client.send_goal(
                    text=text,
                    language=self.say_skill_language,
                    volume=self.say_skill_volume,
                    turn_id=turn_id,
                    result_callback=self._on_say_result,
                )
            else:
                self._trace(
                    turn_id,
                    "SAY_UNAVAILABLE",
                    'action="%s"' % self.say_skill_action,
                    level="warn",
                )

        if self.also_publish_speech_topic:
            self._publish_speech_topic(text, turn_id=turn_id)
            self._speech_topic_sent_for_current_turn = True

        if sent_via_say:
            self._trace(
                turn_id,
                "SAY_DISPATCHED",
                'action="%s" text_len=%d' % (self.say_skill_action, len(text)),
            )
            return

        if self.fallback_publish_speech_topic and not self._speech_topic_sent_for_current_turn:
            self._publish_speech_topic(text, turn_id=turn_id)
            self._speech_topic_sent_for_current_turn = True

        self._set_state("idle", "assistant_turn_published_topic_only")
        self._trace(turn_id, "TURN_DONE", "assistant turn published via topic only")
        self._pending_assistant_turn_id = ""

    def _on_say_result(self, result) -> None:
        turn_id = (
            str(getattr(result, "turn_id", "")).strip()
            or self._pending_assistant_turn_id
            or self._current_turn_id
            or "unknown"
        )
        if result.success:
            self._set_state("idle", "assistant_turn_spoken")
            self._trace(turn_id, "TURN_DONE", "assistant turn spoken")
            self._pending_assistant_turn_id = ""
            return

        self._trace(
            turn_id,
            "SAY_FAILED",
            result.message or "unknown",
            level="warn",
        )
        if self.fallback_publish_speech_topic and not self._speech_topic_sent_for_current_turn:
            if self._pending_assistant_text:
                self._publish_speech_topic(
                    self._pending_assistant_text,
                    turn_id=turn_id,
                )
                self._speech_topic_sent_for_current_turn = True
        self._set_state("idle", "assistant_turn_say_failed")
        self._trace(turn_id, "TURN_DONE", "assistant turn completed with say fallback")
        self._pending_assistant_turn_id = ""

    def _set_state(self, state: str, reason: str) -> None:
        payload = {
            "state": state,
            "reason": reason,
            "awaiting_assistant_turn": self._awaiting_assistant_turn,
            "latest_intent": self._latest_intent,
            "turn_id": self._current_turn_id or "unknown",
        }
        encoded = json.dumps(payload, separators=(",", ":"))
        if encoded == self._last_state_payload:
            return
        self._last_state_payload = encoded
        self._publish_text(self.dialogue_state_publisher, encoded)

    def _publish_speech_topic(self, text: str, turn_id: str = "") -> None:
        self._publish_text(self.naoqi_speech_publisher, text)
        self._trace(
            turn_id,
            "SPEECH_PUBLISHED",
            'topic="%s" text="%s"'
            % (self.naoqi_speech_topic, self._preview_text(text)),
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

    def _next_turn_id(self) -> str:
        self._turn_counter += 1
        return f"d{self._turn_counter:05d}"

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
        return clean[: max_len - 3] + "..."

    @staticmethod
    def _encode_text_payload(text: str, turn_id: str) -> str:
        return json.dumps(
            {
                "text": str(text).strip(),
                "turn_id": str(turn_id).strip(),
            },
            separators=(",", ":"),
        )

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
                parsed.get(
                    "intent",
                    parsed.get(
                        "user_text",
                        parsed.get("message", parsed.get("input", "")),
                    ),
                ),
            )
        ).strip()
        turn_id = str(parsed.get("turn_id", parsed.get("trace_id", ""))).strip()
        if not text:
            if turn_id:
                return "", turn_id
            return raw, ""
        return text, turn_id


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DialogueManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
