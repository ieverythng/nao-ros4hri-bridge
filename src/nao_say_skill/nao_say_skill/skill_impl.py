#!/usr/bin/env python3
"""Lifecycle implementation of the NAO-specific /nao/say skill."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from action_msgs.msg import GoalStatus
from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
from std_skills.msg import Result as SkillResult
from tts_msgs.action import TTS


@dataclass(slots=True)
class _RuntimeStats:
    goals_started: int = 0
    goals_succeeded: int = 0
    goals_failed: int = 0
    last_message: str = ""


class NaoSaySkill(Node):
    """Expose `/nao/say` and forward speech requests to the robot TTS stack."""

    def __init__(self) -> None:
        super().__init__("nao_say_skill")

        self.declare_parameter("say_action_name", "/nao/say")
        self.declare_parameter("tts_action_name", "/tts_engine/tts")
        self.declare_parameter("tts_backend_action_name", "")
        self.declare_parameter("speech_topic", "/speech")
        self.declare_parameter("debug_speech_topic", "/debug/nao_say/speech")
        self.declare_parameter("default_language", "en-US")
        self.declare_parameter("default_volume", 1.0)
        self.declare_parameter("tts_server_wait_sec", 0.5)
        self.declare_parameter("fallback_to_speech_topic", True)
        self.declare_parameter("also_publish_debug_topic", True)
        self.declare_parameter("fallback_to_debug_topic", True)

        self.action_name = str(self.get_parameter("say_action_name").value)
        self.tts_action_name = str(self.get_parameter("tts_action_name").value)
        self.tts_backend_action_name = str(
            self.get_parameter("tts_backend_action_name").value
        )
        self.speech_topic = str(self.get_parameter("speech_topic").value)
        self.debug_speech_topic = str(self.get_parameter("debug_speech_topic").value)
        self.default_language = str(self.get_parameter("default_language").value)
        self.default_volume = float(self.get_parameter("default_volume").value)
        self.tts_server_wait_sec = max(
            0.0,
            float(self.get_parameter("tts_server_wait_sec").value),
        )
        self.fallback_to_speech_topic = self._as_bool(
            self.get_parameter("fallback_to_speech_topic").value
        )
        self.also_publish_debug_topic = self._as_bool(
            self.get_parameter("also_publish_debug_topic").value
        )
        self.fallback_to_debug_topic = self._as_bool(
            self.get_parameter("fallback_to_debug_topic").value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._say_action_server = None
        self._tts_action_server = None
        self._tts_backend_client = None
        self._speech_pub = None
        self._debug_speech_pub = None
        self._diag_pub = None
        self._diag_timer = None
        self._is_active = False
        self._stats = _RuntimeStats()

        self.get_logger().info("nao_say_skill created; waiting for lifecycle configure")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._tts_backend_client = self._create_tts_backend_client()
        self._speech_pub = self.create_publisher(String, self.speech_topic, 10)
        self._debug_speech_pub = self.create_publisher(
            String,
            self.debug_speech_topic,
            10,
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._say_action_server = ActionServer(
            self,
            Say,
            self.action_name,
            execute_callback=self.execute_say_callback,
            goal_callback=self.say_goal_callback,
            cancel_callback=self.generic_cancel_callback,
            callback_group=self._callback_group,
        )
        self._tts_action_server = ActionServer(
            self,
            TTS,
            self.tts_action_name,
            execute_callback=self.execute_tts_callback,
            goal_callback=self.tts_goal_callback,
            cancel_callback=self.generic_cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "nao_say_skill configured | say:%s tts:%s backend:%s speech:%s debug:%s"
            % (
                self.action_name,
                self.tts_action_name,
                self.tts_backend_action_name or "(speech_topic_fallback)",
                self.speech_topic,
                self.debug_speech_topic,
            )
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = True
        self.get_logger().info("nao_say_skill active")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._is_active = False
        self.get_logger().info("nao_say_skill inactive")
        return super().on_deactivate(state)

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._is_active = False
        if self._say_action_server is not None:
            self._say_action_server.destroy()
            self._say_action_server = None
        if self._tts_action_server is not None:
            self._tts_action_server.destroy()
            self._tts_action_server = None
        if self._tts_backend_client is not None:
            self._tts_backend_client.destroy()
            self._tts_backend_client = None
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._speech_pub is not None:
            self.destroy_publisher(self._speech_pub)
            self._speech_pub = None
        if self._debug_speech_pub is not None:
            self.destroy_publisher(self._debug_speech_pub)
            self._debug_speech_pub = None
        self.get_logger().info("nao_say_skill shutdown complete")
        return TransitionCallbackReturn.SUCCESS

    def say_goal_callback(self, goal_request: Say.Goal) -> GoalResponse:
        turn_id = self._extract_turn_id(goal_request)
        if not self._is_active:
            self._trace(turn_id, "SAY_REJECTED", "node not active", level="warn")
            return GoalResponse.REJECT
        if self._execution_lock.locked():
            self._trace(turn_id, "SAY_REJECTED", "another goal is running", level="warn")
            return GoalResponse.REJECT
        if not goal_request.input.strip():
            self._trace(turn_id, "SAY_REJECTED", "empty text", level="warn")
            return GoalResponse.REJECT
        self._trace(turn_id, "SAY_ACCEPTED", "goal accepted")
        return GoalResponse.ACCEPT

    def tts_goal_callback(self, goal_request: TTS.Goal) -> GoalResponse:
        turn_id = self._extract_turn_id_from_text(goal_request.input)
        if not self._is_active:
            self._trace(turn_id, "TTS_REJECTED", "node not active", level="warn")
            return GoalResponse.REJECT
        if self._execution_lock.locked():
            self._trace(turn_id, "TTS_REJECTED", "another goal is running", level="warn")
            return GoalResponse.REJECT
        if not goal_request.input.strip():
            self._trace(turn_id, "TTS_REJECTED", "empty text", level="warn")
            return GoalResponse.REJECT
        self._trace(turn_id, "TTS_ACCEPTED", "goal accepted")
        return GoalResponse.ACCEPT

    def generic_cancel_callback(self, goal_handle) -> CancelResponse:
        turn_id = self._extract_turn_id(getattr(goal_handle, "request", goal_handle))
        self._trace(turn_id, "SAY_CANCEL", "cancel request received")
        return CancelResponse.ACCEPT

    async def execute_say_callback(self, goal_handle):
        turn_id = self._extract_turn_id(goal_handle.request)
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._canonical_result(
                False,
                "Another /nao/say goal is already executing",
                SkillResult.ROS_ECANCELED,
            )

        self._stats.goals_started += 1
        try:
            goal = goal_handle.request
            text = goal.input.strip()
            self._stats.last_message = text
            status, message, _duration = await self._run_execution(
                text=text,
                language=self.default_language,
                volume=self.default_volume,
                turn_id=turn_id,
                say_goal_handle=goal_handle,
                tts_goal_handle=None,
            )
            if status == "canceled":
                self._stats.goals_failed += 1
                goal_handle.canceled()
                return self._canonical_result(
                    False,
                    message,
                    SkillResult.ROS_ECANCELED,
                )
            if status == "aborted":
                self._stats.goals_failed += 1
                goal_handle.abort()
                return self._canonical_result(
                    False,
                    message,
                    SkillResult.ROS_EOTHER,
                )
            self._stats.goals_succeeded += 1
            goal_handle.succeed()
            return self._canonical_result(
                True,
                message,
                SkillResult.ROS_ENOERR,
            )
        finally:
            self._execution_lock.release()

    async def execute_tts_callback(self, goal_handle):
        turn_id = self._extract_turn_id_from_text(goal_handle.request.input)
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return self._tts_result("Another speech goal is already executing")

        self._stats.goals_started += 1
        try:
            goal = goal_handle.request
            text = goal.input.strip()
            self._stats.last_message = text
            language = self._normalize_language(goal.locale)
            status, message, _duration = await self._run_execution(
                text=text,
                language=language,
                volume=self.default_volume,
                turn_id=turn_id,
                say_goal_handle=None,
                tts_goal_handle=goal_handle,
            )
            if status == "canceled":
                self._stats.goals_failed += 1
                goal_handle.canceled()
                return self._tts_result(message)
            if status == "aborted":
                self._stats.goals_failed += 1
                goal_handle.abort()
                return self._tts_result(message)
            self._stats.goals_succeeded += 1
            goal_handle.succeed()
            return self._tts_result("")
        finally:
            self._execution_lock.release()

    async def _run_execution(
        self,
        text: str,
        language: str,
        volume: float,
        turn_id: str,
        say_goal_handle,
        tts_goal_handle,
    ) -> tuple[str, str, float]:
        start_time = time.monotonic()
        if not text:
            return ("aborted", "Goal text is empty", 0.0)
        if volume < 0.0 or volume > 1.0:
            return ("aborted", "Goal volume is invalid", 0.0)

        if say_goal_handle is not None:
            self._publish_feedback(say_goal_handle, "preparing", 0.0)
        self._trace(
            turn_id,
            "SAY_START",
            "text_len=%d lang=%s volume=%.2f" % (len(text), language, volume),
        )
        if self._is_cancel_requested(say_goal_handle, tts_goal_handle):
            return ("canceled", "Cancelled before execution", 0.0)

        if self.also_publish_debug_topic:
            self._publish_debug_speech(text, turn_id)

        tts_status, message = await self._forward_tts_goal(
            text=text,
            language=language,
            volume=volume,
            turn_id=turn_id,
            say_goal_handle=say_goal_handle,
            tts_goal_handle=tts_goal_handle,
        )
        duration = time.monotonic() - start_time
        if tts_status == GoalStatus.STATUS_CANCELED:
            return ("canceled", message, duration)
            if tts_status != GoalStatus.STATUS_SUCCEEDED:
                if self.fallback_to_speech_topic:
                    self._publish_speech_topic(text, turn_id)
                    if tts_goal_handle is not None:
                        self._publish_tts_feedback_words(tts_goal_handle, text)
                    return (
                        "succeeded",
                        f"{message}; delivered via speech topic fallback",
                    duration,
                )
            if self.fallback_to_debug_topic:
                if not self.also_publish_debug_topic:
                    self._publish_debug_speech(text, turn_id)
                return (
                    "succeeded",
                    f"{message}; delivered via debug topic fallback",
                    duration,
                )
            return ("aborted", message, duration)

        if say_goal_handle is not None:
            self._publish_feedback(say_goal_handle, "completing", 1.0)
        return ("succeeded", "Spoken via TTS action server", duration)

    async def _forward_tts_goal(
        self,
        text: str,
        language: str,
        volume: float,
        turn_id: str,
        say_goal_handle,
        tts_goal_handle,
    ) -> tuple[int, str]:
        if self._tts_backend_client is None:
            return (GoalStatus.STATUS_ABORTED, "No downstream TTS action is configured")
        if not self._tts_backend_client.wait_for_server(
            timeout_sec=self.tts_server_wait_sec
        ):
            return (
                GoalStatus.STATUS_ABORTED,
                f'TTS action "{self.tts_backend_action_name}" is unavailable',
            )

        if self._is_cancel_requested(say_goal_handle, tts_goal_handle):
            return (GoalStatus.STATUS_CANCELED, "Cancelled before TTS dispatch")

        if say_goal_handle is not None:
            self._publish_feedback(say_goal_handle, "speaking", 0.4)
        tts_goal = TTS.Goal()
        tts_goal.input = text
        tts_goal.locale = self._normalize_tts_locale(language)

        self._trace(
            turn_id,
            "SAY_TTS_FORWARD",
            "tts=%s lang=%s volume=%.2f"
            % (self.tts_backend_action_name, language, volume),
        )
        send_goal_future = self._tts_backend_client.send_goal_async(
            tts_goal,
            feedback_callback=(
                None
                if tts_goal_handle is None
                else lambda feedback: self._forward_tts_feedback(tts_goal_handle, feedback)
            ),
        )
        downstream_tts_goal_handle = await send_goal_future
        if downstream_tts_goal_handle is None or not downstream_tts_goal_handle.accepted:
            return (GoalStatus.STATUS_ABORTED, "TTS goal rejected by server")

        if self._is_cancel_requested(say_goal_handle, tts_goal_handle):
            cancel_future = downstream_tts_goal_handle.cancel_goal_async()
            await cancel_future
            return (GoalStatus.STATUS_CANCELED, "Cancelled while waiting for TTS result")

        result_future = downstream_tts_goal_handle.get_result_async()
        wrapped_result = await result_future
        if wrapped_result is None:
            return (GoalStatus.STATUS_ABORTED, "TTS result wrapper was empty")
        if wrapped_result.result and wrapped_result.result.error_msg:
            return (int(wrapped_result.status), wrapped_result.result.error_msg)
        return (int(wrapped_result.status), "TTS action completed")

    def _publish_debug_speech(self, text: str, turn_id: str) -> None:
        if self._debug_speech_pub is None:
            return
        msg = String()
        msg.data = text
        self._debug_speech_pub.publish(msg)
        self._trace(
            turn_id,
            "DEBUG_SPEECH_PUBLISHED",
            'topic="%s" text_len=%d' % (self.debug_speech_topic, len(text)),
        )

    def _publish_speech_topic(self, text: str, turn_id: str) -> None:
        if self._speech_pub is None:
            return
        msg = String()
        msg.data = text
        self._speech_pub.publish(msg)
        self._trace(
            turn_id,
            "SPEECH_TOPIC_PUBLISHED",
            'topic="%s" text_len=%d' % (self.speech_topic, len(text)),
        )

    def _create_tts_backend_client(self):
        backend_name = self.tts_backend_action_name.strip()
        if not backend_name:
            return None
        if backend_name == self.tts_action_name:
            self.get_logger().warn(
                "tts_backend_action_name matches tts_action_name; disabling downstream client "
                "to avoid a feedback loop"
            )
            return None
        return ActionClient(
            self,
            TTS,
            backend_name,
            callback_group=self._callback_group,
        )

    def _publish_tts_feedback_words(self, goal_handle, text: str) -> None:
        words = [word for word in text.split() if word]
        if not words:
            return
        for word in words:
            if goal_handle.is_cancel_requested:
                break
            feedback = TTS.Feedback()
            feedback.word = word
            goal_handle.publish_feedback(feedback)
            time.sleep(0.01)

    @staticmethod
    def _forward_tts_feedback(goal_handle, feedback_message) -> None:
        if feedback_message is None:
            return
        feedback = TTS.Feedback()
        feedback.word = str(feedback_message.feedback.word)
        goal_handle.publish_feedback(feedback)

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/nao_say_skill",
            message="nao_say_skill running",
            values=[
                KeyValue(key="state", value="active" if self._is_active else "inactive"),
                KeyValue(key="say_action_name", value=self.action_name),
                KeyValue(key="tts_action_name", value=self.tts_action_name),
                KeyValue(
                    key="tts_backend_action_name",
                    value=self.tts_backend_action_name or "(speech_topic_fallback)",
                ),
                KeyValue(key="speech_topic", value=self.speech_topic),
                KeyValue(key="goals_started", value=str(self._stats.goals_started)),
                KeyValue(key="goals_succeeded", value=str(self._stats.goals_succeeded)),
                KeyValue(key="goals_failed", value=str(self._stats.goals_failed)),
                KeyValue(key="last_message", value=self._stats.last_message[:120]),
            ],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = Say.Feedback()
        feedback.feedback.data_str = status
        feedback.feedback.data_float = float(progress)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _canonical_result(success: bool, message: str, error_code: int):
        result = Say.Result()
        result.result.error_code = int(error_code)
        result.result.error_msg = "" if success else message
        return result

    @staticmethod
    def _tts_result(message: str):
        result = TTS.Result()
        result.error_msg = str(message or "")
        return result

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    @staticmethod
    def _extract_turn_id(goal: Say.Goal) -> str:
        for raw_value in (
            getattr(goal, "group_id", ""),
            getattr(goal, "person_id", ""),
        ):
            value = str(raw_value).strip()
            if not value:
                continue
            if value.startswith("turn:"):
                return value[5:].strip() or "unknown"
            if value.startswith("turn_id:"):
                return value[8:].strip() or "unknown"
            return value
        return "unknown"

    @staticmethod
    def _extract_turn_id_from_text(text: str) -> str:
        clean_text = str(text).strip()
        if not clean_text:
            return "unknown"
        return clean_text[:32]

    @staticmethod
    def _normalize_tts_locale(language: str) -> str:
        return str(language or "").strip().replace("-", "_")

    @staticmethod
    def _normalize_language(language: str) -> str:
        clean = str(language or "").strip()
        if not clean:
            return "en-US"
        return clean.replace("_", "-")

    @staticmethod
    def _is_cancel_requested(say_goal_handle, tts_goal_handle) -> bool:
        if say_goal_handle is not None and say_goal_handle.is_cancel_requested:
            return True
        if tts_goal_handle is not None and tts_goal_handle.is_cancel_requested:
            return True
        return False

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
