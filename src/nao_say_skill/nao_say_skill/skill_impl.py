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
        self.declare_parameter("debug_speech_topic", "/debug/nao_say/speech")
        self.declare_parameter("default_language", "en-US")
        self.declare_parameter("default_volume", 1.0)
        self.declare_parameter("tts_server_wait_sec", 0.5)
        self.declare_parameter("also_publish_debug_topic", True)
        self.declare_parameter("fallback_to_debug_topic", True)

        self.action_name = str(self.get_parameter("say_action_name").value)
        self.tts_action_name = str(self.get_parameter("tts_action_name").value)
        self.debug_speech_topic = str(self.get_parameter("debug_speech_topic").value)
        self.default_language = str(self.get_parameter("default_language").value)
        self.default_volume = float(self.get_parameter("default_volume").value)
        self.tts_server_wait_sec = max(
            0.0,
            float(self.get_parameter("tts_server_wait_sec").value),
        )
        self.also_publish_debug_topic = self._as_bool(
            self.get_parameter("also_publish_debug_topic").value
        )
        self.fallback_to_debug_topic = self._as_bool(
            self.get_parameter("fallback_to_debug_topic").value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._execution_lock = threading.Lock()
        self._action_server = None
        self._tts_client = None
        self._debug_speech_pub = None
        self._diag_pub = None
        self._diag_timer = None
        self._is_active = False
        self._stats = _RuntimeStats()

        self.get_logger().info("nao_say_skill created; waiting for lifecycle configure")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        self._tts_client = ActionClient(
            self,
            TTS,
            self.tts_action_name,
            callback_group=self._callback_group,
        )
        self._debug_speech_pub = self.create_publisher(
            String, self.debug_speech_topic, 10
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._action_server = ActionServer(
            self,
            Say,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "nao_say_skill configured | action:%s tts:%s debug:%s"
            % (
                self.action_name,
                self.tts_action_name,
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
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._tts_client is not None:
            self._tts_client.destroy()
            self._tts_client = None
        if self._diag_timer is not None:
            self.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._debug_speech_pub is not None:
            self.destroy_publisher(self._debug_speech_pub)
            self._debug_speech_pub = None
        self.get_logger().info("nao_say_skill shutdown complete")
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self, goal_request: Say.Goal) -> GoalResponse:
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

    def cancel_callback(self, goal_handle) -> CancelResponse:
        turn_id = self._extract_turn_id(goal_handle.request)
        self._trace(turn_id, "SAY_CANCEL", "cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
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
                goal_handle=goal_handle,
                text=text,
                language=self.default_language,
                volume=self.default_volume,
                turn_id=turn_id,
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

    async def _run_execution(
        self,
        goal_handle,
        text: str,
        language: str,
        volume: float,
        turn_id: str,
    ) -> tuple[str, str, float]:
        start_time = time.monotonic()
        if not text:
            return ("aborted", "Goal text is empty", 0.0)
        if volume < 0.0 or volume > 1.0:
            return ("aborted", "Goal volume is invalid", 0.0)

        self._publish_feedback(goal_handle, "preparing", 0.0)
        self._trace(
            turn_id,
            "SAY_START",
            "text_len=%d lang=%s volume=%.2f" % (len(text), language, volume),
        )
        if goal_handle.is_cancel_requested:
            return ("canceled", "Cancelled before execution", 0.0)

        if self.also_publish_debug_topic:
            self._publish_debug_speech(text, turn_id)

        tts_status, message = await self._forward_tts_goal(
            goal_handle=goal_handle,
            text=text,
            language=language,
            volume=volume,
            turn_id=turn_id,
        )
        duration = time.monotonic() - start_time
        if tts_status == GoalStatus.STATUS_CANCELED:
            return ("canceled", message, duration)
        if tts_status != GoalStatus.STATUS_SUCCEEDED:
            if self.fallback_to_debug_topic:
                if not self.also_publish_debug_topic:
                    self._publish_debug_speech(text, turn_id)
                return (
                    "succeeded",
                    f"{message}; delivered via debug topic fallback",
                    duration,
                )
            return ("aborted", message, duration)

        self._publish_feedback(goal_handle, "completing", 1.0)
        return ("succeeded", "Spoken via TTS action server", duration)

    async def _forward_tts_goal(
        self,
        goal_handle,
        text: str,
        language: str,
        volume: float,
        turn_id: str,
    ) -> tuple[int, str]:
        if self._tts_client is None:
            return (GoalStatus.STATUS_ABORTED, "TTS client is unavailable")
        if not self._tts_client.wait_for_server(timeout_sec=self.tts_server_wait_sec):
            return (
                GoalStatus.STATUS_ABORTED,
                f'TTS action "{self.tts_action_name}" is unavailable',
            )

        if goal_handle.is_cancel_requested:
            return (GoalStatus.STATUS_CANCELED, "Cancelled before TTS dispatch")

        self._publish_feedback(goal_handle, "speaking", 0.4)
        tts_goal = TTS.Goal()
        tts_goal.input = text
        tts_goal.locale = language.replace("-", "_")

        self._trace(
            turn_id,
            "SAY_TTS_FORWARD",
            "tts=%s lang=%s volume=%.2f"
            % (self.tts_action_name, language, volume),
        )
        send_goal_future = self._tts_client.send_goal_async(tts_goal)
        tts_goal_handle = await send_goal_future
        if tts_goal_handle is None or not tts_goal_handle.accepted:
            return (GoalStatus.STATUS_ABORTED, "TTS goal rejected by server")

        if goal_handle.is_cancel_requested:
            cancel_future = tts_goal_handle.cancel_goal_async()
            await cancel_future
            return (GoalStatus.STATUS_CANCELED, "Cancelled while waiting for TTS result")

        result_future = tts_goal_handle.get_result_async()
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
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    @staticmethod
    def _extract_turn_id(goal: Say.Goal) -> str:
        for raw_value in (goal.group_id, goal.person_id):
            value = str(raw_value).strip()
            if not value:
                continue
            if value.startswith("turn:"):
                return value[5:].strip() or "unknown"
            if value.startswith("turn_id:"):
                return value[8:].strip() or "unknown"
            return value
        return "unknown"

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
