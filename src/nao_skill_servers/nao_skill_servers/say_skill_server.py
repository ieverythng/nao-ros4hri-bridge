#!/usr/bin/env python3
"""Action server wrapper for NAO speech/TTS execution."""

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from communication_skills.action import Say
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_skills.msg import Result as SkillResult
from tts_msgs.action import TTS


class SaySkillServer(Node):
    """Serve `/skill/say` goals by forwarding to `/tts_engine/tts`."""

    def __init__(self) -> None:
        super().__init__("say_skill_server")

        self.declare_parameter("action_name", "/skill/say")
        self.declare_parameter("tts_action_name", "/tts_engine/tts")
        self.declare_parameter("naoqi_speech_topic", "/speech")
        self.declare_parameter("default_language", "en-US")
        self.declare_parameter("default_volume", 1.0)
        self.declare_parameter("tts_server_wait_sec", 0.5)
        self.declare_parameter("also_publish_speech_topic", True)
        self.declare_parameter("fallback_to_speech_topic", True)

        self.action_name = str(self.get_parameter("action_name").value)
        self.tts_action_name = str(self.get_parameter("tts_action_name").value)
        self.naoqi_speech_topic = str(self.get_parameter("naoqi_speech_topic").value)
        self.default_language = str(self.get_parameter("default_language").value)
        self.default_volume = float(self.get_parameter("default_volume").value)
        self.tts_server_wait_sec = max(
            0.0,
            float(self.get_parameter("tts_server_wait_sec").value),
        )
        self.also_publish_speech_topic = self._as_bool(
            self.get_parameter("also_publish_speech_topic").value
        )
        self.fallback_to_speech_topic = self._as_bool(
            self.get_parameter("fallback_to_speech_topic").value
        )

        self._execution_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._speech_publisher = self.create_publisher(
            String, self.naoqi_speech_topic, 10
        )
        self._tts_client = ActionClient(self, TTS, self.tts_action_name)

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
            "say_skill_server ready | action:%s canonical_type:communication_skills/action/Say "
            "tts:%s speech:%s fallback:%s publish:%s"
            % (
                self.action_name,
                self.tts_action_name,
                self.naoqi_speech_topic,
                self.fallback_to_speech_topic,
                self.also_publish_speech_topic,
            )
        )

    def goal_callback(self, goal_request: Say.Goal) -> GoalResponse:
        turn_id = self._extract_turn_id(goal_request)
        if self._execution_lock.locked():
            self._trace(turn_id, "SAY_REJECTED", "another goal is running", level="warn")
            return GoalResponse.REJECT

        text = goal_request.input.strip()
        if not text:
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
                "Another say goal is already executing",
                error_code=SkillResult.ROS_ECANCELED,
            )
        try:
            goal = goal_handle.request
            status, message, _duration = await self._run_execution(
                goal_handle=goal_handle,
                text=goal.input.strip(),
                language=self.default_language,
                volume=self.default_volume,
                feedback_publisher=self._publish_feedback,
                turn_id=turn_id,
            )
            if status == "canceled":
                goal_handle.canceled()
                self._trace(turn_id, "TURN_DONE", "say canceled", level="warn")
                return self._canonical_result(
                    False, message, error_code=SkillResult.ROS_ECANCELED
                )
            if status == "aborted":
                goal_handle.abort()
                self._trace(turn_id, "TURN_DONE", "say aborted", level="error")
                return self._canonical_result(
                    False, message, error_code=SkillResult.ROS_EOTHER
                )
            goal_handle.succeed()
            self._trace(turn_id, "TURN_DONE", "say succeeded")
            return self._canonical_result(
                True, message, error_code=SkillResult.ROS_ENOERR
            )
        finally:
            self._execution_lock.release()

    async def _run_execution(
        self,
        goal_handle,
        text: str,
        language: str,
        volume: float,
        feedback_publisher,
        turn_id: str,
    ) -> tuple[str, str, float]:
        start_time = time.monotonic()
        if not text:
            return ("aborted", "Goal text is empty", 0.0)
        if volume < 0.0 or volume > 1.0:
            return ("aborted", "Goal volume is invalid", 0.0)

        self._trace(
            turn_id,
            "SAY_START",
            "text_len=%d lang=%s volume=%.2f" % (len(text), language, volume),
        )
        feedback_publisher(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            return ("canceled", "Cancelled before execution", 0.0)

        if self.also_publish_speech_topic:
            self._publish_speech_topic(text, turn_id=turn_id)

        tts_status, message = await self._forward_tts_goal(
            goal_handle=goal_handle,
            text=text,
            language=language,
            volume=volume,
            feedback_publisher=feedback_publisher,
            turn_id=turn_id,
        )
        duration = time.monotonic() - start_time
        if tts_status == GoalStatus.STATUS_CANCELED:
            return ("canceled", message, duration)
        if tts_status != GoalStatus.STATUS_SUCCEEDED:
            if self.fallback_to_speech_topic:
                if not self.also_publish_speech_topic:
                    self._publish_speech_topic(text, turn_id=turn_id)
                self._trace(
                    turn_id,
                    "SAY_TTS_FALLBACK",
                    "TTS unavailable/failed; used speech topic fallback",
                    level="warn",
                )
                feedback_publisher(goal_handle, "completing", 1.0)
                return (
                    "succeeded",
                    f"{message}; delivered via speech topic fallback",
                    duration,
                )

            return ("aborted", message, duration)

        feedback_publisher(goal_handle, "completing", 1.0)
        return ("succeeded", "Spoken via TTS action server", duration)

    async def _forward_tts_goal(
        self,
        goal_handle,
        text: str,
        language: str,
        volume: float,
        feedback_publisher,
        turn_id: str,
    ) -> tuple[int, str]:
        if not self._tts_client.wait_for_server(timeout_sec=self.tts_server_wait_sec):
            return (
                GoalStatus.STATUS_ABORTED,
                f'TTS action "{self.tts_action_name}" is unavailable',
            )

        if goal_handle.is_cancel_requested:
            return (GoalStatus.STATUS_CANCELED, "Cancelled before TTS dispatch")

        feedback_publisher(goal_handle, "speaking", 0.4)
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

    def _publish_speech_topic(self, text: str, turn_id: str = "") -> None:
        msg = String()
        msg.data = text
        self._speech_publisher.publish(msg)
        self._trace(
            turn_id,
            "SPEECH_PUBLISHED",
            'topic="%s" text_len=%d' % (self.naoqi_speech_topic, len(text)),
        )

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SaySkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
