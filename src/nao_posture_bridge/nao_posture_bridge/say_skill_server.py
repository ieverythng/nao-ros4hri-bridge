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
        if self._execution_lock.locked():
            self.get_logger().warn("Rejected say goal because another goal is running")
            return GoalResponse.REJECT

        text = goal_request.input.strip()
        if not text:
            self.get_logger().warn("Rejected say goal with empty text")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for say goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
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
            )
            if status == "canceled":
                goal_handle.canceled()
                return self._canonical_result(
                    False, message, error_code=SkillResult.ROS_ECANCELED
                )
            if status == "aborted":
                goal_handle.abort()
                return self._canonical_result(
                    False, message, error_code=SkillResult.ROS_EOTHER
                )
            goal_handle.succeed()
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
    ) -> tuple[str, str, float]:
        start_time = time.monotonic()
        if not text:
            return ("aborted", "Goal text is empty", 0.0)
        if volume < 0.0 or volume > 1.0:
            return ("aborted", "Goal volume is invalid", 0.0)

        feedback_publisher(goal_handle, "preparing", 0.0)
        if goal_handle.is_cancel_requested:
            return ("canceled", "Cancelled before execution", 0.0)

        if self.also_publish_speech_topic:
            self._publish_speech_topic(text)

        tts_status, message = await self._forward_tts_goal(
            goal_handle=goal_handle,
            text=text,
            language=language,
            volume=volume,
            feedback_publisher=feedback_publisher,
        )
        duration = time.monotonic() - start_time
        if tts_status == GoalStatus.STATUS_CANCELED:
            return ("canceled", message, duration)
        if tts_status != GoalStatus.STATUS_SUCCEEDED:
            if self.fallback_to_speech_topic:
                if not self.also_publish_speech_topic:
                    self._publish_speech_topic(text)
                self.get_logger().warn(
                    "TTS action unavailable/failed. Completed with speech-topic fallback."
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

        self.get_logger().info(
            "Forwarding say request to TTS action | lang:%s volume:%.2f text:%s"
            % (language, volume, text)
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

    def _publish_speech_topic(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._speech_publisher.publish(msg)
        self.get_logger().info(
            'Published speech fallback to "%s": %s'
            % (self.naoqi_speech_topic, text)
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
