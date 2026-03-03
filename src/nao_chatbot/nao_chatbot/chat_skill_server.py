#!/usr/bin/env python3
"""Action server for `/skill/chat` backed by Ollama."""

from __future__ import annotations

import threading

from communication_skills.action import Chat
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_skills.msg import Result as SkillResult

from nao_chatbot.chat_config import ChatSkillConfig
from nao_chatbot.chat_config import declare_chat_parameters
from nao_chatbot.chat_config import load_chat_config
from nao_chatbot.chat_goal_codec import canonical_result
from nao_chatbot.chat_goal_codec import extract_canonical_goal
from nao_chatbot.chat_turn_engine import ChatTurnEngine
from nao_chatbot.ollama_transport import OllamaTransport
from nao_chatbot.skill_catalog import build_skill_catalog_text


class ChatSkillServer(Node):
    """Serve Ollama-backed conversational turns over `/skill/chat`."""

    def __init__(self) -> None:
        super().__init__("ollama_chatbot")

        declare_chat_parameters(self)
        self._config: ChatSkillConfig = load_chat_config(self)

        self._execution_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._skill_catalog_text = ""
        if self._config.use_skill_catalog and self._config.skill_catalog_packages:
            self._skill_catalog_text, descriptors = build_skill_catalog_text(
                package_names=self._config.skill_catalog_packages,
                max_entries=self._config.skill_catalog_max_entries,
                max_chars=self._config.skill_catalog_max_chars,
                logger=self.get_logger(),
            )
            if descriptors:
                self.get_logger().info(
                    "Loaded skill catalog (%d entries) from packages: %s"
                    % (
                        len(descriptors),
                        ", ".join(self._config.skill_catalog_packages),
                    )
                )

        self._transport = OllamaTransport(
            ollama_url=self._config.ollama_url,
            context_window_tokens=self._config.context_window_tokens,
            logger=self.get_logger(),
        )
        self._turn_engine = ChatTurnEngine(
            config=self._config,
            transport=self._transport,
            logger=self.get_logger(),
            skill_catalog_text=self._skill_catalog_text,
        )

        self._action_server = ActionServer(
            self,
            Chat,
            self._config.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "ollama_chatbot ready | action:%s canonical_type:communication_skills/action/Chat "
            "backend_enabled:%s model:%s intent_model:%s mode:%s prompt_pack:%s"
            % (
                self._config.action_name,
                self._config.enabled,
                self._config.model,
                self._config.intent_model,
                self._config.intent_detection_mode,
                self._config.prompt_pack_path or "<defaults>",
            )
        )
        self._transport.log_model_inventory()

    def goal_callback(self, goal_request: Chat.Goal) -> GoalResponse:
        if self._execution_lock.locked():
            self.get_logger().warn("Rejected chat goal because another goal is running")
            return GoalResponse.REJECT

        user_message, _history, _user_id, _turn_id = extract_canonical_goal(goal_request)
        if not user_message:
            self.get_logger().warn("Rejected chat goal with empty user message")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for chat goal")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        if not self._execution_lock.acquire(blocking=False):
            goal_handle.abort()
            return canonical_result(
                False,
                "Another chat goal is already executing",
                [],
                "",
                SkillResult.ROS_ECANCELED,
                intent="fallback",
                intent_source="server_busy",
                intent_confidence=0.0,
                user_intent={},
                turn_id="unknown",
            )

        try:
            user_text, history, user_id, turn_id = extract_canonical_goal(goal_handle.request)
            if not user_text:
                goal_handle.abort()
                return canonical_result(
                    False,
                    "Goal user message is empty",
                    history,
                    "",
                    SkillResult.ROS_EBADMSG,
                    intent="fallback",
                    intent_source="bad_goal",
                    intent_confidence=0.0,
                    user_intent={},
                    turn_id=turn_id,
                )

            turn = self._turn_engine.execute_turn(
                goal_handle=goal_handle,
                user_text=user_text,
                history=history,
                user_id=user_id,
                publish_feedback=self._publish_feedback,
                turn_id=turn_id,
                trace=self._trace,
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return canonical_result(
                    False,
                    "Cancelled while executing chat goal",
                    history,
                    "",
                    SkillResult.ROS_ECANCELED,
                    intent="fallback",
                    intent_source="cancelled",
                    intent_confidence=0.0,
                    user_intent={},
                    turn_id=turn_id,
                )

            goal_handle.succeed()
            error_code = SkillResult.ROS_ENOERR if turn.success else SkillResult.ROS_EOTHER
            message = "" if turn.success else "No assistant response generated"
            return canonical_result(
                turn.success,
                message,
                turn.updated_history,
                turn.verbal_ack,
                error_code,
                intent=turn.intent,
                intent_source=turn.intent_source,
                intent_confidence=turn.intent_confidence,
                user_intent=turn.user_intent,
                turn_id=turn_id,
            )
        finally:
            self._execution_lock.release()

    @staticmethod
    def _publish_feedback(goal_handle, status: str, progress: float) -> None:
        feedback = Chat.Feedback()
        feedback.feedback.data_str = status
        feedback.feedback.data_float = float(progress)
        goal_handle.publish_feedback(feedback)

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
    node = ChatSkillServer()
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
