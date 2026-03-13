from __future__ import annotations

import time

from hri_actions_msgs.msg import ClosedCaption
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotSpeechDebugNode(Node):
    """Mirror final robot utterances into ROS logs for operator consoles."""

    def __init__(self) -> None:
        super().__init__("robot_speech_debug")

        self.declare_parameter("debug_speech_topic", "/debug/nao_say/speech")
        self.declare_parameter(
            "closed_captions_topic",
            "/dialogue_manager/closed_captions",
        )
        self.declare_parameter("dedupe_window_sec", 0.5)

        self._last_text = ""
        self._last_timestamp = 0.0
        self._dedupe_window_sec = max(
            0.0,
            float(self.get_parameter("dedupe_window_sec").value),
        )

        debug_speech_topic = str(self.get_parameter("debug_speech_topic").value)
        closed_captions_topic = str(self.get_parameter("closed_captions_topic").value)

        self.create_subscription(
            String,
            debug_speech_topic,
            self._on_debug_speech,
            10,
        )
        self.create_subscription(
            ClosedCaption,
            closed_captions_topic,
            self._on_closed_caption,
            10,
        )
        self.get_logger().info(
            "robot_speech_debug listening | debug:%s captions:%s"
            % (debug_speech_topic, closed_captions_topic)
        )

    def _on_debug_speech(self, msg: String) -> None:
        self._log_robot_text(msg.data, source="debug_speech")

    def _on_closed_caption(self, msg: ClosedCaption) -> None:
        self._log_robot_text(msg.text, source="closed_caption")

    def _log_robot_text(self, text: str, source: str) -> None:
        clean_text = str(text).strip()
        if not clean_text:
            return

        now = time.monotonic()
        if (
            clean_text == self._last_text
            and (now - self._last_timestamp) <= self._dedupe_window_sec
        ):
            return

        self._last_text = clean_text
        self._last_timestamp = now
        self.get_logger().info(f'[ROBOT OUTPUT] ({source}) "{clean_text}"')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotSpeechDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
