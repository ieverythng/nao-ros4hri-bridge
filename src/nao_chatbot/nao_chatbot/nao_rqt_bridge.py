import time

import rclpy
from hri_msgs.msg import LiveSpeech
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tts_msgs.action import TTS


class NaoRqtBridge(Node):
    """Bridge between ROS4HRI chat topics and robot speech outputs."""

    def __init__(self) -> None:
        super().__init__("nao_rqt_bridge")

        self.declare_parameter(
            "input_speech_topic", "/humans/voices/anonymous_speaker/speech"
        )
        self.declare_parameter("user_text_topic", "/chatbot/user_text")
        self.declare_parameter("assistant_text_topic", "/chatbot/assistant_text")
        self.declare_parameter("naoqi_speech_topic", "/speech")
        self.declare_parameter("use_tts_action", True)
        self.declare_parameter("also_publish_speech_topic", True)
        self.declare_parameter("tts_action_name", "/tts_engine/tts")
        self.declare_parameter("reply_prefix", "")
        self.declare_parameter("dedupe_window_sec", 0.8)

        self.input_speech_topic = self.get_parameter("input_speech_topic").value
        self.user_text_topic = self.get_parameter("user_text_topic").value
        self.assistant_text_topic = self.get_parameter("assistant_text_topic").value
        self.naoqi_speech_topic = self.get_parameter("naoqi_speech_topic").value
        self.use_tts_action = self.get_parameter("use_tts_action").value
        self.also_publish_speech_topic = self.get_parameter(
            "also_publish_speech_topic"
        ).value
        self.tts_action_name = self.get_parameter("tts_action_name").value
        self.reply_prefix = self.get_parameter("reply_prefix").value
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self._last_user_text = ""
        self._last_user_text_ts = 0.0
        self._last_assistant_text = ""
        self._last_assistant_text_ts = 0.0

        self.user_text_publisher = self.create_publisher(String, self.user_text_topic, 10)
        self.naoqi_speech_publisher = self.create_publisher(
            String, self.naoqi_speech_topic, 10
        )
        self.tts_client = ActionClient(self, TTS, self.tts_action_name)

        self.create_subscription(
            LiveSpeech, self.input_speech_topic, self._on_live_speech, 10
        )
        self.create_subscription(
            String, self.assistant_text_topic, self._on_assistant_text, 10
        )

        self.get_logger().info(
            "nao_rqt_bridge ready | in:%s user:%s assistant:%s tts:%s speech:%s"
            % (
                self.input_speech_topic,
                self.user_text_topic,
                self.assistant_text_topic,
                self.tts_action_name,
                self.naoqi_speech_topic,
            )
        )

    def _on_live_speech(self, msg: LiveSpeech) -> None:
        text = self._extract_text(msg)
        if not text:
            self.get_logger().warn("LiveSpeech received without usable text")
            return
        now = time.monotonic()
        if (
            text == self._last_user_text
            and now - self._last_user_text_ts <= self.dedupe_window_sec
        ):
            self.get_logger().warn(f'Ignored duplicate user text within {self.dedupe_window_sec}s')
            return
        self._last_user_text = text
        self._last_user_text_ts = now

        user_msg = String()
        user_msg.data = text
        self.user_text_publisher.publish(user_msg)
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
        self._send_robot_output(text)

    def _send_robot_output(self, text: str) -> None:
        if self.use_tts_action and self.tts_client.wait_for_server(timeout_sec=0.2):
            goal = TTS.Goal()
            goal.input = text
            self.tts_client.send_goal_async(goal)
            self.get_logger().info(f'Sent TTS action goal "{self.tts_action_name}": {text}')
        elif self.use_tts_action:
            self.get_logger().warn(f'TTS action "{self.tts_action_name}" not available')

        if self.also_publish_speech_topic:
            speech_msg = String()
            speech_msg.data = text
            self.naoqi_speech_publisher.publish(speech_msg)
            self.get_logger().info(
                f'Published robot speech on "{self.naoqi_speech_topic}": {text}'
            )

    @staticmethod
    def _extract_text(msg: LiveSpeech) -> str:
        for field_name in ("final", "incremental", "text", "transcript", "utterance"):
            value = getattr(msg, field_name, "")
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaoRqtBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
