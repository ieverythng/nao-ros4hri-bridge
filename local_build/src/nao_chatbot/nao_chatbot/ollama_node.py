import rclpy
from hri_msgs.msg import LiveSpeech
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tts_msgs.action import TTS


class OllamaChatbot(Node):
    """Step-1 bridge: ROS4HRI chat input -> NAO speech output."""

    def __init__(self):
        super().__init__("ollama_chatbot")

        self.declare_parameter(
            "input_topic", "/humans/voices/anonymous_speaker/speech"
        )
        self.declare_parameter("output_topic", "/speech")
        self.declare_parameter("use_tts_action", True)
        self.declare_parameter("also_publish_speech_topic", True)
        self.declare_parameter("tts_action_name", "/tts_engine/tts")
        self.declare_parameter("reply_prefix", "")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.use_tts_action = self.get_parameter("use_tts_action").value
        self.also_publish_speech_topic = self.get_parameter(
            "also_publish_speech_topic"
        ).value
        self.tts_action_name = self.get_parameter("tts_action_name").value
        self.reply_prefix = self.get_parameter("reply_prefix").value

        self.subscription = self.create_subscription(
            LiveSpeech,
            self.input_topic,
            self.speech_callback,
            10,
        )
        self.speech_publisher = self.create_publisher(String, self.output_topic, 10)
        self.tts_client = ActionClient(self, TTS, self.tts_action_name)

        self.get_logger().info(
            "nao_chatbot bridge ready | input: %s | tts_action: %s | speech_topic: %s"
            % (self.input_topic, self.tts_action_name, self.output_topic)
        )

    def speech_callback(self, msg: LiveSpeech) -> None:
        # Support different LiveSpeech field names across package versions.
        text = self._extract_text(msg)
        if not text:
            self.get_logger().warn("Received LiveSpeech without usable text field")
            return

        reply = self._build_reply(text)
        output_text = f"{self.reply_prefix}{reply}"
        self._speak(output_text)
        self.get_logger().info(f'Received user text: "{text}"')

    def _speak(self, text: str) -> None:
        sent_to_tts_action = False
        if self.use_tts_action and self.tts_client.wait_for_server(timeout_sec=0.2):
            goal = TTS.Goal()
            goal.input = text
            self.tts_client.send_goal_async(goal)
            sent_to_tts_action = True
            self.get_logger().info(
                f'Sent TTS action goal to "{self.tts_action_name}": {text}'
            )
        elif self.use_tts_action:
            self.get_logger().warn(
                f'TTS action "{self.tts_action_name}" not available'
            )

        if self.also_publish_speech_topic:
            msg = String()
            msg.data = text
            self.speech_publisher.publish(msg)
            self.get_logger().info(f'Published speech on "{self.output_topic}": {text}')
            return

        if not sent_to_tts_action:
            self.get_logger().warn("Reply dropped: no TTS action and speech publishing disabled")

    @staticmethod
    def _build_reply(user_text: str) -> str:
        lowered = user_text.lower()
        if any(token in lowered for token in ("hello", "hi", "hey")):
            return "Hello! Nice to meet you."
        if "how are you" in lowered:
            return "I am doing well. Thank you for asking."
        if "your name" in lowered or "who are you" in lowered:
            return "I am the Nao chatbot bridge."
        if "help" in lowered:
            return "You can greet me, ask my name, or ask how I am."
        if "slavic" in lowered:
            return "So you are talking about the hottest girl in the world?"    
        return "I heard you. We are testing the chat to speech pipeline."

    @staticmethod
    def _extract_text(msg: LiveSpeech) -> str:
        for field_name in ("final", "incremental", "text", "transcript", "utterance"):
            value = getattr(msg, field_name, "")
            if isinstance(value, str) and value.strip():
                return value.strip()
        return ""

def main(args=None):
    rclpy.init(args=args)
    node = OllamaChatbot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()