from audio_common_msgs.msg import AudioData
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hri_msgs.msg import IdsList, LiveSpeech
import json
import re
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.lifecycle import Node, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from vosk import Model, KaldiRecognizer


class NodeVosk(Node):
    """Vosk speech recognition node."""

    def __init__(self):
        super().__init__('asr_vosk')

        self.declare_parameter(
            'audio_rate', 16000, ParameterDescriptor(description='Device sampling rate'))
        self.declare_parameter(
            'model', "vosk_model_small", ParameterDescriptor(description='Model family name'))
        self.declare_parameter(
            'microphone_topic', '/robot/microphone0', ParameterDescriptor(description='Microphone audio input topic'))
        self.declare_parameter(
            'start_listening', True, ParameterDescriptor(description='Start listening'))
        self.declare_parameter(
            'output_speech_topic', '/humans/voices/anonymous_speaker/speech',
            ParameterDescriptor(description='LiveSpeech output topic'))
        self.declare_parameter(
            'speech_locale', 'en_US', ParameterDescriptor(description='Locale used in LiveSpeech messages'))
        self.declare_parameter(
            'publish_partials', True, ParameterDescriptor(description='Publish incremental partial hypotheses.'))
        self.declare_parameter(
            'min_final_chars', 2, ParameterDescriptor(description='Drop final hypotheses shorter than this number of characters.'))
        self.declare_parameter(
            'min_final_words', 1, ParameterDescriptor(description='Drop final hypotheses with fewer words than this value.'))
        self.declare_parameter(
            'min_final_confidence', 0.0, ParameterDescriptor(description='Minimum average word confidence [0-1] for final hypotheses.'))
        self.declare_parameter(
            'ignore_single_token_fillers', True, ParameterDescriptor(description='Drop single-token filler words (eg. uh, um, huh).'))
        self.declare_parameter(
            'single_token_fillers_csv', 'uh,um,hmm,huh,erm,ah,eh',
            ParameterDescriptor(description='Comma-separated filler tokens dropped when ignore_single_token_fillers is true.'))
        self.declare_parameter(
            'debug_log_results', False, ParameterDescriptor(description='Log final hypotheses and filter decisions for debugging.'))

        self.get_logger().info('State: Unconfigured.')

        # Add parameter callback for dynamic topic change
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.audio_data_sub = None
        self.voice_detected_sub = None
        self.robot_speaking_sub = None

    def on_parameter_change(self, params):
        for param in params:
            if param.name == "microphone_topic" and param.type_ == Parameter.Type.STRING:
                new_topic = param.value
                if new_topic != self.microphone_topic:
                    self.get_logger().info(f"Changing microphone topic to: {new_topic}")
                    # Unsubscribe from old topic
                    if self.audio_data_sub:
                        self.destroy_subscription(self.audio_data_sub)
                    # Subscribe to new topic
                    self.audio_data_sub = self.create_subscription(
                        AudioData, new_topic, self.on_audio_data, 10)
                    self.microphone_topic = new_topic
                else:
                    return SetParametersResult(successful=False, reason="Topic is the same as current.")
            else:
                return SetParametersResult(successful=False, reason="Invalid parameter change. Types do not match.")
        return SetParametersResult(successful=True)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.internal_cleanup()
        self.get_logger().info('State: Unconfigured.')
        return super().on_cleanup(state)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.audio_rate = self.get_parameter('audio_rate').value
        self.model = self.get_parameter('model').value
        self.microphone_topic = self.get_parameter('microphone_topic').value
        self.start_listening = self.get_parameter('start_listening').value
        self.output_speech_topic = self.get_parameter('output_speech_topic').value
        self.speech_locale = self.get_parameter('speech_locale').value
        self.publish_partials = self.get_parameter('publish_partials').value
        self.min_final_chars = self.get_parameter('min_final_chars').value
        self.min_final_words = self.get_parameter('min_final_words').value
        self.min_final_confidence = self.get_parameter('min_final_confidence').value
        self.ignore_single_token_fillers = self.get_parameter(
            'ignore_single_token_fillers'
        ).value
        fillers_csv = self.get_parameter('single_token_fillers_csv').value
        self.single_token_fillers = {
            token.strip().lower() for token in str(fillers_csv).split(',') if token.strip()
        }
        self.debug_log_results = self.get_parameter('debug_log_results').value

        loaded_model, _ = self.load_model(self.model)
        if not loaded_model:
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'Using model: {self.model}')
        self.get_logger().info(f'Using microphone topic: {self.microphone_topic}')
        self.get_logger().info(f'Using speech topic: {self.output_speech_topic}')
        self.get_logger().info(f'Using locale: {self.speech_locale}')
        self.get_logger().info(
            'Final filter: min_chars=%d min_words=%d min_conf=%.2f '
            'ignore_single_token_fillers=%s publish_partials=%s'
            % (
                self.min_final_chars,
                self.min_final_words,
                float(self.min_final_confidence),
                str(self.ignore_single_token_fillers),
                str(self.publish_partials),
            )
        )

        self.get_logger().info('State: Inactive.')
        return super().on_configure(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.internal_deactivate()
        self.get_logger().info('State: Inactive.')
        return super().on_deactivate(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.current_incremental = ''
        self.last_final = ''
        self.last_final_confidence = 0.0
        self.last_drop_reason = ''
        self.dropped_final_count = 0
        self.listening = self.start_listening
        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1)
        self.voices_pub = self.create_publisher(
            IdsList, "/humans/voices/tracked", 1)
        self.speech_pub = self.create_publisher(
            LiveSpeech, self.output_speech_topic, 10)
        self.voice_audio_pub = self.create_publisher(
            AudioData, "/humans/voices/anonymous_speaker/audio", 10)
        self.is_speaking_pub = self.create_publisher(
            Bool, "/humans/voices/anonymous_speaker/is_speaking", 10)

        # Subscribe to microphone topic
        self.audio_data_sub = self.create_subscription(
            AudioData, self.microphone_topic, self.on_audio_data, 10)

        self.voice_detected_sub = self.create_subscription(
            Bool, "audio/voice_detected", self.on_voice_detected, 1)
        self.robot_speaking_sub = self.create_subscription(
            Bool, "/robot_speaking", self.on_robot_speaking, 1)
        #self.listening_sub = self.create_subscription(
        #    Bool, "/start_listening", self.on_start_listening, 1)

        self.diag_timer = self.create_timer(1., self.publish_diagnostics)

        # currently the voice is always associated to the same anonymous speaker
        # publish it repeatedly to ensure latecomers get the message (the topic is not latched)
        def publish_anonymous_voice_id():
            self.voices_pub.publish(IdsList(ids=["anonymous_speaker"]))

        publish_anonymous_voice_id()
        self.voices_timer = self.create_timer(
            1., publish_anonymous_voice_id, clock=self.get_clock())

        self.get_logger().info('State: Active.')
        return super().on_activate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if state.state_id == State.PRIMARY_STATE_ACTIVE:
            self.internal_deactivate()
        if state.state_id in [State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE]:
            self.internal_cleanup()
        self.get_logger().info('State: Finalized.')
        return super().on_shutdown(state)

    def internal_cleanup(self):
        del self.recognizer

    def internal_deactivate(self):
        self.destroy_timer(self.diag_timer)
        self.destroy_subscription(self.audio_data_sub)
        if self.voice_detected_sub is not None:
            self.destroy_subscription(self.voice_detected_sub)
            self.voice_detected_sub = None
        if self.robot_speaking_sub is not None:
            self.destroy_subscription(self.robot_speaking_sub)
            self.robot_speaking_sub = None
        self.destroy_publisher(self.diag_pub)
        self.destroy_publisher(self.voices_pub)
        self.destroy_publisher(self.speech_pub)
        self.destroy_publisher(self.voice_audio_pub)
        self.destroy_publisher(self.is_speaking_pub)

    def load_model(self, model):
        try:
            model = Model(str(model))
            self.get_logger().info(f'Loaded {self.model}')
        except Exception as e:  # vosk Model raises generic exceptions :/
            error_msg = f'Failed to load {self.model}: {str(e)}'
            self.get_logger().error(error_msg)
            return False, error_msg

        self.recognizer = KaldiRecognizer(model, self.audio_rate)
        return True, ""

    def on_voice_detected(self, msg):
        self.is_speaking_pub.publish(msg)

    def on_start_listening(self, msg):
        self.listening = msg.data
        #self.start_listening = True

    def on_robot_speaking(self, msg):
        # Stop listening when robot is speaking
        if msg.data:
            self.listening = False
        else:
            # Resume listening when robot is not speaking
            self.listening = True

    def on_audio_data(self, audio_data_msg):

        self.voice_audio_pub.publish(audio_data_msg)
        if self.listening:
            speech_msg = LiveSpeech(locale=self.speech_locale)
            speech_msg.header.stamp = self.get_clock().now().to_msg()

            if self.recognizer.AcceptWaveform(bytes(audio_data_msg.data)):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").strip()
                confidence = self._extract_avg_confidence(result)
                if text:
                    should_publish, drop_reason = self._should_publish_final(
                        text, confidence
                    )
                    if should_publish:
                        speech_msg.incremental = text
                        speech_msg.final = text
                        speech_msg.confidence = confidence
                        self.speech_pub.publish(speech_msg)
                        self.last_final = text
                        self.last_final_confidence = confidence
                        self.last_drop_reason = ''
                    else:
                        self.dropped_final_count += 1
                        self.last_drop_reason = f'{drop_reason}: "{text}"'
                        if self.debug_log_results:
                            self.get_logger().info(
                                'Dropped final hypothesis | reason=%s text="%s" conf=%.2f'
                                % (drop_reason, text, confidence)
                            )
                    '''
                    if not self.start_listening:
                        self.get_logger().info("stop listening")
                        self.listening = False
                    '''
                elif self.debug_log_results:
                    self.get_logger().debug('Ignoring empty final hypothesis.')

                self.current_incremental = ''
            else:
                result = json.loads(self.recognizer.PartialResult())
                partial = result.get("partial", "").strip()
                if (
                    self.publish_partials
                    and partial
                    and (partial != self.current_incremental)
                ):
                    speech_msg.incremental = partial
                    self.speech_pub.publish(speech_msg)

                self.current_incremental = partial

    def _extract_avg_confidence(self, result):
        """Compute average word confidence from a Vosk final result JSON."""
        words = result.get('result', [])
        confidences = []
        for word in words:
            conf = word.get('conf')
            if isinstance(conf, (int, float)):
                confidences.append(float(conf))
        if not confidences:
            return 0.0
        return sum(confidences) / len(confidences)

    def _should_publish_final(self, text, confidence):
        """Apply lightweight filtering to avoid low-quality/filler final hypotheses."""
        normalized_text = text.strip()
        if len(normalized_text) < int(self.min_final_chars):
            return False, 'min_final_chars'

        tokens = [t for t in re.split(r'\s+', normalized_text) if t]
        if len(tokens) < int(self.min_final_words):
            return False, 'min_final_words'

        if float(confidence) < float(self.min_final_confidence):
            return False, 'min_final_confidence'

        if (
            self.ignore_single_token_fillers
            and len(tokens) == 1
            and tokens[0].lower() in self.single_token_fillers
        ):
            return False, 'single_token_filler'

        return True, ''

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/communication/asr/asr_vosk",
            message="vosk ASR running",
            values=[
                KeyValue(key="Module name", value="asr_vosk"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="Model", value=self.model),
                KeyValue(key="Currently listening", value=str(self.listening)),
                KeyValue(key="Last recognised sentence",
                         value=self.last_final),
                KeyValue(key="Last recognised confidence", value=f'{self.last_final_confidence:.2f}'),
                KeyValue(key="Dropped final hypotheses", value=str(self.dropped_final_count)),
                KeyValue(key="Last drop reason", value=self.last_drop_reason),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = NodeVosk()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()


if __name__ == '__main__':
    main()
