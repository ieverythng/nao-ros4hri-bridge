import queue
import threading
import time

import rclpy
from hri_msgs.msg import LiveSpeech
from rclpy.node import Node

from nao_chatbot.asr_utils import normalize_text, parse_vosk_result, set_text


class LaptopAsr(Node):
    """Microphone ASR using Vosk and publishing LiveSpeech."""

    def __init__(self) -> None:
        super().__init__("laptop_asr")

        self.declare_parameter("enabled", False)
        self.declare_parameter(
            "output_speech_topic", "/humans/voices/anonymous_speaker/speech"
        )
        self.declare_parameter("vosk_model_path", "")
        self.declare_parameter("sample_rate_hz", 16000)
        self.declare_parameter("block_duration_ms", 250)
        self.declare_parameter("device_index", -1)
        self.declare_parameter("min_chars", 2)
        self.declare_parameter("dedupe_window_sec", 0.8)
        self.declare_parameter("publish_partial", False)

        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.output_speech_topic = self.get_parameter("output_speech_topic").value
        self.vosk_model_path = self.get_parameter("vosk_model_path").value
        self.sample_rate_hz = int(self.get_parameter("sample_rate_hz").value)
        self.block_duration_ms = int(self.get_parameter("block_duration_ms").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.min_chars = int(self.get_parameter("min_chars").value)
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self.publish_partial = self._as_bool(
            self.get_parameter("publish_partial").value
        )

        self.speech_publisher = self.create_publisher(
            LiveSpeech, self.output_speech_topic, 10
        )

        self._audio_queue = queue.Queue(maxsize=64)
        self._stop_event = threading.Event()
        self._worker_thread = None
        self._stream = None
        self._recognizer = None
        self._last_text = ""
        self._last_text_ts = 0.0

        if not self.enabled:
            self.get_logger().warn("laptop_asr disabled (set laptop_asr_enabled:=true)")
            return
        self._start_asr()

    def _start_asr(self) -> None:
        if not self.vosk_model_path:
            self.get_logger().error("Missing vosk_model_path; laptop_asr cannot start")
            return

        try:
            import sounddevice as sd
        except ImportError as err:
            self.get_logger().error(f"sounddevice import failed: {err}")
            return
        try:
            from vosk import KaldiRecognizer
            from vosk import Model
        except ImportError as err:
            self.get_logger().error(f"vosk import failed: {err}")
            return

        try:
            model = Model(self.vosk_model_path)
            self._recognizer = KaldiRecognizer(model, self.sample_rate_hz)
        except Exception as err:
            self.get_logger().error(f"Could not initialize Vosk model: {err}")
            return

        blocksize = max(
            1,
            int(self.sample_rate_hz * (self.block_duration_ms / 1000.0)),
        )

        try:
            self._stream = sd.RawInputStream(
                samplerate=self.sample_rate_hz,
                blocksize=blocksize,
                device=None if self.device_index < 0 else self.device_index,
                dtype="int16",
                channels=1,
                callback=self._on_audio_chunk,
            )
            self._stream.start()
        except Exception as err:
            self.get_logger().error(f"Could not open microphone input stream: {err}")
            self._stream = None
            return

        self._worker_thread = threading.Thread(
            target=self._recognition_loop,
            name="laptop_asr_worker",
            daemon=True,
        )
        self._worker_thread.start()
        self.get_logger().info(
            "laptop_asr ready | out:%s sample_rate:%s block_ms:%s device:%s"
            % (
                self.output_speech_topic,
                self.sample_rate_hz,
                self.block_duration_ms,
                self.device_index,
            )
        )

    def _on_audio_chunk(self, indata, frames, time_info, status) -> None:
        del frames
        del time_info
        if status:
            self.get_logger().warn(f"Microphone stream status: {status}")
        try:
            self._audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            self.get_logger().warn("Dropped audio frame because ASR queue is full")

    def _recognition_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                audio_chunk = self._audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if not self._recognizer:
                continue

            if self._recognizer.AcceptWaveform(audio_chunk):
                result_text = parse_vosk_result(self._recognizer.Result())
                self._publish_text(result_text, partial=False)
            elif self.publish_partial:
                partial_text = parse_vosk_result(self._recognizer.PartialResult())
                self._publish_text(partial_text, partial=True)

    def _publish_text(self, text: str, partial: bool) -> None:
        clean = normalize_text(text)
        if len(clean) < self.min_chars:
            return

        now = time.monotonic()
        if (
            clean == self._last_text
            and now - self._last_text_ts <= self.dedupe_window_sec
        ):
            return
        self._last_text = clean
        self._last_text_ts = now

        out_msg = LiveSpeech()
        if not set_text(out_msg, clean, partial=partial):
            self.get_logger().error("Could not map transcript into LiveSpeech fields")
            return

        self.speech_publisher.publish(out_msg)
        if partial:
            self.get_logger().debug(f"Published ASR partial: {clean}")
        else:
            self.get_logger().info(f"Published ASR final: {clean}")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
        return super().destroy_node()

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaptopAsr()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
