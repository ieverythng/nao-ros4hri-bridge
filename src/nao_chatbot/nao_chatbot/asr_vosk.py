import queue
import threading
import time
from typing import List

import rclpy
from hri_msgs.msg import LiveSpeech
from rclpy.node import Node
from std_msgs.msg import String

from nao_chatbot.asr_utils import normalize_text, parse_vosk_result, set_text


class AsrVosk(Node):
    """Microphone ASR using Vosk and publishing LiveSpeech."""

    def __init__(self) -> None:
        super().__init__("asr_vosk")

        self.declare_parameter("enabled", False)
        self.declare_parameter(
            "output_speech_topic", "/humans/voices/anonymous_speaker/speech"
        )
        self.declare_parameter("vosk_model_path", "")
        self.declare_parameter("sample_rate_hz", 16000)
        self.declare_parameter("block_duration_ms", 250)
        self.declare_parameter("device_index", -1)
        self.declare_parameter("min_chars", 2)
        self.declare_parameter("min_words", 1)
        self.declare_parameter("dedupe_window_sec", 0.8)
        self.declare_parameter("publish_partial", False)
        self.declare_parameter("allow_sample_rate_fallback", True)
        self.declare_parameter("suppress_during_robot_speech", True)
        self.declare_parameter("robot_speech_topic", "/speech")
        self.declare_parameter("speech_guard_sec_per_word", 0.33)
        self.declare_parameter("speech_guard_min_sec", 1.0)
        self.declare_parameter("speech_guard_extra_sec", 0.5)
        self.declare_parameter("status_warn_period_sec", 2.0)
        self.declare_parameter("ignore_single_token_fillers", True)
        self.declare_parameter(
            "filler_tokens", ["uh", "um", "huh", "hmm", "erm", "ah"]
        )

        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.output_speech_topic = self.get_parameter("output_speech_topic").value
        self.vosk_model_path = self.get_parameter("vosk_model_path").value
        self.sample_rate_hz = int(self.get_parameter("sample_rate_hz").value)
        self.block_duration_ms = int(self.get_parameter("block_duration_ms").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.min_chars = int(self.get_parameter("min_chars").value)
        self.min_words = int(self.get_parameter("min_words").value)
        self.dedupe_window_sec = float(self.get_parameter("dedupe_window_sec").value)
        self.publish_partial = self._as_bool(
            self.get_parameter("publish_partial").value
        )
        self.allow_sample_rate_fallback = self._as_bool(
            self.get_parameter("allow_sample_rate_fallback").value
        )
        self.suppress_during_robot_speech = self._as_bool(
            self.get_parameter("suppress_during_robot_speech").value
        )
        self.robot_speech_topic = self.get_parameter("robot_speech_topic").value
        self.speech_guard_sec_per_word = float(
            self.get_parameter("speech_guard_sec_per_word").value
        )
        self.speech_guard_min_sec = float(
            self.get_parameter("speech_guard_min_sec").value
        )
        self.speech_guard_extra_sec = float(
            self.get_parameter("speech_guard_extra_sec").value
        )
        self.status_warn_period_sec = float(
            self.get_parameter("status_warn_period_sec").value
        )
        self.ignore_single_token_fillers = self._as_bool(
            self.get_parameter("ignore_single_token_fillers").value
        )
        self.filler_tokens = {
            token.strip().lower()
            for token in self.get_parameter("filler_tokens").value
            if isinstance(token, str) and token.strip()
        }

        self.speech_publisher = self.create_publisher(
            LiveSpeech, self.output_speech_topic, 10
        )

        self._audio_queue = queue.Queue(maxsize=64)
        self._stop_event = threading.Event()
        self._worker_thread = None
        self._stream = None
        self._recognizer = None
        self._active_sample_rate_hz = 0
        self._last_text = ""
        self._last_text_ts = 0.0
        self._suppress_until = 0.0
        self._last_status_warn_ts = 0.0
        self._last_queue_full_warn_ts = 0.0
        if self.suppress_during_robot_speech:
            self.create_subscription(
                String, self.robot_speech_topic, self._on_robot_speech, 10
            )

        if not self.enabled:
            self.get_logger().warn("asr_vosk disabled (set asr_vosk_enabled:=true)")
            return
        self._start_asr()

    def _start_asr(self) -> None:
        if not self.vosk_model_path:
            self.get_logger().error("Missing vosk_model_path; asr_vosk cannot start")
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
        except Exception as err:
            self.get_logger().error(f"Could not initialize Vosk model: {err}")
            return

        device = None if self.device_index < 0 else self.device_index
        input_channels = self._resolve_input_channels(sd, device)
        if input_channels <= 0:
            self.get_logger().error(
                "Selected device has no input channels; choose a valid input device"
            )
            return

        candidate_rates = self._build_candidate_rates(sd, device)
        last_error = None

        for candidate_rate in candidate_rates:
            try:
                recognizer = KaldiRecognizer(model, candidate_rate)
                blocksize = max(
                    1,
                    int(candidate_rate * (self.block_duration_ms / 1000.0)),
                )
                stream = sd.RawInputStream(
                    samplerate=candidate_rate,
                    blocksize=blocksize,
                    device=device,
                    dtype="int16",
                    channels=1,
                    callback=self._on_audio_chunk,
                )
                stream.start()
                self._recognizer = recognizer
                self._stream = stream
                self._active_sample_rate_hz = int(candidate_rate)
                if int(candidate_rate) != int(self.sample_rate_hz):
                    self.get_logger().warn(
                        "Requested sample_rate_hz=%d not supported by device %s; "
                        "using %d Hz"
                        % (
                            self.sample_rate_hz,
                            str(self.device_index),
                            int(candidate_rate),
                        )
                    )
                break
            except Exception as err:
                last_error = err

        if self._stream is None:
            self.get_logger().error(
                "Could not open microphone input stream with candidate sample rates %s: %s"
                % (candidate_rates, str(last_error))
            )
            return

        self._worker_thread = threading.Thread(
            target=self._recognition_loop,
            name="asr_vosk_worker",
            daemon=True,
        )
        self._worker_thread.start()
        self.get_logger().info(
            "asr_vosk ready | out:%s sample_rate:%s block_ms:%s device:%s"
            % (
                self.output_speech_topic,
                self._active_sample_rate_hz,
                self.block_duration_ms,
                self.device_index,
            )
        )

    def _build_candidate_rates(self, sd, device) -> List[float]:
        candidates: List[float] = []
        requested = float(self.sample_rate_hz)
        if requested > 0:
            candidates.append(requested)

        if self.allow_sample_rate_fallback:
            default_rate = self._resolve_default_sample_rate(sd, device)
            if default_rate > 0 and default_rate not in candidates:
                candidates.append(default_rate)

            for common_rate in (16000.0, 48000.0, 44100.0, 32000.0):
                if common_rate not in candidates:
                    candidates.append(common_rate)

        return candidates

    def _resolve_default_sample_rate(self, sd, device) -> float:
        try:
            info = sd.query_devices(device, kind="input")
            rate = float(info.get("default_samplerate", 0.0))
            if rate > 0:
                return rate
        except Exception:
            pass
        return 0.0

    def _resolve_input_channels(self, sd, device) -> int:
        try:
            info = sd.query_devices(device, kind="input")
            return int(info.get("max_input_channels", 0))
        except Exception as err:
            self.get_logger().error(f"Could not query input device: {err}")
            return 0

    def _on_audio_chunk(self, indata, frames, time_info, status) -> None:
        del frames
        del time_info
        if status:
            self._warn_stream_status(status)
        if self._is_suppressed():
            return
        try:
            self._audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            now = time.monotonic()
            if now - self._last_queue_full_warn_ts >= self.status_warn_period_sec:
                self._last_queue_full_warn_ts = now
                self.get_logger().warn(
                    "Dropped audio frame because ASR queue is full"
                )

    def _recognition_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._is_suppressed():
                self._clear_audio_queue()
                time.sleep(0.05)
                continue
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
        if self._is_suppressed():
            return
        clean = normalize_text(text)
        if len(clean) < self.min_chars:
            return
        token_count = len(clean.split())
        if token_count < self.min_words:
            return
        if (
            self.ignore_single_token_fillers
            and token_count == 1
            and clean.lower() in self.filler_tokens
        ):
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

    def _on_robot_speech(self, msg: String) -> None:
        text = normalize_text(msg.data)
        words = len(text.split()) if text else 0
        guard_window = max(
            self.speech_guard_min_sec,
            (words * self.speech_guard_sec_per_word) + self.speech_guard_extra_sec,
        )
        now = time.monotonic()
        self._suppress_until = max(self._suppress_until, now + guard_window)
        self._clear_audio_queue()
        if self._recognizer is not None:
            try:
                self._recognizer.Reset()
            except Exception:
                pass

    def _warn_stream_status(self, status) -> None:
        now = time.monotonic()
        if now - self._last_status_warn_ts < self.status_warn_period_sec:
            return
        self._last_status_warn_ts = now
        self.get_logger().warn(f"Microphone stream status: {status}")

    def _is_suppressed(self) -> bool:
        if not self.suppress_during_robot_speech:
            return False
        return time.monotonic() < self._suppress_until

    def _clear_audio_queue(self) -> None:
        while True:
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

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
    node = AsrVosk()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
