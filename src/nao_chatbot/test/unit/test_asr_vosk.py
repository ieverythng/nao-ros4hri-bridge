import importlib
import queue
import sys
import time
import types


class _FakeLogger:
    def __init__(self) -> None:
        self.errors = []

    def error(self, msg: str) -> None:
        self.errors.append(msg)


class _FakeSoundDeviceOk:
    @staticmethod
    def query_devices(device=None, kind="input"):
        del device
        del kind
        return {"default_samplerate": 48000.0, "max_input_channels": 2}


class _FakeSoundDeviceNoDefault:
    @staticmethod
    def query_devices(device=None, kind="input"):
        del device
        del kind
        return {"default_samplerate": 0.0, "max_input_channels": 1}


class _FakeSoundDeviceRaises:
    @staticmethod
    def query_devices(device=None, kind="input"):
        del device
        del kind
        raise RuntimeError("device query failed")


def _import_asr_vosk_module():
    """Import nao_chatbot.asr_vosk with lightweight stubs for ROS deps."""
    if "rclpy" not in sys.modules:
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy_node = types.ModuleType("rclpy.node")

        class _FakeNode:
            pass

        fake_rclpy_node.Node = _FakeNode
        fake_rclpy.node = fake_rclpy_node
        sys.modules["rclpy"] = fake_rclpy
        sys.modules["rclpy.node"] = fake_rclpy_node

    if "hri_msgs" not in sys.modules:
        fake_hri_msgs = types.ModuleType("hri_msgs")
        fake_hri_msgs_msg = types.ModuleType("hri_msgs.msg")

        class _FakeLiveSpeech:
            pass

        fake_hri_msgs_msg.LiveSpeech = _FakeLiveSpeech
        fake_hri_msgs.msg = fake_hri_msgs_msg
        sys.modules["hri_msgs"] = fake_hri_msgs
        sys.modules["hri_msgs.msg"] = fake_hri_msgs_msg

    if "std_msgs" not in sys.modules:
        fake_std_msgs = types.ModuleType("std_msgs")
        fake_std_msgs_msg = types.ModuleType("std_msgs.msg")

        class _FakeString:
            def __init__(self):
                self.data = ""

        fake_std_msgs_msg.String = _FakeString
        fake_std_msgs.msg = fake_std_msgs_msg
        sys.modules["std_msgs"] = fake_std_msgs
        sys.modules["std_msgs.msg"] = fake_std_msgs_msg

    return importlib.import_module("nao_chatbot.asr_vosk")


def test_build_candidate_rates_with_fallback() -> None:
    asr_vosk = _import_asr_vosk_module()
    node = object.__new__(asr_vosk.AsrVosk)
    node.sample_rate_hz = 16000
    node.allow_sample_rate_fallback = True

    rates = node._build_candidate_rates(_FakeSoundDeviceOk(), device=3)

    assert rates[0] == 16000.0
    assert 48000.0 in rates
    assert 44100.0 in rates
    assert len(rates) == len(set(rates))


def test_build_candidate_rates_without_fallback() -> None:
    asr_vosk = _import_asr_vosk_module()
    node = object.__new__(asr_vosk.AsrVosk)
    node.sample_rate_hz = 16000
    node.allow_sample_rate_fallback = False

    rates = node._build_candidate_rates(_FakeSoundDeviceOk(), device=3)

    assert rates == [16000.0]


def test_resolve_default_sample_rate_handles_missing_default() -> None:
    asr_vosk = _import_asr_vosk_module()
    node = object.__new__(asr_vosk.AsrVosk)

    assert node._resolve_default_sample_rate(_FakeSoundDeviceNoDefault(), device=3) == 0.0


def test_resolve_input_channels_handles_exception() -> None:
    asr_vosk = _import_asr_vosk_module()
    node = object.__new__(asr_vosk.AsrVosk)
    logger = _FakeLogger()
    node.get_logger = lambda: logger

    channels = node._resolve_input_channels(_FakeSoundDeviceRaises(), device=3)

    assert channels == 0
    assert any("Could not query input device" in msg for msg in logger.errors)


def test_robot_speech_guard_suppresses_and_clears_queue() -> None:
    asr_vosk = _import_asr_vosk_module()
    node = object.__new__(asr_vosk.AsrVosk)
    node.suppress_during_robot_speech = True
    node.speech_guard_min_sec = 1.0
    node.speech_guard_sec_per_word = 0.2
    node.speech_guard_extra_sec = 0.3
    node._suppress_until = 0.0
    node._audio_queue = queue.Queue()
    node._audio_queue.put_nowait(b"x")
    node._recognizer = None

    msg = types.SimpleNamespace(data="please sit down now")
    started = time.monotonic()
    node._on_robot_speech(msg)

    assert node._suppress_until >= started + 1.0
    assert node._audio_queue.empty()
