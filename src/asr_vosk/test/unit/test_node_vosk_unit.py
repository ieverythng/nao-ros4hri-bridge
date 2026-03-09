import importlib
import sys
import types


def _set_stub_module(monkeypatch, name: str, module: types.ModuleType) -> None:
    monkeypatch.setitem(sys.modules, name, module)


def _install_node_vosk_stubs(monkeypatch) -> None:
    audio_msg = types.ModuleType("audio_common_msgs.msg")

    class AudioData:
        def __init__(self, data=None):
            self.data = data or []

    audio_msg.AudioData = AudioData
    audio_pkg = types.ModuleType("audio_common_msgs")
    audio_pkg.msg = audio_msg
    _set_stub_module(monkeypatch, "audio_common_msgs", audio_pkg)
    _set_stub_module(monkeypatch, "audio_common_msgs.msg", audio_msg)

    diagnostic_msg = types.ModuleType("diagnostic_msgs.msg")

    class DiagnosticArray:
        pass

    class DiagnosticStatus:
        OK = 0

    class KeyValue:
        def __init__(self, key="", value=""):
            self.key = key
            self.value = value

    diagnostic_msg.DiagnosticArray = DiagnosticArray
    diagnostic_msg.DiagnosticStatus = DiagnosticStatus
    diagnostic_msg.KeyValue = KeyValue
    diagnostic_pkg = types.ModuleType("diagnostic_msgs")
    diagnostic_pkg.msg = diagnostic_msg
    _set_stub_module(monkeypatch, "diagnostic_msgs", diagnostic_pkg)
    _set_stub_module(monkeypatch, "diagnostic_msgs.msg", diagnostic_msg)

    hri_msg = types.ModuleType("hri_msgs.msg")

    class IdsList:
        def __init__(self, ids=None):
            self.ids = ids or []

    class LiveSpeech:
        def __init__(self, locale=""):
            self.locale = locale
            self.header = types.SimpleNamespace(stamp=None)
            self.incremental = ""
            self.final = ""

    hri_msg.IdsList = IdsList
    hri_msg.LiveSpeech = LiveSpeech
    hri_pkg = types.ModuleType("hri_msgs")
    hri_pkg.msg = hri_msg
    _set_stub_module(monkeypatch, "hri_msgs", hri_pkg)
    _set_stub_module(monkeypatch, "hri_msgs.msg", hri_msg)

    lifecycle_msg = types.ModuleType("lifecycle_msgs.msg")

    class State:
        PRIMARY_STATE_ACTIVE = 3
        PRIMARY_STATE_INACTIVE = 2

    lifecycle_msg.State = State
    lifecycle_pkg = types.ModuleType("lifecycle_msgs")
    lifecycle_pkg.msg = lifecycle_msg
    _set_stub_module(monkeypatch, "lifecycle_msgs", lifecycle_pkg)
    _set_stub_module(monkeypatch, "lifecycle_msgs.msg", lifecycle_msg)

    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")

    class ParameterDescriptor:
        def __init__(self, description=""):
            self.description = description

    class SetParametersResult:
        def __init__(self, successful=False, reason=""):
            self.successful = successful
            self.reason = reason

    rcl_interfaces_msg.ParameterDescriptor = ParameterDescriptor
    rcl_interfaces_msg.SetParametersResult = SetParametersResult
    rcl_interfaces_pkg = types.ModuleType("rcl_interfaces")
    rcl_interfaces_pkg.msg = rcl_interfaces_msg
    _set_stub_module(monkeypatch, "rcl_interfaces", rcl_interfaces_pkg)
    _set_stub_module(monkeypatch, "rcl_interfaces.msg", rcl_interfaces_msg)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda *args, **kwargs: None
    _set_stub_module(monkeypatch, "rclpy", rclpy_module)

    rclpy_exec = types.ModuleType("rclpy.executors")

    class SingleThreadedExecutor:
        def add_node(self, _node):
            return None

        def spin(self):
            return None

    class ExternalShutdownException(Exception):
        pass

    rclpy_exec.SingleThreadedExecutor = SingleThreadedExecutor
    rclpy_exec.ExternalShutdownException = ExternalShutdownException
    _set_stub_module(monkeypatch, "rclpy.executors", rclpy_exec)

    rclpy_lifecycle = types.ModuleType("rclpy.lifecycle")

    class Node:
        pass

    class LifecycleState:
        pass

    class TransitionCallbackReturn:
        FAILURE = "failure"

    rclpy_lifecycle.Node = Node
    rclpy_lifecycle.LifecycleState = LifecycleState
    rclpy_lifecycle.TransitionCallbackReturn = TransitionCallbackReturn
    _set_stub_module(monkeypatch, "rclpy.lifecycle", rclpy_lifecycle)

    rclpy_parameter = types.ModuleType("rclpy.parameter")

    class Parameter:
        class Type:
            STRING = 4

    rclpy_parameter.Parameter = Parameter
    _set_stub_module(monkeypatch, "rclpy.parameter", rclpy_parameter)

    std_msg = types.ModuleType("std_msgs.msg")

    class Bool:
        def __init__(self, data=False):
            self.data = data

    std_msg.Bool = Bool
    std_pkg = types.ModuleType("std_msgs")
    std_pkg.msg = std_msg
    _set_stub_module(monkeypatch, "std_msgs", std_pkg)
    _set_stub_module(monkeypatch, "std_msgs.msg", std_msg)

    vosk_module = types.ModuleType("vosk")

    class Model:
        def __init__(self, _path):
            pass

    class KaldiRecognizer:
        def __init__(self, _model, _rate):
            pass

    vosk_module.Model = Model
    vosk_module.KaldiRecognizer = KaldiRecognizer
    _set_stub_module(monkeypatch, "vosk", vosk_module)


def _import_node_vosk(monkeypatch):
    _install_node_vosk_stubs(monkeypatch)
    sys.modules.pop("asr_vosk.node_vosk", None)
    return importlib.import_module("asr_vosk.node_vosk")


class _Logger:
    def __init__(self):
        self.info_msgs = []
        self.error_msgs = []

    def info(self, message):
        self.info_msgs.append(message)

    def error(self, message):
        self.error_msgs.append(message)


def test_load_model_success_sets_recognizer(monkeypatch):
    module = _import_node_vosk(monkeypatch)
    logger = _Logger()

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.model = "/models/fake"
    node.audio_rate = 16000
    node.get_logger = lambda: logger

    monkeypatch.setattr(module, "Model", lambda model_path: {"model_path": model_path})
    monkeypatch.setattr(
        module,
        "KaldiRecognizer",
        lambda model, rate: {"model": model, "rate": rate},
    )

    success, reason = module.NodeVosk.load_model(node, node.model)

    assert success is True
    assert reason == ""
    assert node.recognizer == {"model": {"model_path": "/models/fake"}, "rate": 16000}


def test_load_model_failure_returns_error(monkeypatch):
    module = _import_node_vosk(monkeypatch)
    logger = _Logger()

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.model = "/models/missing"
    node.audio_rate = 16000
    node.get_logger = lambda: logger

    def _raise(_model_path):
        raise RuntimeError("missing model")

    monkeypatch.setattr(module, "Model", _raise)

    success, reason = module.NodeVosk.load_model(node, node.model)

    assert success is False
    assert "Failed to load" in reason
    assert "missing model" in reason
    assert logger.error_msgs


def test_on_parameter_change_updates_microphone_topic(monkeypatch):
    module = _import_node_vosk(monkeypatch)
    logger = _Logger()

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.microphone_topic = "/laptop/microphone0"
    node.audio_data_sub = "old_subscription"
    node.get_logger = lambda: logger

    destroyed = []
    node.destroy_subscription = lambda sub: destroyed.append(sub)
    node.create_subscription = (
        lambda _msg_type, topic, _callback, _qos: f"sub:{topic}"
    )

    param = types.SimpleNamespace(
        name="microphone_topic",
        type_=module.Parameter.Type.STRING,
        value="/robot/microphone0",
    )

    result = module.NodeVosk.on_parameter_change(node, [param])

    assert result.successful is True
    assert destroyed == ["old_subscription"]
    assert node.audio_data_sub == "sub:/robot/microphone0"
    assert node.microphone_topic == "/robot/microphone0"


def test_on_parameter_change_rejects_same_topic(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.microphone_topic = "/same/topic"
    node.audio_data_sub = None
    node.get_logger = lambda: _Logger()

    param = types.SimpleNamespace(
        name="microphone_topic",
        type_=module.Parameter.Type.STRING,
        value="/same/topic",
    )

    result = module.NodeVosk.on_parameter_change(node, [param])

    assert result.successful is False
    assert "same" in result.reason.lower()


def test_on_parameter_change_rejects_invalid_type(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.microphone_topic = "/topic"
    node.audio_data_sub = None
    node.get_logger = lambda: _Logger()

    param = types.SimpleNamespace(
        name="microphone_topic",
        type_=999,
        value="/new_topic",
    )

    result = module.NodeVosk.on_parameter_change(node, [param])

    assert result.successful is False
    assert "invalid" in result.reason.lower()


def test_on_robot_speaking_toggles_listening(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.operator_listening_enabled = True
    node.robot_speaking = False
    node.listening = True

    module.NodeVosk.on_robot_speaking(node, types.SimpleNamespace(data=True))
    assert node.listening is False

    module.NodeVosk.on_robot_speaking(node, types.SimpleNamespace(data=False))
    assert node.listening is True


def test_on_push_to_talk_enables_listening(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.operator_listening_enabled = False
    node.robot_speaking = False
    node.listening = False

    module.NodeVosk.on_push_to_talk(node, types.SimpleNamespace(data=True))

    assert node.operator_listening_enabled is True
    assert node.listening is True


def test_on_push_to_talk_respects_robot_speaking_gate(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.operator_listening_enabled = False
    node.robot_speaking = True
    node.listening = False

    module.NodeVosk.on_push_to_talk(node, types.SimpleNamespace(data=True))

    assert node.operator_listening_enabled is True
    assert node.listening is False


def test_extract_avg_confidence_from_result_words(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    confidence = module.NodeVosk._extract_avg_confidence(
        node,
        {
            "text": "hello world",
            "result": [{"word": "hello", "conf": 0.90}, {"word": "world", "conf": 0.70}],
        },
    )

    assert confidence == 0.80


def test_extract_avg_confidence_without_words(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    confidence = module.NodeVosk._extract_avg_confidence(node, {"text": "hello"})

    assert confidence == 0.0


def test_should_publish_final_rejects_filler(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.min_final_chars = 2
    node.min_final_words = 1
    node.min_final_confidence = 0.0
    node.ignore_single_token_fillers = True
    node.single_token_fillers = {"huh", "uh"}

    allowed, reason = module.NodeVosk._should_publish_final(node, "huh", 0.95)

    assert allowed is False
    assert reason == "single_token_filler"


def test_should_publish_final_respects_confidence_threshold(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.min_final_chars = 2
    node.min_final_words = 1
    node.min_final_confidence = 0.6
    node.ignore_single_token_fillers = True
    node.single_token_fillers = {"huh", "uh"}

    allowed, reason = module.NodeVosk._should_publish_final(node, "stand up", 0.3)

    assert allowed is False
    assert reason == "min_final_confidence"


def test_should_publish_final_accepts_regular_text(monkeypatch):
    module = _import_node_vosk(monkeypatch)

    node = module.NodeVosk.__new__(module.NodeVosk)
    node.min_final_chars = 2
    node.min_final_words = 1
    node.min_final_confidence = 0.0
    node.ignore_single_token_fillers = True
    node.single_token_fillers = {"huh", "uh"}

    allowed, reason = module.NodeVosk._should_publish_final(node, "hello nao", 0.4)

    assert allowed is True
    assert reason == ""
