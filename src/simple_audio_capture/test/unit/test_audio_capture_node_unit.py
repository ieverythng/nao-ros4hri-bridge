import importlib
import sys
import types


def _set_stub_module(monkeypatch, name: str, module: types.ModuleType) -> None:
    monkeypatch.setitem(sys.modules, name, module)


def _install_audio_capture_stubs(monkeypatch) -> None:
    gi_module = types.ModuleType("gi")
    gi_module.require_version = lambda *_args, **_kwargs: None

    class _DefaultRegistry:
        @staticmethod
        def get():
            return types.SimpleNamespace(get_feature_list=lambda _factory: [])

    default_gst = types.SimpleNamespace(
        init=lambda *_args, **_kwargs: None,
        Registry=_DefaultRegistry,
        ElementFactory=object,
    )
    repository_module = types.ModuleType("gi.repository")
    repository_module.Gst = default_gst
    repository_module.GLib = types.SimpleNamespace(MainLoop=lambda: None)

    gi_module.repository = repository_module
    _set_stub_module(monkeypatch, "gi", gi_module)
    _set_stub_module(monkeypatch, "gi.repository", repository_module)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda *_args, **_kwargs: None
    rclpy_module.ok = lambda: False
    _set_stub_module(monkeypatch, "rclpy", rclpy_module)

    rclpy_node_module = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node_module.Node = Node
    _set_stub_module(monkeypatch, "rclpy.node", rclpy_node_module)

    qos_module = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, depth=10, durability=None, reliability=None):
            self.depth = depth
            self.durability = durability
            self.reliability = reliability

    class QoSDurabilityPolicy:
        TRANSIENT_LOCAL = 1

    class QoSReliabilityPolicy:
        RELIABLE = 1

    qos_module.QoSProfile = QoSProfile
    qos_module.QoSDurabilityPolicy = QoSDurabilityPolicy
    qos_module.QoSReliabilityPolicy = QoSReliabilityPolicy
    _set_stub_module(monkeypatch, "rclpy.qos", qos_module)

    audio_msg = types.ModuleType("audio_common_msgs.msg")

    class AudioData:
        def __init__(self):
            self.data = []

    class AudioDataStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.audio = None

    class AudioInfo:
        def __init__(self):
            self.channels = 0
            self.sample_rate = 0
            self.sample_format = ""
            self.bitrate = 0
            self.coding_format = ""

    audio_msg.AudioData = AudioData
    audio_msg.AudioDataStamped = AudioDataStamped
    audio_msg.AudioInfo = AudioInfo
    audio_pkg = types.ModuleType("audio_common_msgs")
    audio_pkg.msg = audio_msg
    _set_stub_module(monkeypatch, "audio_common_msgs", audio_pkg)
    _set_stub_module(monkeypatch, "audio_common_msgs.msg", audio_msg)


def _import_audio_capture_node(monkeypatch):
    _install_audio_capture_stubs(monkeypatch)
    sys.modules.pop("simple_audio_capture.audio_capture_node", None)
    return importlib.import_module("simple_audio_capture.audio_capture_node")


class _Logger:
    def __init__(self):
        self.warning_msgs = []
        self.error_msgs = []

    def warning(self, message):
        self.warning_msgs.append(message)

    def error(self, message):
        self.error_msgs.append(message)


def _param(value):
    return types.SimpleNamespace(value=value)


def test_get_parameters_trims_source_type(monkeypatch):
    module = _import_audio_capture_node(monkeypatch)
    logger = _Logger()

    node = module.AudioCaptureNode.__new__(module.AudioCaptureNode)
    params = {
        "device": "",
        "source_type": "  pulsesrc  ",
        "format": "wave",
        "sample_format": "S16LE",
        "sample_rate": 16000,
        "channels": 1,
        "depth": 16,
        "chunk_size": 2048,
        "audio_topic": "laptop/microphone0",
    }
    node.get_parameter = lambda name: _param(params[name])
    node.get_logger = lambda: logger

    module.AudioCaptureNode._get_parameters(node)

    assert node.source_type == "pulsesrc"
    assert node.audio_topic == "laptop/microphone0"
    assert logger.warning_msgs == []


def test_get_parameters_defaults_empty_source_type(monkeypatch):
    module = _import_audio_capture_node(monkeypatch)
    logger = _Logger()

    node = module.AudioCaptureNode.__new__(module.AudioCaptureNode)
    params = {
        "device": "",
        "source_type": "   ",
        "format": "wave",
        "sample_format": "S16LE",
        "sample_rate": 16000,
        "channels": 1,
        "depth": 16,
        "chunk_size": 2048,
        "audio_topic": "audio",
    }
    node.get_parameter = lambda name: _param(params[name])
    node.get_logger = lambda: logger

    module.AudioCaptureNode._get_parameters(node)

    assert node.source_type == "pulsesrc"
    assert logger.warning_msgs


def test_publish_audio_info_populates_message(monkeypatch):
    module = _import_audio_capture_node(monkeypatch)
    logger = _Logger()
    published = []

    node = module.AudioCaptureNode.__new__(module.AudioCaptureNode)
    node.channels = 1
    node.sample_rate = 16000
    node.sample_format = "S16LE"
    node.format = "wave"
    node.audio_info_pub = types.SimpleNamespace(publish=lambda msg: published.append(msg))
    node.get_logger = lambda: logger

    module.AudioCaptureNode._publish_audio_info(node)

    assert len(published) == 1
    msg = published[0]
    assert msg.channels == 1
    assert msg.sample_rate == 16000
    assert msg.sample_format == "S16LE"
    assert msg.coding_format == "wave"


def test_list_audio_factories_filters_audio_entries(monkeypatch):
    module = _import_audio_capture_node(monkeypatch)

    class _Feature:
        def __init__(self, name, klass):
            self._name = name
            self._klass = klass

        def get_name(self):
            return self._name

        def get_klass(self):
            return self._klass

    class _Registry:
        def get_feature_list(self, _factory_type):
            return [
                _Feature("pulsesrc", "Source/Audio"),
                _Feature("videotestsrc", "Source/Video"),
                _Feature("alsasink", "Sink/Audio"),
            ]

    class _RegistryWrapper:
        @staticmethod
        def get():
            return _Registry()

    fake_gst = types.SimpleNamespace(
        Registry=_RegistryWrapper,
        ElementFactory=object,
    )
    monkeypatch.setattr(module, "Gst", fake_gst)

    node = module.AudioCaptureNode.__new__(module.AudioCaptureNode)
    results = module.AudioCaptureNode._list_audio_factories(node, limit=10)

    assert any("pulsesrc" in item for item in results)
    assert any("alsasink" in item for item in results)
    assert len(results) >= 2


def test_list_audio_factories_handles_registry_error(monkeypatch):
    module = _import_audio_capture_node(monkeypatch)

    class _RegistryWrapper:
        @staticmethod
        def get():
            raise RuntimeError("registry unavailable")

    fake_gst = types.SimpleNamespace(
        Registry=_RegistryWrapper,
        ElementFactory=object,
    )
    monkeypatch.setattr(module, "Gst", fake_gst)

    node = module.AudioCaptureNode.__new__(module.AudioCaptureNode)
    results = module.AudioCaptureNode._list_audio_factories(node, limit=5)

    assert results == ["<could not enumerate factories>"]
