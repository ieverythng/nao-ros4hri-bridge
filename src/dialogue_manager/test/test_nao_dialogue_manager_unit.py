import importlib
import json
import sys
import types


def _set_stub_module(monkeypatch, name: str, module: types.ModuleType) -> None:
    monkeypatch.setitem(sys.modules, name, module)


def _install_stubs(monkeypatch) -> None:
    hri_msg = types.ModuleType("hri_msgs.msg")

    class LiveSpeech:
        def __init__(self, final="", incremental="", locale="", confidence=0.0):
            self.final = final
            self.incremental = incremental
            self.locale = locale
            self.confidence = confidence

    hri_msg.LiveSpeech = LiveSpeech
    hri_pkg = types.ModuleType("hri_msgs")
    hri_pkg.msg = hri_msg
    _set_stub_module(monkeypatch, "hri_msgs", hri_pkg)
    _set_stub_module(monkeypatch, "hri_msgs.msg", hri_msg)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda *args, **kwargs: None
    rclpy_module.shutdown = lambda *args, **kwargs: None
    rclpy_module.spin = lambda *args, **kwargs: None
    _set_stub_module(monkeypatch, "rclpy", rclpy_module)

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node
    _set_stub_module(monkeypatch, "rclpy.node", rclpy_node)

    std_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""

    std_msg.String = String
    std_pkg = types.ModuleType("std_msgs")
    std_pkg.msg = std_msg
    _set_stub_module(monkeypatch, "std_msgs", std_pkg)
    _set_stub_module(monkeypatch, "std_msgs.msg", std_msg)

    say_skill_client = types.ModuleType("dialogue_manager.say_skill_client")

    class SaySkillClient:
        def __init__(self, *_args, **_kwargs):
            pass

        def wait_for_server(self, timeout_sec=0.0):
            return True

    say_skill_client.SaySkillClient = SaySkillClient
    _set_stub_module(monkeypatch, "dialogue_manager.say_skill_client", say_skill_client)


def _import_module(monkeypatch):
    _install_stubs(monkeypatch)
    sys.modules.pop("dialogue_manager.nao_dialogue_manager", None)
    return importlib.import_module("dialogue_manager.nao_dialogue_manager")


class _Logger:
    def __init__(self):
        self.warn_msgs = []
        self.info_msgs = []
        self.error_msgs = []

    def warn(self, message):
        self.warn_msgs.append(message)

    def info(self, message):
        self.info_msgs.append(message)

    def error(self, message):
        self.error_msgs.append(message)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _build_manager(module, logger):
    node = module.DialogueManager.__new__(module.DialogueManager)
    node.accept_incremental_speech = False
    node.ignore_user_speech_while_busy = True
    node.user_turn_holdoff_sec = 0.6
    node.user_turn_min_chars = 2
    node.user_turn_min_words = 1
    node.dedupe_window_sec = 0.8
    node.user_text_topic = "/chatbot/user_text"
    node._last_user_text = ""
    node._last_user_text_ts = 0.0
    node._last_assistant_text = ""
    node._last_assistant_text_ts = 0.0
    node._awaiting_assistant_turn = False
    node._latest_intent = ""
    node._last_state_payload = ""
    node._speech_topic_sent_for_current_turn = False
    node._pending_assistant_text = ""
    node._pending_assistant_turn_id = ""
    node._pending_user_text = ""
    node._pending_user_locale = ""
    node._pending_user_confidence = 0.0
    node._pending_user_deadline_ts = 0.0
    node._current_turn_id = ""
    node._turn_counter = 0
    node.user_text_publisher = _Publisher()
    node.dialogue_state_publisher = _Publisher()
    node.get_logger = lambda: logger
    node._trace = lambda *_args, **_kwargs: None
    node._set_state = lambda *_args, **_kwargs: None
    return node


def test_on_live_speech_ignores_incremental_by_default(monkeypatch):
    module = _import_module(monkeypatch)
    logger = _Logger()
    node = _build_manager(module, logger)

    msg = module.LiveSpeech(final="", incremental="hello there", locale="en_US")
    module.DialogueManager._on_live_speech(node, msg)

    assert node._pending_user_text == ""
    assert node.user_text_publisher.messages == []
    assert logger.warn_msgs


def test_on_live_speech_buffers_and_merges_before_forward(monkeypatch):
    module = _import_module(monkeypatch)
    logger = _Logger()
    node = _build_manager(module, logger)
    current_time = [10.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: current_time[0])

    first = module.LiveSpeech(final="hello", locale="en_US", confidence=0.5)
    second = module.LiveSpeech(final="hello robot", locale="en_US", confidence=0.7)

    module.DialogueManager._on_live_speech(node, first)
    assert node._pending_user_text == "hello"
    assert node.user_text_publisher.messages == []

    current_time[0] = 10.2
    module.DialogueManager._on_live_speech(node, second)
    assert node._pending_user_text == "hello robot"

    current_time[0] = 10.9
    module.DialogueManager._flush_pending_user_turn_if_due(node)

    assert len(node.user_text_publisher.messages) == 1
    payload = json.loads(node.user_text_publisher.messages[0].data)
    assert payload["text"] == "hello robot"
    assert payload["turn_id"] == "d00001"
    assert node._awaiting_assistant_turn is True


def test_flush_pending_user_turn_ignores_recent_duplicate(monkeypatch):
    module = _import_module(monkeypatch)
    logger = _Logger()
    node = _build_manager(module, logger)
    current_time = [20.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: current_time[0])

    node._last_user_text = "hello robot"
    node._last_user_text_ts = 19.5
    node._pending_user_text = "hello robot"
    node._pending_user_deadline_ts = 19.9

    module.DialogueManager._flush_pending_user_turn_if_due(node)

    assert node.user_text_publisher.messages == []
    assert logger.warn_msgs


def test_on_live_speech_ignored_while_busy(monkeypatch):
    module = _import_module(monkeypatch)
    logger = _Logger()
    node = _build_manager(module, logger)
    node._awaiting_assistant_turn = True

    msg = module.LiveSpeech(final="new request", locale="en_US")
    module.DialogueManager._on_live_speech(node, msg)

    assert node._pending_user_text == ""
    assert node.user_text_publisher.messages == []
