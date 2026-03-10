import importlib
import math
import sys
import types


def _set_stub_module(monkeypatch, name: str, module: types.ModuleType) -> None:
    monkeypatch.setitem(sys.modules, name, module)


def _install_common_stubs(monkeypatch) -> None:
    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda *args, **kwargs: None
    rclpy_module.shutdown = lambda *args, **kwargs: None
    _set_stub_module(monkeypatch, "rclpy", rclpy_module)

    action_module = types.ModuleType("rclpy.action")
    action_module.ActionClient = object
    action_module.ActionServer = object
    action_module.CancelResponse = types.SimpleNamespace(ACCEPT="accept")
    action_module.GoalResponse = types.SimpleNamespace(ACCEPT="accept", REJECT="reject")
    _set_stub_module(monkeypatch, "rclpy.action", action_module)

    callback_groups = types.ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = object
    _set_stub_module(monkeypatch, "rclpy.callback_groups", callback_groups)

    executors = types.ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    _set_stub_module(monkeypatch, "rclpy.executors", executors)

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node
    _set_stub_module(monkeypatch, "rclpy.node", rclpy_node)

    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = String
    std_msgs_pkg = types.ModuleType("std_msgs")
    std_msgs_pkg.msg = std_msgs_msg
    _set_stub_module(monkeypatch, "std_msgs", std_msgs_pkg)
    _set_stub_module(monkeypatch, "std_msgs.msg", std_msgs_msg)

    nao_skills_msg = types.ModuleType("nao_skills.action")

    class _SimpleFeedback:
        def __init__(self):
            self.status = ""
            self.progress = 0.0

    class _SimpleResult:
        def __init__(self):
            self.success = False
            self.message = ""
            self.duration = 0.0

    class DoPosture:
        class Goal:
            pass

        class Feedback(_SimpleFeedback):
            pass

        class Result(_SimpleResult):
            pass

    class DoHeadMotion:
        class Goal:
            pass

        class Feedback(_SimpleFeedback):
            pass

        class Result(_SimpleResult):
            pass

    nao_skills_msg.DoPosture = DoPosture
    nao_skills_msg.DoHeadMotion = DoHeadMotion
    nao_skills_pkg = types.ModuleType("nao_skills")
    nao_skills_pkg.action = nao_skills_msg
    _set_stub_module(monkeypatch, "nao_skills", nao_skills_pkg)
    _set_stub_module(monkeypatch, "nao_skills.action", nao_skills_msg)

    joint_angles_msg = types.ModuleType("naoqi_bridge_msgs.msg")
    joint_angles_msg.JointAnglesWithSpeed = type("JointAnglesWithSpeed", (), {})
    joint_angles_pkg = types.ModuleType("naoqi_bridge_msgs")
    joint_angles_pkg.msg = joint_angles_msg
    _set_stub_module(monkeypatch, "naoqi_bridge_msgs", joint_angles_pkg)
    _set_stub_module(monkeypatch, "naoqi_bridge_msgs.msg", joint_angles_msg)

    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = types.SimpleNamespace(
        STATUS_SUCCEEDED=4,
        STATUS_ABORTED=6,
        STATUS_CANCELED=5,
    )
    action_msgs_pkg = types.ModuleType("action_msgs")
    action_msgs_pkg.msg = action_msgs_msg
    _set_stub_module(monkeypatch, "action_msgs", action_msgs_pkg)
    _set_stub_module(monkeypatch, "action_msgs.msg", action_msgs_msg)

    communication_msg = types.ModuleType("communication_skills.action")

    class _SkillPayload:
        def __init__(self):
            self.error_code = 0
            self.error_msg = ""

    class Say:
        class Goal:
            def __init__(self):
                self.group_id = ""
                self.person_id = ""

        class Feedback:
            def __init__(self):
                self.feedback = types.SimpleNamespace(
                    data_str="",
                    data_float=0.0,
                )

        class Result:
            def __init__(self):
                self.result = _SkillPayload()

    communication_msg.Say = Say
    communication_pkg = types.ModuleType("communication_skills")
    communication_pkg.action = communication_msg
    _set_stub_module(monkeypatch, "communication_skills", communication_pkg)
    _set_stub_module(monkeypatch, "communication_skills.action", communication_msg)

    std_skills_msg = types.ModuleType("std_skills.msg")
    std_skills_msg.Result = types.SimpleNamespace(
        ROS_ENOERR=0,
        ROS_ECANCELED=1,
        ROS_EOTHER=2,
    )
    std_skills_pkg = types.ModuleType("std_skills")
    std_skills_pkg.msg = std_skills_msg
    _set_stub_module(monkeypatch, "std_skills", std_skills_pkg)
    _set_stub_module(monkeypatch, "std_skills.msg", std_skills_msg)

    tts_msg = types.ModuleType("tts_msgs.action")
    tts_msg.TTS = type("TTS", (), {"Goal": type("Goal", (), {})})
    tts_pkg = types.ModuleType("tts_msgs")
    tts_pkg.action = tts_msg
    _set_stub_module(monkeypatch, "tts_msgs", tts_pkg)
    _set_stub_module(monkeypatch, "tts_msgs.action", tts_msg)


def _import_module(monkeypatch, module_name: str):
    _install_common_stubs(monkeypatch)
    sys.modules.pop(module_name, None)
    return importlib.import_module(module_name)


def test_posture_skill_server_resolves_aliases_and_speed(monkeypatch):
    module = _import_module(monkeypatch, "nao_skill_servers.posture_skill_server")
    server = module.PostureSkillServer.__new__(module.PostureSkillServer)
    server.default_speed = 0.8

    assert module.PostureSkillServer._normalize_name(" Stand Init ") == "standinit"
    assert server._resolve_posture_name("kneel") == "Crouch"
    assert server._resolve_posture_name("stand full") == "Stand"
    assert server._resolve_posture_name("") is None
    assert server._resolve_speed(0.0) == 0.8
    assert server._resolve_speed(0.5) == 0.5
    assert server._resolve_speed(1.5) is None

    result = module.PostureSkillServer._result(True, "ok", 1.25)
    assert result.success is True
    assert result.message == "ok"
    assert result.duration == 1.25


def test_head_motion_skill_server_validates_ranges(monkeypatch):
    module = _import_module(monkeypatch, "nao_skill_servers.head_motion_skill_server")
    server = module.HeadMotionSkillServer.__new__(module.HeadMotionSkillServer)
    server.default_speed = 0.2
    server.yaw_min = -2.0
    server.yaw_max = 2.0
    server.pitch_min = -0.5
    server.pitch_max = 0.5

    assert server._resolve_speed(0.0) == 0.2
    assert server._resolve_speed(0.7) == 0.7
    assert server._resolve_speed(math.nan) is None
    assert server._validate_angles(0.1, 0.2, relative=False) is None
    assert "Yaw out of range" in server._validate_angles(3.0, 0.2, relative=False)
    assert "Pitch out of range" in server._validate_angles(0.2, 0.7, relative=False)
    assert server._validate_angles(9.0, -9.0, relative=True) is None

    result = module.HeadMotionSkillServer._result(False, "bad", 0.4)
    assert result.success is False
    assert result.message == "bad"
    assert result.duration == 0.4


def test_say_skill_server_helpers(monkeypatch):
    module = _import_module(monkeypatch, "nao_skill_servers.say_skill_server")
    goal = module.Say.Goal()
    goal.group_id = "turn:d0042"
    goal.person_id = ""

    assert module.SaySkillServer._as_bool(True) is True
    assert module.SaySkillServer._as_bool("yes") is True
    assert module.SaySkillServer._as_bool("off") is False
    assert module.SaySkillServer._extract_turn_id(goal) == "d0042"
    assert module.SaySkillServer._turn_label("d0042") == "[turn:d0042]"
    assert module.SaySkillServer._turn_label("") == "[turn:unknown]"

    result = module.SaySkillServer._canonical_result(False, "failed", 2)
    assert result.result.error_code == 2
    assert result.result.error_msg == "failed"

    ok_result = module.SaySkillServer._canonical_result(True, "ignored", 0)
    assert ok_result.result.error_code == 0
    assert ok_result.result.error_msg == ""
