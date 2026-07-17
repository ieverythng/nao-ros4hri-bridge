from nao_replay_motion.head_motion_skill_server import HeadMotionSkillServer
from nao_replay_motion.replay_motion_skill_server import ReplayMotionSkillServer
from nao_replay_motion.replay_motion_skill_server import _parse_posture_result_message
from nao_replay_motion.replay_motion_skill_server import _posture_result_matches


class _FakeGoalHandle:
    def __init__(self, cancel_requested=False):
        self.is_cancel_requested = cancel_requested
        self.feedback = []
        self.succeeded = False
        self.aborted = False
        self.canceled_called = False

    def publish_feedback(self, feedback):
        self.feedback.append(feedback.status)

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True

    def canceled(self):
        self.canceled_called = True


def test_replay_motion_aliases_are_stable():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    assert server._resolve_motion(" Stand Init ") == ("standinit", "StandInit")
    assert server._resolve_motion("kneel") == ("kneel", "Crouch")
    assert server._resolve_motion("unknown") is None


def test_replay_motion_open_loop_succeeds_when_naoqi_is_unavailable():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    server.default_speed = 0.8
    server.allow_open_loop_without_naoqi = True
    server.fallback_to_posture_topic = False
    server._ensure_connection = lambda: False
    server._publish_replay_feedback = ReplayMotionSkillServer._publish_replay_feedback

    goal_handle = _FakeGoalHandle()
    result = server._execute_motion(
        goal_handle=goal_handle,
        requested_name="standinit",
        requested_speed=0.8,
        feedback_builder=lambda handle, status, _progress: handle.feedback.append(status),
        result_builder=lambda success, message, duration: {
            "success": success,
            "message": message,
            "duration": duration,
        },
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result["success"] is True
    assert result["message"] == "Executed motion 'standinit' via open_loop"
    assert goal_handle.feedback == ["preparing", "executing", "completing"]


def test_replay_motion_prefers_confirmed_posture_bridge_over_open_loop():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    server.default_speed = 0.8
    server.allow_open_loop_without_naoqi = True
    server.fallback_to_posture_topic = True
    server._ensure_connection = lambda: False
    server._posture_bridge_available = lambda: True
    bridge_calls = []
    server._execute_posture_via_topic_fallback = (
        lambda posture_name: bridge_calls.append(posture_name) or "bridge confirmed"
    )

    goal_handle = _FakeGoalHandle()
    result = server._execute_motion(
        goal_handle=goal_handle,
        requested_name="stand",
        requested_speed=0.8,
        feedback_builder=lambda handle, status, _progress: handle.feedback.append(status),
        result_builder=lambda success, message, duration: {
            "success": success,
            "message": message,
            "duration": duration,
        },
    )

    assert bridge_calls == ["Stand"]
    assert result["success"] is True
    assert result["message"] == (
        "Executed motion 'stand' via topic_fallback (bridge confirmed)"
    )


def test_replay_motion_uses_open_loop_when_posture_bridge_reports_failure():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    server.default_speed = 0.8
    server.allow_open_loop_without_naoqi = True
    server.fallback_to_posture_topic = True
    server._ensure_connection = lambda: False
    server._posture_bridge_available = lambda: True
    server._execute_posture_via_topic_fallback = lambda _posture_name: (_ for _ in ()).throw(
        RuntimeError("Failed to execute posture command")
    )

    goal_handle = _FakeGoalHandle()
    result = server._execute_motion(
        goal_handle=goal_handle,
        requested_name="stand",
        requested_speed=0.8,
        feedback_builder=lambda handle, status, _progress: handle.feedback.append(status),
        result_builder=lambda success, message, duration: {
            "success": success,
            "message": message,
            "duration": duration,
        },
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result["success"] is True
    assert result["message"] == "Executed motion 'stand' via open_loop"


def test_replay_motion_uses_open_loop_when_posture_bridge_is_not_ready():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    server.default_speed = 0.8
    server.allow_open_loop_without_naoqi = True
    server.fallback_to_posture_topic = True
    server._ensure_connection = lambda: False
    server._posture_bridge_available = lambda: False
    server._execute_posture_via_topic_fallback = lambda _posture_name: (_ for _ in ()).throw(
        AssertionError("unavailable bridge must not receive a posture command")
    )

    goal_handle = _FakeGoalHandle()
    result = server._execute_motion(
        goal_handle=goal_handle,
        requested_name="stand",
        requested_speed=0.8,
        feedback_builder=lambda handle, status, _progress: handle.feedback.append(status),
        result_builder=lambda success, message, duration: {
            "success": success,
            "message": message,
            "duration": duration,
        },
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result["success"] is True
    assert result["message"] == "Executed motion 'stand' via open_loop"


def test_head_motion_validation_rejects_out_of_range_absolute_motion():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.yaw_min = -1.0
    server.yaw_max = 1.0
    server.pitch_min = -0.5
    server.pitch_max = 0.5
    assert server._validate_angles(2.0, 0.0, relative=False) is not None
    assert server._validate_angles(0.0, 0.0, relative=False) is None


def test_head_motion_resolves_relative_targets_from_current_state():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    resolved = server._resolve_target_angles(
        yaw=0.2,
        pitch=-0.1,
        relative=True,
        current_state={"HeadYaw": 0.5, "HeadPitch": 0.25},
    )

    assert resolved == (0.7, 0.15)


def test_head_motion_reaches_target_within_tolerance():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.convergence_tolerance_rad = 0.05
    server._current_head_state = lambda: {"HeadYaw": 0.48, "HeadPitch": -0.02}

    assert server._has_reached_target(0.5, 0.0) is True
    assert server._has_reached_target(0.6, 0.0) is False


def test_head_motion_timeout_reason_flags_unchanged_joint_state():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server._HEAD_JOINTS = ("HeadYaw", "HeadPitch")
    server.convergence_tolerance_rad = 0.08
    server.joint_angles_topic = "/joint_angles"
    server._current_head_state = lambda: {"HeadYaw": -0.42, "HeadPitch": 0.05}
    server._joint_state_age_sec = lambda: 0.3

    reason = server._convergence_timeout_reason(
        target_yaw=0.45,
        target_pitch=0.0,
        initial_state={"HeadYaw": -0.42, "HeadPitch": 0.05},
    )

    assert "did not change after publishing" in reason


def test_head_motion_open_loop_dispatch_succeeds_for_absolute_goal():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.yaw_min = -1.0
    server.yaw_max = 1.0
    server.pitch_min = -0.5
    server.pitch_max = 0.5
    server.joint_angles_topic = "/joint_angles"
    server._publish_feedback = HeadMotionSkillServer._publish_feedback
    server._result = HeadMotionSkillServer._result
    server.get_logger = lambda: type("Logger", (), {"warn": lambda *_args: None})()
    published = []
    server._publish_joint_angles = lambda **kwargs: published.append(kwargs)

    goal_handle = _FakeGoalHandle()
    result = server._execute_open_loop(
        goal_handle,
        start_time=0.0,
        yaw=0.4,
        pitch=0.0,
        speed=0.2,
        relative=False,
        target_yaw=0.4,
        target_pitch=0.0,
        reason="no joint state",
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert goal_handle.feedback == ["executing_open_loop", "completing"]
    assert result.success is True
    assert published == [{"yaw": 0.4, "pitch": 0.0, "speed": 0.2, "relative": False}]


def test_head_motion_open_loop_dispatch_still_validates_absolute_target():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.yaw_min = -0.2
    server.yaw_max = 0.2
    server.pitch_min = -0.5
    server.pitch_max = 0.5
    server._result = HeadMotionSkillServer._result
    server.get_logger = lambda: type("Logger", (), {"warn": lambda *_args: None})()
    server._publish_joint_angles = lambda **_kwargs: None

    goal_handle = _FakeGoalHandle()
    result = server._execute_open_loop(
        goal_handle,
        start_time=0.0,
        yaw=0.4,
        pitch=0.0,
        speed=0.2,
        relative=False,
        target_yaw=0.4,
        target_pitch=0.0,
        reason="no joint state",
    )

    assert goal_handle.aborted is True
    assert goal_handle.succeeded is False
    assert result.success is False
    assert "Yaw out of range" in result.message


def test_head_motion_convergence_timeout_fails_when_not_assuming_success():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.assume_success_on_convergence_timeout = False
    server._result = HeadMotionSkillServer._result
    server.get_logger = lambda: type("Logger", (), {"warn": lambda *_args: None})()

    goal_handle = _FakeGoalHandle()
    result = server._outcome_after_convergence_timeout(
        goal_handle,
        start_time=0.0,
        reason="Head motion timed out before convergence",
        target_yaw=0.4,
        target_pitch=0.0,
    )

    assert goal_handle.aborted is True
    assert goal_handle.succeeded is False
    assert result.success is False
    assert "timed out" in result.message


def test_head_motion_convergence_timeout_can_be_debug_open_loop_success():
    server = HeadMotionSkillServer.__new__(HeadMotionSkillServer)
    server.assume_success_on_convergence_timeout = True
    server._result = HeadMotionSkillServer._result
    server._publish_feedback = HeadMotionSkillServer._publish_feedback
    server.get_logger = lambda: type("Logger", (), {"warn": lambda *_args: None})()

    goal_handle = _FakeGoalHandle()
    result = server._outcome_after_convergence_timeout(
        goal_handle,
        start_time=0.0,
        reason="Head motion timed out before convergence",
        target_yaw=0.4,
        target_pitch=0.0,
    )

    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result.success is True
    assert "convergence was not observed" in result.message


def test_posture_result_helpers_match_bridge_payload():
    payload = _parse_posture_result_message(
        '{"command":"stand","normalized_command":"stand","posture_name":"Stand","success":true,"message":"Executed posture command"}'
    )

    assert payload["success"] is True
    assert _posture_result_matches(payload, "stand") is True
    assert _posture_result_matches(payload, "Stand") is True
    assert _parse_posture_result_message("not-json") == {}
