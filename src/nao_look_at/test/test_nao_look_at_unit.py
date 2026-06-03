import pytest

NaoLookAtSkill = pytest.importorskip("nao_look_at.skill_impl").NaoLookAtSkill


class _Header:
    def __init__(self, frame_id=""):
        self.frame_id = frame_id
        self.stamp = type("Stamp", (), {"sec": 0, "nanosec": 0})()


class _Target:
    def __init__(self, frame_id="", x=0.0, y=0.0, z=0.0):
        self.header = _Header(frame_id=frame_id)
        self.point = type("Point", (), {"x": x, "y": y, "z": z})()


class _Goal:
    def __init__(self, frame_id="", x=0.0, y=0.0, z=0.0):
        self.target = _Target(frame_id=frame_id, x=x, y=y, z=z)


def test_normalize_policy_is_lowercase():
    assert NaoLookAtSkill._normalize_policy(" RESET ") == "reset"


def test_has_target_uses_frame_id():
    assert NaoLookAtSkill._has_target(_Goal(frame_id="map")) is True
    assert NaoLookAtSkill._has_target(_Goal(frame_id="")) is False


def test_publish_reset_pose_returns_false_without_joint_publisher():
    skill = NaoLookAtSkill.__new__(NaoLookAtSkill)
    skill._joint_angles_pub = None
    skill.reset_yaw = 0.0
    skill.reset_pitch = 0.0
    assert skill._publish_reset_pose() is False


def test_vector_to_angles_uses_forward_left_up_convention():
    yaw, pitch = NaoLookAtSkill._vector_to_angles(1.0, 1.0, 0.5)
    assert round(yaw, 3) == 0.785
    assert round(pitch, 3) == -0.34


def test_rotate_vector_handles_identity_quaternion():
    rotated = NaoLookAtSkill._rotate_vector((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert rotated == (1.0, 2.0, 3.0)


def test_resolve_target_vector_uses_direct_reference_frame_without_tf():
    skill = NaoLookAtSkill.__new__(NaoLookAtSkill)
    skill.look_from_frame = "base_link"
    skill.fallback_look_from_frame = "CameraTop_frame"
    skill._tf_buffer = None
    skill.minimum_target_distance_m = 0.05
    resolved = skill._resolve_target_vector(_Goal(frame_id="base_link", x=1.0, y=0.2, z=0.1))
    assert resolved == (1.0, 0.2, 0.1, "base_link")


def test_resolve_policy_pose_auto_resets():
    skill = NaoLookAtSkill.__new__(NaoLookAtSkill)
    skill.reset_yaw = 0.0
    skill.reset_pitch = 0.0
    skill.max_yaw_abs = 1.5
    skill.min_pitch = -0.67
    skill.max_pitch = 0.51

    assert skill._resolve_policy_pose("auto") == (0.0, 0.0, "auto_reset")


def test_resolve_policy_pose_random_is_bounded():
    skill = NaoLookAtSkill.__new__(NaoLookAtSkill)
    skill.random_yaw_abs = 0.2
    skill.random_pitch_min = -0.1
    skill.random_pitch_max = 0.1
    skill.max_yaw_abs = 1.5
    skill.min_pitch = -0.67
    skill.max_pitch = 0.51

    yaw, pitch, label = skill._resolve_policy_pose("random")

    assert label == "random_scan"
    assert -0.2 <= yaw <= 0.2
    assert -0.1 <= pitch <= 0.1
