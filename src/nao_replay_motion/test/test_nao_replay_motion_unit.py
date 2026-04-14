from nao_replay_motion.head_motion_skill_server import HeadMotionSkillServer
from nao_replay_motion.replay_motion_skill_server import ReplayMotionSkillServer
from nao_replay_motion.replay_motion_skill_server import _parse_posture_result_message
from nao_replay_motion.replay_motion_skill_server import _posture_result_matches


def test_replay_motion_aliases_are_stable():
    server = ReplayMotionSkillServer.__new__(ReplayMotionSkillServer)
    assert server._resolve_motion(" Stand Init ") == ("standinit", "StandInit")
    assert server._resolve_motion("kneel") == ("kneel", "Crouch")
    assert server._resolve_motion("unknown") is None


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


def test_posture_result_helpers_match_bridge_payload():
    payload = _parse_posture_result_message(
        '{"command":"stand","normalized_command":"stand","posture_name":"Stand","success":true,"message":"Executed posture command"}'
    )

    assert payload["success"] is True
    assert _posture_result_matches(payload, "stand") is True
    assert _posture_result_matches(payload, "Stand") is True
    assert _parse_posture_result_message("not-json") == {}
