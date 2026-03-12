from nao_replay_motion.head_motion_skill_server import HeadMotionSkillServer
from nao_replay_motion.replay_motion_skill_server import ReplayMotionSkillServer


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
