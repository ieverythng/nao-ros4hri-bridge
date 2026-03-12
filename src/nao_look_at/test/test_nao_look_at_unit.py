from nao_look_at.skill_impl import NaoLookAtSkill


class _Header:
    def __init__(self, frame_id=""):
        self.frame_id = frame_id


class _Target:
    def __init__(self, frame_id=""):
        self.header = _Header(frame_id=frame_id)


class _Goal:
    def __init__(self, frame_id=""):
        self.target = _Target(frame_id=frame_id)


def test_normalize_policy_is_lowercase():
    assert NaoLookAtSkill._normalize_policy(" RESET ") == "reset"


def test_has_target_uses_frame_id():
    assert NaoLookAtSkill._has_target(_Goal(frame_id="map")) is True
    assert NaoLookAtSkill._has_target(_Goal(frame_id="")) is False
