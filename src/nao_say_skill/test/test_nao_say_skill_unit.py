from nao_say_skill.skill_impl import NaoSaySkill


class _Goal:
    def __init__(self, person_id="", group_id=""):
        self.person_id = person_id
        self.group_id = group_id


def test_extract_turn_id_prefers_turn_prefix():
    goal = _Goal(person_id="turn:abc123")
    assert NaoSaySkill._extract_turn_id(goal) == "abc123"


def test_extract_turn_id_falls_back_to_plain_identifier():
    goal = _Goal(group_id="operator")
    assert NaoSaySkill._extract_turn_id(goal) == "operator"


def test_as_bool_accepts_string_variants():
    assert NaoSaySkill._as_bool("true") is True
    assert NaoSaySkill._as_bool("on") is True
    assert NaoSaySkill._as_bool("off") is False


def test_turn_label_handles_empty_value():
    assert NaoSaySkill._turn_label("") == "[turn:unknown]"
