"""Contract checks for typed fake-skill action endpoints."""

from pathlib import Path


def test_look_at_has_a_declared_typed_action_endpoint() -> None:
    source = (
        Path(__file__).parents[1] / 'fake_skills' / 'action_server.py'
    ).read_text(encoding='utf-8')

    assert "self.declare_parameter('look_at_action_name', '/skill/fake/look_at')" in source
    assert "('look_at_action_name', 'look_at')" in source


def test_manipulation_skills_have_declared_typed_action_endpoints() -> None:
    source = (
        Path(__file__).parents[1] / 'fake_skills' / 'action_server.py'
    ).read_text(encoding='utf-8')

    for skill_name in ('pick_object', 'place_object', 'bring_object'):
        assert (
            "self.declare_parameter('%s_action_name', '/skill/fake/%s')"
            % (skill_name, skill_name)
        ) in source
        assert "('%s_action_name', '%s')" % (skill_name, skill_name) in source
