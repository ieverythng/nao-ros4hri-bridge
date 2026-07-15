import importlib

import pytest


@pytest.mark.skipif(importlib.util.find_spec('rclpy') is None, reason='rclpy unavailable in unit shell')
def test_action_server_module_imports() -> None:
    module = importlib.import_module('fake_skills.action_server')
    assert hasattr(module, 'FakeSkillActionServer')


@pytest.mark.skipif(importlib.util.find_spec('rclpy') is None, reason='rclpy unavailable in unit shell')
def test_action_server_mode_overrides_parser() -> None:
    module = importlib.import_module('fake_skills.action_server')
    parser = module.FakeSkillActionServer._parse_mode_overrides_json

    assert parser('{"find_object":"always_fail"}') == {'find_object': 'always_fail'}
    assert parser('{"find_object":""}') == {}
    assert parser('not-json') is None


@pytest.mark.skipif(importlib.util.find_spec('rclpy') is None, reason='rclpy unavailable in unit shell')
def test_action_server_imports_kb_query_boundary() -> None:
    module = importlib.import_module('fake_skills.action_server')

    assert hasattr(module, 'KnowledgeCoreQueryClient')
