import importlib

import pytest


@pytest.mark.skipif(importlib.util.find_spec('rclpy') is None, reason='rclpy unavailable in unit shell')
def test_action_server_module_imports() -> None:
    module = importlib.import_module('fake_skills.action_server')
    assert hasattr(module, 'FakeSkillActionServer')
