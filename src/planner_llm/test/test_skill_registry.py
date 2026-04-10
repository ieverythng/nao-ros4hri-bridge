from planner_llm.skill_registry import SkillRegistry


def test_skill_registry_loads_from_source_fallback_when_install_overlay_is_missing() -> None:
    registry = SkillRegistry.load()

    assert registry.step_types == ('noop', 'say', 'skill', 'look_at')
    assert 'perform_motion' in registry.allowed_skill_names
    assert 'motion' in registry.allowed_skill_names
    assert 'look_at' in registry.allowed_skill_names


def test_skill_registry_derives_planner_skills_from_package_exports() -> None:
    registry = SkillRegistry.load()
    skills_by_name = {skill.name: skill for skill in registry.skills}

    perform_motion = skills_by_name['perform_motion']
    look_at = skills_by_name['look_at']

    assert perform_motion.robot_adapter_mapping == 'nao_orchestrator.perform_motion'
    assert perform_motion.required_params == ('object',)
    assert 'motion' in perform_motion.aliases
    assert look_at.robot_adapter_mapping == 'nao_orchestrator.look_at'
