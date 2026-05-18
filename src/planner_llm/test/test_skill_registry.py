from planner_llm.skill_registry import SkillRegistry


def test_skill_registry_loads_from_source_fallback_when_install_overlay_is_missing() -> None:
    registry = SkillRegistry.load()

    assert registry.step_types == ('noop', 'say', 'skill', 'look_at')
    assert 'perform_motion' in registry.allowed_skill_names
    assert 'motion' in registry.allowed_skill_names
    assert 'look_at' in registry.allowed_skill_names
    assert 'scan' in registry.allowed_skill_names
    assert 'report_result' in registry.allowed_skill_names


def test_skill_registry_derives_planner_skills_from_package_exports() -> None:
    registry = SkillRegistry.load()
    skills_by_name = {skill.name: skill for skill in registry.skills}

    perform_motion = skills_by_name['perform_motion']
    look_at = skills_by_name['look_at']
    scan = skills_by_name['scan']
    report_result = skills_by_name['report_result']

    assert perform_motion.robot_adapter_mapping == 'nao_orchestrator.perform_motion'
    assert perform_motion.required_params == ('object',)
    assert 'motion' in perform_motion.aliases
    assert 'posture' in perform_motion.aliases
    assert 'head_motion' in perform_motion.aliases
    assert look_at.robot_adapter_mapping == 'nao_orchestrator.look_at'
    assert 'gaze_at' in look_at.aliases
    assert scan.robot_adapter_mapping == 'nao_orchestrator.scan'
    assert scan.aliases == ('look_around', 'inspect_scene', 'check_visible_entities')
    assert scan.params == ('target', 'target_kind', 'max_sweeps', 'evidence_policy')
    assert scan.planner_guidance
    assert any('scan' in item.lower() for item in scan.planner_guidance)
    assert scan.observable_success
    assert 'perception' in scan.safety_flags
    assert report_result.robot_adapter_mapping == 'nao_orchestrator.report_result'
    assert report_result.params == ('summary_text',)


def test_skill_prompt_summary_exposes_planner_contract_fields() -> None:
    registry = SkillRegistry.load()
    summaries = {item['name']: item for item in registry.prompt_manifest()}

    look_at = summaries['look_at']
    scan = summaries['scan']

    assert 'aliases' in look_at
    assert 'params' in look_at
    assert 'observable_success' in look_at
    assert look_at['robot_adapter_mapping'] == 'nao_orchestrator.look_at'
    assert 'target_frame' in look_at['params']
    assert scan['robot_adapter_mapping'] == 'nao_orchestrator.scan'
    assert scan['observable_success']
    assert scan['planner_guidance']


def test_skill_registry_reports_rejected_steps() -> None:
    registry = SkillRegistry.load()
    supported, rejected = registry.filter_supported_steps_with_rejections(
        [
            {'type': 'skill', 'name': 'perform_motion', 'args': {'object': 'stand'}},
            {'type': 'skill', 'name': 'dance', 'args': {'style': 'wave'}},
        ]
    )

    assert [step['name'] for step in supported] == ['perform_motion']
    assert [step['name'] for step in rejected] == ['dance']


def test_skill_registry_canonicalizes_scan_alias_steps() -> None:
    registry = SkillRegistry.load()
    supported, rejected = registry.filter_supported_steps_with_rejections(
        [
            {'type': 'skill', 'name': 'look_around', 'args': {'target_kind': 'scene'}},
            {'type': 'skill', 'name': 'inspect_scene', 'args': {'target_kind': 'scene'}},
        ]
    )

    assert rejected == []
    assert [step['name'] for step in supported] == ['scan', 'scan']
