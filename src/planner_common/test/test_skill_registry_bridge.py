from planner_common.skill_registry_bridge import merge_fake_skill_aliases
from planner_common.skill_registry_bridge import merge_scan_skill_names
from planner_common.skill_registry_bridge import merge_supported_skill_names


def test_merge_supported_skill_names_includes_fallback_and_manifest_aliases() -> None:
    names = merge_supported_skill_names(
        fallback_names=('scan', 'report_result'),
        manifest=[
            {
                'name': 'perform_motion',
                'aliases': ['motion'],
                'robot_adapter_mapping': 'nao_orchestrator.perform_motion',
            }
        ],
    )
    assert 'scan' in names
    assert 'report_result' in names
    assert 'perform_motion' in names
    assert 'motion' in names


def test_merge_scan_skill_names_uses_mapping_and_aliases() -> None:
    names = merge_scan_skill_names(
        fallback_names=('scan',),
        manifest=[
            {
                'name': 'scan',
                'aliases': ['look_around', 'inspect_scene'],
                'robot_adapter_mapping': 'nao_orchestrator.scan',
            }
        ],
    )
    assert names == {'scan', 'look_around', 'inspect_scene'}


def test_merge_fake_skill_aliases_uses_mapping_suffix() -> None:
    aliases = merge_fake_skill_aliases(
        fallback_aliases={'navigate_to': 'navigate_to'},
        manifest=[
            {
                'name': 'walk_to',
                'aliases': ['step_to'],
                'robot_adapter_mapping': 'fake_skills.walk_to',
            }
        ],
    )
    assert aliases['navigate_to'] == 'navigate_to'
    assert aliases['walk_to'] == 'walk_to'
    assert aliases['step_to'] == 'walk_to'
