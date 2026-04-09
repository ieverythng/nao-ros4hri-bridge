import importlib.util
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _load_launch_module(relative_path: str, module_name: str):
    sys.path.insert(0, str(PACKAGE_ROOT))
    try:
        launch_path = PACKAGE_ROOT / relative_path
        spec = importlib.util.spec_from_file_location(module_name, launch_path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.pop(0)


def _launch_defaults(relative_path: str, module_name: str) -> dict[str, str]:
    module = _load_launch_module(relative_path, module_name)
    launch_description = module.generate_launch_description()
    defaults: dict[str, str] = {}
    for entity in launch_description.entities:
        if not isinstance(entity, DeclareLaunchArgument):
            continue
        default = entity.default_value
        if isinstance(default, str):
            defaults[entity.name] = default
            continue
        if isinstance(default, (list, tuple)):
            defaults[entity.name] = "".join(
                getattr(item, "text", str(item)) for item in default
            )
            continue
        defaults[entity.name] = getattr(default, "text", str(default))
    return defaults


def test_planner_local_profile_disables_runtime_nodes_by_default():
    defaults = _launch_defaults(
        "launch/nao_chatbot_planner_local.launch.py",
        "nao_chatbot_planner_local_launch_test",
    )
    assert defaults["start_planner_llm"] == "true"
    assert defaults["start_chatbot_llm"] == "false"
    assert defaults["start_dialogue_manager"] == "false"
    assert defaults["start_knowledge_core"] == "false"
    assert defaults["start_nao_say_skill"] == "false"
    assert defaults["start_nao_replay_motion"] == "false"
    assert defaults["start_nao_look_at"] == "false"
    assert defaults["start_robot_speech_debug"] == "false"


def test_sim_profile_keeps_interaction_sim_enabled_without_planner():
    defaults = _launch_defaults(
        "launch/nao_chatbot_sim.launch.py",
        "nao_chatbot_sim_launch_test",
    )
    assert defaults["start_interaction_sim"] == "true"
    assert defaults["start_interaction_sim_perception"] == "true"
    assert defaults["start_interaction_sim_tools"] == "true"
    assert defaults["start_rqt_console"] == "true"
    assert defaults["start_planner_llm"] == "false"
