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
    assert defaults["planner_llm_model"] == "gemma4:31b-cloud"
    assert defaults["planner_llm_preflight_required"] == "true"


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
    assert defaults["interaction_sim_hri_log_profile"] == "quiet"
    assert defaults["scene_grounding_fallback_match_distance_px"] == "40.0"
    assert defaults["scene_grounding_fallback_match_max_age_sec"] == "1.2"
    assert defaults["planner_llm_model"] == "gemma4:31b-cloud"
    assert defaults["chatbot_preflight_required"] == "true"
    assert defaults["planner_llm_preflight_required"] == "true"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["chatbot_planner_request_topic"] == "/nao_orchestrator/planner_request"
    assert defaults["chatbot_server_url"] == "http://127.0.0.1:11434/api/chat"
    assert defaults["planner_llm_base_url"] == "http://127.0.0.1:11435"
    assert defaults["start_managed_ollama"] == "false"
    assert defaults["start_demo_log_window"] == "true"


def test_robot_profile_enables_planner_mode_by_default():
    defaults = _launch_defaults(
        "launch/nao_chatbot_robot.launch.py",
        "nao_chatbot_robot_launch_test",
    )
    assert defaults["nao_ip"] == "172.26.112.25"
    assert defaults["start_planner_llm"] == "true"
    assert defaults["chatbot_planner_mode_enabled"] == "true"
    assert defaults["planner_llm_model"] == "gemma4:31b-cloud"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["chatbot_planner_request_topic"] == "/nao_orchestrator/planner_request"
    assert defaults["chatbot_server_url"] == "http://127.0.0.1:11434/api/chat"
    assert defaults["planner_llm_base_url"] == "http://127.0.0.1:11435"
    assert defaults["start_managed_ollama"] == "false"
    assert defaults["start_demo_log_window"] == "true"


def test_robot_demo_profile_enables_demo_scan_defaults():
    defaults = _launch_defaults(
        "launch/nao_chatbot_robot_demo.launch.py",
        "nao_chatbot_robot_demo_launch_test",
    )
    assert defaults["enable_demo_scan_skill"] == "true"
    assert defaults["demo_scan_result_mode"] == "success"
    assert "demo scene summary" in defaults["demo_scan_summary"]
    assert defaults["chatbot_preflight_required"] == "true"
    assert defaults["planner_llm_preflight_required"] == "true"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["chatbot_planner_request_topic"] == "/nao_orchestrator/planner_request"
    assert defaults["chatbot_server_url"] == "http://127.0.0.1:11434/api/chat"
    assert defaults["planner_llm_base_url"] == "http://127.0.0.1:11435"
    assert defaults["start_managed_ollama"] == "false"
    assert defaults["start_demo_log_window"] == "true"


def test_sim_demo_profile_matches_demo_scan_defaults():
    defaults = _launch_defaults(
        "launch/nao_chatbot_sim_demo.launch.py",
        "nao_chatbot_sim_demo_launch_test",
    )
    assert defaults["start_naoqi_driver"] == "true"
    assert defaults["start_object_detection"] == "true"
    assert defaults["start_scene_grounding"] == "true"
    assert defaults["start_planner_llm"] == "true"
    assert defaults["chatbot_planner_mode_enabled"] == "true"
    assert defaults["enable_demo_scan_skill"] == "true"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["start_managed_ollama"] == "false"
    assert defaults["chatbot_planner_request_topic"] == "/nao_orchestrator/planner_request"


def test_stack_uses_launch_events_for_chatbot_and_dialogue_lifecycle():
    stack_launch = _load_launch_module(
        "nao_chatbot/stack_launch.py",
        "nao_chatbot_stack_launch_test",
    )
    launch_description = stack_launch.generate_profile_launch_description()
    entity_type_names = [type(entity).__name__ for entity in launch_description.entities]

    assert 'EmitEvent' in entity_type_names
    assert 'RegisterEventHandler' in entity_type_names


def test_stack_does_not_poll_chatbot_lifecycle_with_ros2_cli():
    stack_launch = _load_launch_module(
        "nao_chatbot/stack_launch.py",
        "nao_chatbot_stack_launch_budget_test",
    )
    source = Path(stack_launch.__file__).read_text()

    assert '_lifecycle_wait_then_bootstrap_script' not in source
    assert '_service_wait_then_lifecycle_bootstrap_script' not in source
    assert 'ros2 lifecycle get "$wait_node_name"' not in source
