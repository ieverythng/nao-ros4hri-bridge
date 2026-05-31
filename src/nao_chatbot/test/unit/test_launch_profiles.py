import importlib.util
import sys
from pathlib import Path

import pytest

try:
    from launch.actions import DeclareLaunchArgument
except ModuleNotFoundError:
    DeclareLaunchArgument = None

pytestmark = pytest.mark.skipif(
    DeclareLaunchArgument is None,
    reason="ROS 2 launch Python package is not available",
)


def _nao_chatbot_package_root() -> Path:
    """Resolve ``src/nao_chatbot`` even if tests move under ``tests/unit`` or similar."""
    here = Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / "launch" / "nao_chatbot_sim.launch.py").is_file():
            return candidate
    raise RuntimeError(
        f"Could not locate nao_chatbot package root from {here} "
        "(expected launch/nao_chatbot_sim.launch.py)."
    )


PACKAGE_ROOT = _nao_chatbot_package_root()
LAB_VLLM_MODEL = "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ"
LAB_VLLM_CHAT_URL = "http://10.7.138.215:8004/v1/chat/completions"
LAB_VLLM_BASE_URL = "http://10.7.138.215:8004"


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


def _assert_lab_vllm_defaults(defaults: dict[str, str]) -> None:
    assert defaults["planner_llm_provider"] == "openai_compatible"
    assert defaults["planner_llm_model"] == LAB_VLLM_MODEL
    assert defaults["chatbot_server_url"] == LAB_VLLM_CHAT_URL
    assert defaults["planner_llm_base_url"] == LAB_VLLM_BASE_URL
    assert defaults["start_managed_ollama"] == "false"
    assert defaults["chatbot_preflight_required"] == "true"
    assert defaults["planner_llm_preflight_required"] == "true"


def _assert_asr_is_opt_in(defaults: dict[str, str]) -> None:
    assert defaults["start_asr"] == "false"
    assert defaults["asr_audio_capture_enabled"] == "false"
    assert defaults["asr_push_to_talk_enabled"] == "true"
    assert defaults["chat_input_tracked_topic"] == "/nao_chatbot/humans/voices/tracked"
    assert (
        defaults["chat_input_speech_topic"]
        == "/nao_chatbot/humans/voices/anonymous_speaker/speech"
    )
    assert defaults["tts_backend_action_name"] == ""
    assert defaults["sim_use_laptop_tts"] == "false"


def _assert_planner_dialogue_seam_defaults(defaults: dict[str, str]) -> None:
    assert defaults["planner_dialogue_act_topic"] == "/planner/dialogue_act"
    assert (
        defaults["planner_dialogue_relay_topic"]
        == "/nao_orchestrator/planner_dialogue_act"
    )


def test_sim_profile_provides_gscam_camera_and_rqt_with_planner():
    defaults = _launch_defaults(
        "launch/nao_chatbot_sim.launch.py",
        "nao_chatbot_sim_launch_test",
    )
    assert defaults["start_interaction_sim"] == "true"
    assert defaults["start_interaction_sim_perception"] == "true"
    assert defaults["start_interaction_sim_tools"] == "true"
    assert defaults["start_naoqi_driver"] == "false"
    assert defaults["start_nao_robot"] == "false"
    assert defaults["object_detection_input_image_topic"] == "/camera/image_raw"
    assert defaults["hri_visualization_image_topic"] == "/camera/image_raw"
    assert defaults["start_rqt_console"] == "true"
    assert defaults["start_planner_llm"] == "true"
    assert defaults["chatbot_planner_mode_enabled"] == "true"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["start_fake_skills"] == "true"
    assert defaults["chatbot_planner_request_topic"] == "/nao_orchestrator/planner_request"
    _assert_planner_dialogue_seam_defaults(defaults)
    _assert_lab_vllm_defaults(defaults)
    _assert_asr_is_opt_in(defaults)


def test_robot_profile_uses_robot_camera_and_planner_mode_by_default():
    defaults = _launch_defaults(
        "launch/nao_chatbot_robot.launch.py",
        "nao_chatbot_robot_launch_test",
    )
    assert defaults["nao_ip"] == "172.26.112.25"
    assert defaults["start_nao_robot"] == "true"
    assert defaults["start_interaction_sim"] == "false"
    assert defaults["start_interaction_sim_perception"] == "false"
    assert defaults["object_detection_input_image_topic"] == "/camera/front/image_raw"
    assert defaults["hri_visualization_image_topic"] == "/camera/front/image_raw"
    assert defaults["start_planner_llm"] == "true"
    assert defaults["chatbot_planner_mode_enabled"] == "true"
    assert defaults["enable_orchestrator_planner_gate"] == "true"
    assert defaults["start_fake_skills"] == "true"
    _assert_planner_dialogue_seam_defaults(defaults)
    _assert_lab_vllm_defaults(defaults)
    _assert_asr_is_opt_in(defaults)


def test_demo_profile_is_sim_only_with_mock_scan_and_planner_enabled():
    defaults = _launch_defaults(
        "launch/nao_chatbot_demo.launch.py",
        "nao_chatbot_demo_launch_test",
    )
    assert defaults["start_naoqi_driver"] == "false"
    assert defaults["start_nao_robot"] == "false"
    assert defaults["start_interaction_sim"] == "true"
    assert defaults["start_interaction_sim_perception"] == "true"
    assert defaults["start_object_detection"] == "true"
    assert defaults["start_scene_grounding"] == "true"
    assert defaults["object_detection_backend"] == "emorobcare_cv"
    assert defaults["start_planner_llm"] == "true"
    assert defaults["chatbot_planner_mode_enabled"] == "true"
    assert defaults["start_fake_skills"] == "true"
    assert defaults["fake_skill_global_mode"] == "scenario"
    assert defaults["fake_skill_random_failure_prob"] == "0.50"
    assert defaults["fake_skill_mode_overrides_json"] == "{}"
    assert defaults["scan_result_mode"] == "success"
    assert "current scene summary" in defaults["scan_summary"]
    assert defaults["scan_report_after_success"] == "false"
    _assert_planner_dialogue_seam_defaults(defaults)
    assert defaults["interaction_trace_compact_mode"] == "false"
    assert defaults["interaction_trace_include_raw_payloads"] == "false"
    assert defaults["interaction_trace_include_channels_csv"] == ""
    assert defaults["interaction_trace_include_event_types_csv"] == ""
    _assert_lab_vllm_defaults(defaults)
    _assert_asr_is_opt_in(defaults)


def test_stack_uses_launch_events_for_chatbot_and_dialogue_lifecycle():
    stack_launch = _load_launch_module(
        "nao_chatbot/stack_launch.py",
        "nao_chatbot_stack_launch_test",
    )
    launch_description = stack_launch.generate_profile_launch_description()
    entity_type_names = [type(entity).__name__ for entity in launch_description.entities]

    assert "EmitEvent" in entity_type_names
    assert "RegisterEventHandler" in entity_type_names
