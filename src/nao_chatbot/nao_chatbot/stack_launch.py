"""
Shared launch builder for the kept NAO demo profiles.

The public launch files are intentionally small wrappers. This module owns the
common launch arguments, optional external integrations, and lifecycle startup
ordering for the simulator and real-robot profiles.
"""

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from nao_chatbot.interaction_sim_support import build_interaction_sim_actions


DEFAULT_VLLM_MODEL = "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ"
DEFAULT_VLLM_BASE_URL = "http://10.7.138.215:8004"
DEFAULT_VLLM_CHAT_URL = DEFAULT_VLLM_BASE_URL + "/v1/chat/completions"

_LAB_VLLM_DEFAULTS = {
    "chatbot_model": DEFAULT_VLLM_MODEL,
    "planner_llm_provider": "openai_compatible",
    "planner_llm_model": DEFAULT_VLLM_MODEL,
    "chatbot_server_url": DEFAULT_VLLM_CHAT_URL,
    "planner_llm_base_url": DEFAULT_VLLM_BASE_URL,
    "start_managed_ollama": "false",
    "start_demo_log_window": "true",
    "chatbot_think": "false",
    "chatbot_temperature": "0.2",
    "chatbot_top_p": "0.9",
    "chatbot_top_k": "0",
    "chatbot_min_p": "0.0",
    "chatbot_presence_penalty": "0.0",
    "chatbot_repetition_penalty": "1.0",
    "planner_llm_think": "false",
    "planner_llm_temperature": "0.1",
    "planner_llm_top_p": "1.0",
    "planner_llm_top_k": "0",
    "planner_llm_min_p": "0.0",
    "planner_llm_presence_penalty": "0.0",
    "planner_llm_repetition_penalty": "1.0",
    "chatbot_preflight_required": "true",
    "chatbot_preflight_keepalive_interval_sec": "180.0",
    "planner_llm_preflight_required": "true",
    "chatbot_request_timeout_sec": "60.0",
    "chatbot_first_request_timeout_sec": "75.0",
    "chatbot_response_max_tokens": "192",
    "chatbot_turn_pipeline_mode": "response_first",
    "chatbot_preflight_timeout_sec": "60.0",
    "chatbot_preflight_attempts": "3",
    "chatbot_preflight_realistic_enabled": "true",
    "planner_llm_timeout_sec": "60.0",
    "planner_llm_preflight_timeout_sec": "60.0",
    "planner_llm_preflight_attempts": "3",
    "planner_llm_preflight_realistic_enabled": "true",
}
_PLANNER_GATE_DEFAULTS = {
    "enable_orchestrator_planner_gate": "true",
    "chatbot_planner_request_topic": "/nao_orchestrator/planner_request",
}
_SIM_CAMERA_DEFAULTS = {
    "start_naoqi_driver": "false",
    "start_nao_robot": "false",
    "start_nao_robot_hri_visualization": "false",
    "start_rviz": "false",
    "hri_visualization_image_topic": "/camera/image_raw",
    "object_detection_input_image_topic": "/camera/image_raw",
    "start_interaction_sim": "true",
    "start_interaction_sim_perception": "true",
    "start_interaction_sim_tools": "true",
    "start_interaction_sim_expressive_face": "false",
    "start_interaction_sim_ui": "false",
    "interaction_sim_hri_log_profile": "quiet",
    "start_rqt_console": "true",
    "start_rqt_chat": "true",
    "sim_use_laptop_tts": "false",
    "start_interaction_trace_viewer": "false",
    "interaction_trace_compact_mode": "false",
    "interaction_trace_enable_scene_summary_channel": "false",
    "interaction_trace_scene_summary_emit_on_change_only": "true",
    "interaction_trace_scene_summary_min_interval_sec": "1.0",
    "interaction_trace_rosout_min_level": "warn",
    "interaction_trace_rosout_node_allowlist_csv": (
        "chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,"
        "report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,"
        "head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug"
    ),
    "start_fake_skills": "true",
    "fake_skill_global_mode": "scenario",
    "fake_skill_random_failure_prob": "0.50",
    "fake_skill_mode_overrides_json": "{}",
    "preloaded_environment_ids": "",
    "preloaded_environment_lifespan_sec": "1800.0",
    "head_motion_allow_open_loop_without_joint_state": "true",
    "posture_allow_open_loop_without_naoqi": "true",
    "head_motion_assume_success_on_convergence_timeout": "false",
    "perform_motion_execution_mode": "real",
    "look_at_execution_mode": "fake",
}
_ROBOT_CAMERA_DEFAULTS = {
    "nao_ip": "172.26.112.25",
    "start_naoqi_driver": "false",
    "start_nao_robot": "true",
    "start_nao_robot_hri_visualization": "true",
    "start_rviz": "true",
    "start_object_detection": "true",
    "start_scene_grounding": "true",
    "object_detection_backend": "emorobcare_cv",
    "hri_visualization_image_topic": "/camera/front/image_raw",
    "object_detection_input_image_topic": "/camera/front/image_raw",
    "posture_bridge_connect_on_startup": "true",
    "posture_bridge_disable_autonomous_life_on_connect": "false",
    "posture_bridge_wake_up_on_connect": "true",
    "head_motion_allow_open_loop_without_joint_state": "true",
    "posture_allow_open_loop_without_naoqi": "true",
    "head_motion_assume_success_on_convergence_timeout": "false",
    "perform_motion_execution_mode": "real",
    "look_at_execution_mode": "fake",
    "start_interaction_sim": "false",
    "start_interaction_sim_perception": "false",
    "start_interaction_sim_tools": "true",
    "start_interaction_sim_ui": "false",
    "start_rqt_console": "false",
    "sim_use_laptop_tts": "false",
    "start_interaction_trace_viewer": "false",
    "interaction_trace_enable_scene_summary_channel": "false",
    "interaction_trace_scene_summary_emit_on_change_only": "true",
    "interaction_trace_scene_summary_min_interval_sec": "1.0",
    "interaction_trace_rosout_min_level": "warn",
    "interaction_trace_rosout_node_allowlist_csv": (
        "chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,"
        "report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,"
        "head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug"
    ),
    "start_fake_skills": "true",
    "fake_skill_global_mode": "scenario",
    "fake_skill_random_failure_prob": "0.50",
    "fake_skill_mode_overrides_json": "{}",
}

_RQT_CONTAINER_ENV_GUARD = (
    'export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"; '
    'mkdir -p "$XDG_RUNTIME_DIR"; '
    'chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true; '
)
_SCENE_GROUNDING_ALLOWED_LABELS = ",".join(
    (
        "bottle",
        "cup",
        "book",
        "cell phone",
        "backpack",
        "remote",
        "laptop",
        "keyboard",
        "mouse",
        "chair",
        "blueberry",
        "corn",
        "pear",
        "tomato",
        "zucchini",
    )
)
_DEMO_LOG_NODES = ",".join(
    (
        "chatbot_llm",
        "planner_llm",
        "nao_orchestrator",
        "scan_skill_server",
        "report_result_skill_server",
        "fake_skill_server",
        "dialogue_manager",
        "nao_say_skill",
        "head_motion_skill_server",
        "replay_motion_skill_server",
        "nao_look_at",
        "robot_speech_debug",
    )
)
_GROUNDING_DEFAULTS = {
    "object_detection_threshold": "0.70",
    "scene_grounding_knowledge_lifespan_sec": "8.0",
    "scene_grounding_knowledge_refresh_interval_sec": "0.75",
    "scene_grounding_local_stale_after_sec": "10.0",
    "scene_grounding_fallback_match_distance_px": "72.0",
    "scene_grounding_fallback_match_max_age_sec": "3.0",
}
_DEMO_DEFAULTS = {
    "nao_ip": "172.26.112.25",
    "network_interface": "wlp1s0",
    "start_object_detection": "true",
    "start_scene_grounding": "true",
    "object_detection_backend": "emorobcare_cv",
    "start_planner_llm": "true",
    "chatbot_planner_mode_enabled": "true",
    "scan_result_mode": "success",
    "scan_summary": "I looked around and can report the current scene summary.",
    "scan_report_after_success": "false",
}


def _merged_defaults(*sections: dict[str, str]) -> dict[str, str]:
    defaults: dict[str, str] = {}
    for section in sections:
        defaults.update(section)
    return defaults


def sim_profile_defaults() -> dict[str, str]:
    return _merged_defaults(
        _LAB_VLLM_DEFAULTS,
        _PLANNER_GATE_DEFAULTS,
        _SIM_CAMERA_DEFAULTS,
        _GROUNDING_DEFAULTS,
        {
            "start_planner_llm": "true",
            "chatbot_planner_mode_enabled": "true",
        },
    )


def robot_profile_defaults() -> dict[str, str]:
    return _merged_defaults(
        _LAB_VLLM_DEFAULTS,
        _PLANNER_GATE_DEFAULTS,
        _ROBOT_CAMERA_DEFAULTS,
        _GROUNDING_DEFAULTS,
        {
            "start_planner_llm": "true",
            "chatbot_planner_mode_enabled": "true",
        },
    )


def demo_profile_defaults() -> dict[str, str]:
    return _merged_defaults(
        sim_profile_defaults(),
        _DEMO_DEFAULTS,
    )


# -----------------------------------------------------------------------------
# Generic launch helpers
# -----------------------------------------------------------------------------


def _profile_default(profile_defaults: dict | None, name: str, fallback: str) -> str:
    """Resolve a per-profile default while keeping the shared argument list small."""
    if profile_defaults and name in profile_defaults:
        return str(profile_defaults[name])
    return str(fallback)


def _make_lifecycle_bundle(
    *,
    package_name,
    executable,
    node_name,
    condition,
    extra_parameters=None,
    remappings=None,
):
    """Create one lifecycle node plus the bootstrap process that activates it."""
    config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "00-defaults.yml"]
    )
    parameters = [config_path]
    if extra_parameters:
        parameters.extend(extra_parameters)

    node = LifecycleNode(
        package=package_name,
        executable=executable,
        namespace="",
        name=node_name,
        parameters=parameters,
        remappings=remappings or [],
        output="both",
        emulate_tty=True,
        condition=condition,
    )
    bootstrap = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            _lifecycle_bootstrap_script(node_name),
        ],
        output="screen",
        condition=condition,
    )
    return [node, bootstrap]


def _managed_ollama_script() -> str:
    """Start ollama unless the configured host already responds."""
    return r"""set +e
python3 - <<'PY'
import os
import sys
import urllib.request

host = os.environ.get("OLLAMA_HOST", "").strip()
url = "http://%s/api/tags" % host
try:
    with urllib.request.urlopen(url, timeout=2.0) as response:
        print("[OLLAMA] existing server ready | host=%s status=%s" % (host, response.status), flush=True)
        sys.exit(0)
except Exception as exc:
    print("[OLLAMA] starting managed server | host=%s reason=%s" % (host, exc), flush=True)
    sys.exit(1)
PY
probe_status="$?"
set -e
if [ "${probe_status}" -eq 0 ]; then
  exec sleep infinity
fi
if ! command -v ollama >/dev/null 2>&1; then
  echo "[OLLAMA] ollama executable is missing; rebuild the container image with the updated Dockerfile" >&2
  exit 127
fi
exec ollama serve
"""


def _not_launching_chatbot_llm_condition():
    """Condition used when dialogue_manager must boot without chatbot_llm."""
    return IfCondition(
        PythonExpression(
            [
                '"',
                LaunchConfiguration("start_dialogue_manager"),
                '" == "true" and "',
                LaunchConfiguration("start_chatbot_llm"),
                '" != "true"',
            ]
        )
    )


def _standalone_naoqi_driver_condition():
    """Launch naoqi_driver only when the packaged nao_robot path is not used."""
    return IfCondition(
        PythonExpression(
            [
                '"',
                LaunchConfiguration("start_naoqi_driver"),
                '" == "true" and "',
                LaunchConfiguration("start_nao_robot"),
                '" != "true"',
            ]
        )
    )


def _lifecycle_bootstrap_script(node_name: str, timeout_sec: int = 120) -> str:
    """Generate a shell loop that configures and activates one lifecycle node."""
    normalized_name = f"/{str(node_name).lstrip('/')}"
    return f"""
node_name="{normalized_name}"
exec 9>"/tmp/nao_chatbot_lifecycle_${{node_name#/}}.lock"
flock 9
deadline=$((SECONDS + {max(1, int(timeout_sec))}))
while true; do
  state="$(ros2 lifecycle get "$node_name" 2>/dev/null | \
awk '/^(unconfigured|inactive|active|finalized|errorprocessing)/{{print $1; exit}}')"
  case "$state" in
    active)
      exit 0
      ;;
    inactive)
      ros2 lifecycle set "$node_name" activate >/dev/null 2>&1 || true
      ;;
    unconfigured)
      ros2 lifecycle set "$node_name" configure >/dev/null 2>&1 || true
      ;;
    finalized|errorprocessing)
      echo "lifecycle bootstrap failed for $node_name: state=$state" >&2
      exit 1
      ;;
  esac
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "lifecycle bootstrap timed out for $node_name (last_state=${{state:-unknown}})" >&2
    exit 1
  fi
  sleep 0.2
done
""".strip()


def _lifecycle_recovery_script(node_name: str, timeout_sec: int = 240) -> str:
    """Retry lifecycle configure/activate after startup races."""
    normalized_name = f"/{str(node_name).lstrip('/')}"
    return f"""
node_name="{normalized_name}"
exec 9>"/tmp/nao_chatbot_lifecycle_${{node_name#/}}.lock"
flock 9
deadline=$((SECONDS + {max(1, int(timeout_sec))}))
while true; do
  state="$(ros2 lifecycle get "$node_name" 2>/dev/null | \
awk '/^(unconfigured|inactive|active|finalized|errorprocessing)/{{print $1; exit}}')"
  case "$state" in
    active)
      exit 0
      ;;
    inactive)
      ros2 lifecycle set "$node_name" activate >/dev/null 2>&1 || true
      ;;
    unconfigured)
      ros2 lifecycle set "$node_name" configure >/dev/null 2>&1 || true
      ;;
    finalized|errorprocessing)
      echo "lifecycle recovery failed for $node_name: state=$state" >&2
      exit 1
      ;;
  esac
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "lifecycle recovery timed out for $node_name (last_state=${{state:-unknown}})" >&2
    exit 1
  fi
  sleep 1.0
done
""".strip()


def _service_wait_script(service_name: str, timeout_sec: int = 30) -> str:
    """Generate a shell loop that waits for one ROS service to appear."""
    normalized_name = f"/{str(service_name).lstrip('/')}"
    return f"""
service_name="{normalized_name}"
deadline=$((SECONDS + {max(1, int(timeout_sec))}))
while true; do
  if ros2 service type "$service_name" >/dev/null 2>&1; then
    exit 0
  fi
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "service wait timed out for $service_name" >&2
    exit 1
  fi
  sleep 0.2
done
""".strip()


def _configure_lifecycle_node(node, *, condition=None):
    """Emit a configure transition for a launch-managed lifecycle node."""
    return EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=condition,
    )


def _activate_lifecycle_node_on_inactive(node, *, condition=None):
    """Activate a launch-managed lifecycle node after it reaches inactive."""
    return RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
            handle_once=True,
        ),
        condition=condition,
    )


def _configure_and_activate_lifecycle_node(node, *, condition=None):
    """Return launch actions that configure a lifecycle node, then activate it."""
    return (
        _configure_lifecycle_node(node, condition=condition),
        _activate_lifecycle_node_on_inactive(node, condition=condition),
    )


def _configure_lifecycle_node_after_active(active_node, target_node, *, condition=None):
    """Configure one lifecycle node only after another reaches active."""
    return RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=active_node,
            goal_state='active',
            entities=[_configure_lifecycle_node(target_node)],
            handle_once=True,
        ),
        condition=condition,
    )


def _prefer_first_non_empty(*names: str):
    """Build a launch expression that picks the first non-empty argument value."""
    if not names:
        raise ValueError("At least one launch argument name is required")

    expression: list[str] = []
    for index, name in enumerate(names):
        expression.extend(
            [
                '"',
                LaunchConfiguration(name),
                '"',
            ]
        )
        if index < len(names) - 1:
            expression.extend(
                [
                    ' if "',
                    LaunchConfiguration(name),
                    '" != "" else ',
                ]
            )
    return PythonExpression(expression)


def _nao_say_backend_action_name():
    """Pick laptop debug TTS only for simulator-only runs when explicitly requested."""
    return PythonExpression(
        [
            '"',
            LaunchConfiguration("sim_use_laptop_tts"),
            '" == "true" and "',
            LaunchConfiguration("start_naoqi_driver"),
            '" != "true" and "',
            LaunchConfiguration("start_nao_robot"),
            '" != "true" and "',
            LaunchConfiguration("debug_tts_action_name"),
            '" or "',
            LaunchConfiguration("tts_backend_action_name"),
            '"',
        ]
    )


def _effective_posture_bridge_wake_up_on_connect():
    """Force wake-up when a real robot driver path is active."""
    return PythonExpression(
        [
            '"true" if ("',
            LaunchConfiguration("posture_bridge_wake_up_on_connect"),
            '" == "true" or "',
            LaunchConfiguration("start_naoqi_driver"),
            '" == "true" or "',
            LaunchConfiguration("start_nao_robot"),
            '" == "true") else "false"',
        ]
    )


# -----------------------------------------------------------------------------
# Optional external integrations
# -----------------------------------------------------------------------------


def _optional_launch_description(
    context,
    *,
    package_name: str,
    launch_file_name: str,
    launch_arg_name: str,
    required_packages=None,
    launch_arguments=None,
    display_name=None,
):
    """Include an upstream launch file only when its package dependencies exist."""
    if LaunchConfiguration(launch_arg_name).perform(context).lower() != "true":
        return []

    missing_packages = []
    for required_package in required_packages or []:
        try:
            get_package_share_directory(required_package)
        except PackageNotFoundError:
            missing_packages.append(required_package)

    if missing_packages:
        return [
            LogInfo(
                msg=(
                    f"{display_name or package_name} launch skipped because the following upstream "
                    f"packages are missing: {', '.join(missing_packages)}"
                )
            )
        ]

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f"{package_name} is not installed in this workspace; "
                    f"skipping optional launch '{launch_file_name}'."
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", launch_file_name)
            ),
            launch_arguments=(launch_arguments or {}).items(),
        )
    ]


def _optional_object_detection_launch(context):
    """Start the selected detector backend behind a single launch argument."""
    if LaunchConfiguration("start_object_detection").perform(context).lower() != "true":
        return []

    backend = LaunchConfiguration("object_detection_backend").perform(context).strip().lower()
    if backend in ("yolo_ros", "yolo"):
        return _optional_launch_description(
            context,
            package_name="yolo_bringup",
            launch_file_name="yolo.launch.py",
            launch_arg_name="start_object_detection",
            display_name="yolo_ros",
            launch_arguments={
                "namespace": LaunchConfiguration("object_detection_namespace"),
                "model": LaunchConfiguration("object_detection_model"),
                "device": LaunchConfiguration("object_detection_device"),
                "threshold": LaunchConfiguration("object_detection_threshold"),
                "input_image_topic": LaunchConfiguration("object_detection_input_image_topic"),
                "image_reliability": LaunchConfiguration("object_detection_image_reliability"),
                "use_tracking": "True",
                "use_debug": "True",
            },
            required_packages=["yolo_bringup", "yolo_ros"],
        )

    if backend in ("emorobcare_cv", "emorobot", "emorobcare"):
        try:
            get_package_share_directory("emorobcare_cv_object_detection")
            get_package_share_directory("emorobcare_cv_msgs")
        except PackageNotFoundError:
            return [
                LogInfo(
                    msg=(
                        "emorobcare object detection launch skipped because "
                        "'emorobcare_cv_object_detection' and/or "
                        "'emorobcare_cv_msgs' are not installed in this environment."
                    )
                )
            ]
        object_detector_node = Node(
            package="emorobcare_cv_object_detection",
            executable="object_detector_node",
            name="object_detector_node",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("object_detection_log_level"),
            ],
            remappings=[
                ("/camera/image_raw", LaunchConfiguration("object_detection_input_image_topic")),
            ],
        )
        object_detector_bootstrap = ExecuteProcess(
            cmd=["bash", "-lc", _lifecycle_bootstrap_script("object_detector_node")],
            output="screen",
        )
        object_detector_recovery = TimerAction(
            period=24.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-lc", _lifecycle_recovery_script("object_detector_node")],
                    output="screen",
                )
            ],
        )
        return [
            LogInfo(
                msg=(
                    "Launching emorobcare_cv_object_detection. Keep its package-local "
                    "config.yaml aligned with this demo path: use_knowledge_base=false, "
                    "use_human_radar=false, and draw_image=true when debug overlays are needed."
                )
            ),
            object_detector_node,
            object_detector_bootstrap,
            object_detector_recovery,
        ]

    return [
        LogInfo(
            msg=(
                "Unsupported object_detection_backend='%s'. Supported backends are "
                "emorobcare_cv and yolo_ros."
            )
            % backend
        )
    ]


def _optional_rviz_launch(
    context,
    *,
    launch_arg_name: str,
    package_name: str,
    config_relative_path: str,
    display_name=None,
):
    """Start RViz with the packaged demo config when the profile requests it."""
    if LaunchConfiguration(launch_arg_name).perform(context).lower() != "true":
        return []

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f"{display_name or package_name} launch skipped because package "
                    f"'{package_name}' is not installed in this environment."
                )
            )
        ]

    rviz_config = os.path.join(package_share, config_relative_path)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        LogInfo(msg=f"RViz started with config: {rviz_config}"),
    ]


def _optional_hri_visualization_launch(context):
    """Remap hri_visualization onto the active robot camera topic."""
    if LaunchConfiguration("start_nao_robot_hri_visualization").perform(context).lower() != "true":
        return []

    try:
        package_share = get_package_share_directory("hri_visualization")
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "hri_visualization launch skipped because package "
                    "'hri_visualization' is not installed in this environment."
                )
            )
        ]

    image_topic = (
        LaunchConfiguration("hri_visualization_image_topic").perform(context).strip()
        or "/camera/front/image_raw"
    )
    return [
        GroupAction(
            scoped=True,
            actions=[
                SetRemap(src="image", dst=image_topic),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            package_share,
                            "launch",
                            "hri_visualization.launch.py",
                        )
                    )
                ),
            ],
        ),
        LogInfo(
            msg=(
                "hri_visualization input remapped to %s; overlay output is "
                "published on /image/hri_overlay/compressed"
            )
            % image_topic
        ),
    ]


# -----------------------------------------------------------------------------
# Shared profile builder
# -----------------------------------------------------------------------------


def generate_profile_launch_description(
    *,
    profile_defaults: dict | None = None,
):
    """Build the full launch description used by sim and robot wrappers."""
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value=_profile_default(profile_defaults, "start_naoqi_driver", "false"),
        description="Optionally launch naoqi_driver alongside the migrated ROS4HRI stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value=_profile_default(profile_defaults, "nao_ip", ""),
        description="NAO robot IP passed to replay motion nodes and naoqi_driver.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port passed to replay motion nodes and naoqi_driver.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface used by naoqi_driver when enabled.",
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL used by naoqi_driver when enabled.",
    )
    start_chatbot_llm_arg = DeclareLaunchArgument(
        "start_chatbot_llm",
        default_value=_profile_default(profile_defaults, "start_chatbot_llm", "true"),
        description="Launch the upstream-aligned chatbot_llm backend.",
    )
    start_nao_robot_arg = DeclareLaunchArgument(
        "start_nao_robot",
        default_value=_profile_default(profile_defaults, "start_nao_robot", "false"),
        description=(
            "Launch the packaged nao_robot bring-up (naoqi_driver + NAO camera "
            "face detection) for real-robot validation."
        ),
    )
    start_nao_robot_hri_visualization_arg = DeclareLaunchArgument(
        "start_nao_robot_hri_visualization",
        default_value=_profile_default(
            profile_defaults,
            "start_nao_robot_hri_visualization",
            "true",
        ),
        description=(
            "Launch hri_visualization together with nao_robot for robot-camera "
            "overlay topics and diagnostics."
        ),
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value=_profile_default(profile_defaults, "start_rviz", "false"),
        description="Launch rviz2 using the packaged robot-scene validation config.",
    )
    hri_visualization_image_topic_arg = DeclareLaunchArgument(
        "hri_visualization_image_topic",
        default_value=_profile_default(
            profile_defaults,
            "hri_visualization_image_topic",
            "/camera/front/image_raw",
        ),
        description=(
            "Base image topic consumed by hri_visualization; the compressed "
            "transport of this topic is used for overlays."
        ),
    )
    start_object_detection_arg = DeclareLaunchArgument(
        "start_object_detection",
        default_value=_profile_default(profile_defaults, "start_object_detection", "false"),
        description="Optionally launch the configured external object detector backend.",
    )
    object_detection_backend_arg = DeclareLaunchArgument(
        "object_detection_backend",
        default_value="emorobcare_cv",
        description="External detector backend to launch: emorobcare_cv or yolo_ros.",
    )
    start_scene_grounding_arg = DeclareLaunchArgument(
        "start_scene_grounding",
        default_value=_profile_default(profile_defaults, "start_scene_grounding", "false"),
        description="Launch the local object-to-KnowledgeCore grounding node.",
    )
    start_planner_llm_arg = DeclareLaunchArgument(
        "start_planner_llm",
        default_value=_profile_default(profile_defaults, "start_planner_llm", "false"),
        description="Launch the planner_llm node that converts planner requests into executable plans.",
    )
    object_detection_namespace_arg = DeclareLaunchArgument(
        "object_detection_namespace",
        default_value="yolo",
        description="Namespace used for the external detector stack.",
    )
    object_detection_model_arg = DeclareLaunchArgument(
        "object_detection_model",
        default_value="yolov8n.pt",
        description="Detector model name or path forwarded to yolo_ros.",
    )
    object_detection_device_arg = DeclareLaunchArgument(
        "object_detection_device",
        default_value="cpu",
        description="Detector device forwarded to yolo_ros, for example cpu or cuda:0.",
    )
    object_detection_log_level_arg = DeclareLaunchArgument(
        "object_detection_log_level",
        default_value=_profile_default(profile_defaults, "object_detection_log_level", "warn"),
        description=(
            "ROS log level used for the emorobcare detector node. "
            "Override to info or debug when diagnosing detector startup."
        ),
    )
    object_detection_threshold_arg = DeclareLaunchArgument(
        "object_detection_threshold",
        default_value=_profile_default(
            profile_defaults,
            "object_detection_threshold",
            "0.70",
        ),
        description="Detector threshold forwarded to yolo_ros and mirrored into scene grounding defaults.",
    )
    object_detection_input_image_topic_arg = DeclareLaunchArgument(
        "object_detection_input_image_topic",
        default_value=_profile_default(
            profile_defaults,
            "object_detection_input_image_topic",
            "/camera/front/image_raw",
        ),
        description="RGB image topic remapped into the external detector stack.",
    )
    object_detection_image_reliability_arg = DeclareLaunchArgument(
        "object_detection_image_reliability",
        default_value="2",
        description="Detector image QoS reliability, where 2 means Best Effort.",
    )
    scene_grounding_detector_topic_arg = DeclareLaunchArgument(
        "scene_grounding_detector_topic",
        default_value="/detected_objects",
        description="Detection topic consumed by nao_scene_grounding.",
    )
    scene_grounding_summary_topic_arg = DeclareLaunchArgument(
        "scene_grounding_summary_topic",
        default_value="/scene/summary",
        description="JSON summary topic published by nao_scene_grounding.",
    )
    scene_grounding_spatial_overlay_topic_arg = DeclareLaunchArgument(
        "scene_grounding_spatial_overlay_topic",
        default_value="",
        description=(
            "Optional JSON topic providing frame-qualified object positions keyed "
            "by grounded entity id."
        ),
    )
    planner_request_topic_arg = DeclareLaunchArgument(
        "planner_request_topic",
        default_value="/planner/request",
        description="Planner ingress topic consumed by planner_llm.",
    )
    chatbot_planner_request_topic_arg = DeclareLaunchArgument(
        "chatbot_planner_request_topic",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_planner_request_topic",
            "/planner/request",
        ),
        description=(
            "Planner request topic used by chatbot_llm. Set this to the orchestrator "
            "gate topic when enable_orchestrator_planner_gate is true."
        ),
    )
    enable_orchestrator_planner_gate_arg = DeclareLaunchArgument(
        "enable_orchestrator_planner_gate",
        default_value=_profile_default(
            profile_defaults,
            "enable_orchestrator_planner_gate",
            "false",
        ),
        description="Route chatbot planner requests through nao_orchestrator before planner_llm.",
    )
    orchestrator_planner_gate_topic_arg = DeclareLaunchArgument(
        "orchestrator_planner_gate_topic",
        default_value=_profile_default(
            profile_defaults,
            "orchestrator_planner_gate_topic",
            "/nao_orchestrator/planner_request",
        ),
        description="Orchestrator-owned planner admission topic.",
    )
    planner_request_intent_arg = DeclareLaunchArgument(
        "planner_request_intent",
        default_value="planner_request",
        description="Intent label used on planner ingress messages.",
    )
    planner_dialogue_act_topic_arg = DeclareLaunchArgument(
        "planner_dialogue_act_topic",
        default_value="/planner/dialogue_act",
        description="Planner-owned dialogue act topic published by planner_llm.",
    )
    planner_dialogue_relay_topic_arg = DeclareLaunchArgument(
        "planner_dialogue_relay_topic",
        default_value="/nao_orchestrator/planner_dialogue_act",
        description=(
            "Orchestrator-owned planner dialogue relay topic consumed by dialogue_manager."
        ),
    )
    planner_skill_registry_path_arg = DeclareLaunchArgument(
        "planner_skill_registry_path",
        default_value="",
        description="Optional absolute path to a planner_llm skill-registry JSON file.",
    )
    scan_result_mode_arg = DeclareLaunchArgument(
        "scan_result_mode",
        default_value=_profile_default(profile_defaults, "scan_result_mode", "success"),
        description="Deterministic result mode for the scan skill.",
    )
    scan_summary_arg = DeclareLaunchArgument(
        "scan_summary",
        default_value=_profile_default(
            profile_defaults,
            "scan_summary",
            "I looked around and can report the current scene summary.",
        ),
        description="Success summary returned by the scan skill.",
    )
    scan_report_after_success_arg = DeclareLaunchArgument(
        "scan_report_after_success",
        default_value=_profile_default(
            profile_defaults,
            "scan_report_after_success",
            "false",
        ),
        description=(
            "Let nao_orchestrator speak scan results directly. Keep false so "
            "completed task wording routes back through chatbot_llm."
        ),
    )
    scene_grounding_allowed_labels_arg = DeclareLaunchArgument(
        "scene_grounding_allowed_labels",
        default_value=_SCENE_GROUNDING_ALLOWED_LABELS,
        description="Comma-separated detector labels to ground into KnowledgeCore.",
    )
    scene_grounding_knowledge_lifespan_sec_arg = DeclareLaunchArgument(
        "scene_grounding_knowledge_lifespan_sec",
        default_value=_profile_default(
            profile_defaults,
            "scene_grounding_knowledge_lifespan_sec",
            "8.0",
        ),
        description="KnowledgeCore lifespan used for transient grounded object facts.",
    )
    scene_grounding_knowledge_refresh_interval_sec_arg = DeclareLaunchArgument(
        "scene_grounding_knowledge_refresh_interval_sec",
        default_value="1.0",
        description="Minimum interval between grounding refreshes for the same tracked object.",
    )
    scene_grounding_local_stale_after_sec_arg = DeclareLaunchArgument(
        "scene_grounding_local_stale_after_sec",
        default_value=_profile_default(
            profile_defaults,
            "scene_grounding_local_stale_after_sec",
            "10.0",
        ),
        description=(
            "How long nao_scene_grounding keeps a locally tracked object in "
            "scene summaries after the last detector observation."
        ),
    )
    scene_grounding_fallback_match_distance_px_arg = DeclareLaunchArgument(
        "scene_grounding_fallback_match_distance_px",
        default_value=_profile_default(
            profile_defaults,
            "scene_grounding_fallback_match_distance_px",
            "64.0",
        ),
        description=(
            "Maximum pixel distance used by nao_scene_grounding to reconcile "
            "tracker-less detections into one stable entity."
        ),
    )
    scene_grounding_fallback_match_max_age_sec_arg = DeclareLaunchArgument(
        "scene_grounding_fallback_match_max_age_sec",
        default_value=_profile_default(
            profile_defaults,
            "scene_grounding_fallback_match_max_age_sec",
            "2.0",
        ),
        description=(
            "Maximum age of a stale tracker-less detection that can still be "
            "matched to a new detection."
        ),
    )
    start_knowledge_core_arg = DeclareLaunchArgument(
        "start_knowledge_core",
        default_value=_profile_default(profile_defaults, "start_knowledge_core", "true"),
        description=(
            "Optionally launch KnowledgeCore for chatbot_llm grounding when it "
            "is installed in the environment."
        ),
    )
    start_dialogue_manager_arg = DeclareLaunchArgument(
        "start_dialogue_manager",
        default_value=_profile_default(profile_defaults, "start_dialogue_manager", "true"),
        description="Launch the upstream dialogue_manager lifecycle node.",
    )
    start_interaction_sim_arg = DeclareLaunchArgument(
        "start_interaction_sim",
        default_value=_profile_default(profile_defaults, "start_interaction_sim", "false"),
        description="Optionally launch the official interaction_sim support launch for simulator testing.",
    )
    start_interaction_sim_perception_arg = DeclareLaunchArgument(
        "start_interaction_sim_perception",
        default_value=_profile_default(
            profile_defaults,
            "start_interaction_sim_perception",
            "true",
        ),
        description="Launch the interaction_sim webcam/person/emotion perception components.",
    )
    start_interaction_sim_tools_arg = DeclareLaunchArgument(
        "start_interaction_sim_tools",
        default_value=_profile_default(
            profile_defaults,
            "start_interaction_sim_tools",
            "true",
        ),
        description="Launch interaction_sim support tools such as rosbridge and ui_server.",
    )
    start_interaction_sim_expressive_face_arg = DeclareLaunchArgument(
        "start_interaction_sim_expressive_face",
        default_value=_profile_default(
            profile_defaults,
            "start_interaction_sim_expressive_face",
            "false",
        ),
        description=(
            "Launch the simulator expressive_face node. Disable it when you only "
            "need webcam/HRI perception and want to avoid duplicate TTS action "
            "servers in the sim stack."
        ),
    )
    start_interaction_sim_ui_arg = DeclareLaunchArgument(
        "start_interaction_sim_ui",
        default_value=_profile_default(profile_defaults, "start_interaction_sim_ui", "false"),
        description="Start ui_server together with interaction_sim support tools.",
    )
    interaction_sim_hri_log_profile_arg = DeclareLaunchArgument(
        "interaction_sim_hri_log_profile",
        default_value=_profile_default(
            profile_defaults,
            "interaction_sim_hri_log_profile",
            "quiet",
        ),
        description=(
            "Verbosity profile for interaction_sim HRI perception nodes. "
            "Use quiet for demos and debug for full face/person/emotion logs."
        ),
    )
    interaction_sim_gscam_config_arg = DeclareLaunchArgument(
        "interaction_sim_gscam_config",
        default_value="v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert",
        description="GStreamer pipeline used by gscam when interaction_sim support is enabled.",
    )
    start_nao_orchestrator_arg = DeclareLaunchArgument(
        "start_nao_orchestrator",
        default_value=_profile_default(profile_defaults, "start_nao_orchestrator", "true"),
        description="Launch the NAO orchestrator scaffold.",
    )
    start_scan_skill_arg = DeclareLaunchArgument(
        "start_scan_skill",
        default_value=_profile_default(profile_defaults, "start_scan_skill", "true"),
        description="Launch the scan composite skill action server.",
    )
    start_report_result_skill_arg = DeclareLaunchArgument(
        "start_report_result_skill",
        default_value=_profile_default(profile_defaults, "start_report_result_skill", "true"),
        description="Launch the report_result action server.",
    )
    start_fake_skills_arg = DeclareLaunchArgument(
        "start_fake_skills",
        default_value=_profile_default(profile_defaults, "start_fake_skills", "true"),
        description="Launch deterministic fake skill action servers (/skill/fake/*).",
    )
    preloaded_environment_ids_arg = DeclareLaunchArgument(
        "preloaded_environment_ids",
        default_value=_profile_default(profile_defaults, "preloaded_environment_ids", ""),
        description=(
            "Comma-separated KnowledgeCore environment fixtures to preload at "
            "startup, for example baseline_table,kitchen_delivery. Empty disables "
            "the launch-time preload."
        ),
    )
    preloaded_environment_fixtures_path_arg = DeclareLaunchArgument(
        "preloaded_environment_fixtures_path",
        default_value=_profile_default(profile_defaults, "preloaded_environment_fixtures_path", ""),
        description=(
            "Optional absolute fixture JSON path. Empty uses the packaged "
            "nao_chatbot/config/preloaded_environments.json."
        ),
    )
    preloaded_environment_lifespan_sec_arg = DeclareLaunchArgument(
        "preloaded_environment_lifespan_sec",
        default_value=_profile_default(
            profile_defaults,
            "preloaded_environment_lifespan_sec",
            "1800.0",
        ),
        description="KnowledgeCore lifespan for launch-preloaded environment facts.",
    )
    preloaded_environment_kb_models_arg = DeclareLaunchArgument(
        "preloaded_environment_kb_models",
        default_value=_profile_default(profile_defaults, "preloaded_environment_kb_models", ""),
        description="Optional CSV KnowledgeCore model list for launch-preloaded fixtures.",
    )
    fake_skill_scenario_file_arg = DeclareLaunchArgument(
        "fake_skill_scenario_file",
        default_value=_profile_default(profile_defaults, "fake_skill_scenario_file", ""),
        description="Optional YAML file with fake skill default and named scenarios.",
    )
    fake_skill_active_scenario_id_arg = DeclareLaunchArgument(
        "fake_skill_active_scenario_id",
        default_value=_profile_default(profile_defaults, "fake_skill_active_scenario_id", ""),
        description="Optional named scenario id applied by default to fake skill requests.",
    )
    fake_skill_global_mode_arg = DeclareLaunchArgument(
        "fake_skill_global_mode",
        default_value=_profile_default(profile_defaults, "fake_skill_global_mode", "scenario"),
        description=(
            "Global fake-skill policy mode: scenario|always_success|always_fail|"
            "every_other|random_seeded."
        ),
    )
    fake_skill_random_failure_prob_arg = DeclareLaunchArgument(
        "fake_skill_random_failure_prob",
        default_value=_profile_default(profile_defaults, "fake_skill_random_failure_prob", "0.50"),
        description="Failure probability used by fake_skill_global_mode=random_seeded.",
    )
    fake_skill_mode_overrides_json_arg = DeclareLaunchArgument(
        "fake_skill_mode_overrides_json",
        default_value=_profile_default(profile_defaults, "fake_skill_mode_overrides_json", "{}"),
        description='JSON map for per-skill mode overrides, e.g. {"find_object":"always_fail"}.',
    )
    perform_motion_execution_mode_arg = DeclareLaunchArgument(
        "perform_motion_execution_mode",
        default_value=_profile_default(profile_defaults, "perform_motion_execution_mode", "real"),
        description="perform_motion dispatch mode: real by default; fake is an explicit validation opt-in.",
    )
    look_at_execution_mode_arg = DeclareLaunchArgument(
        "look_at_execution_mode",
        default_value=_profile_default(profile_defaults, "look_at_execution_mode", "fake"),
        description="look_at dispatch mode: fake by default; real is an explicit opt-in.",
    )
    start_nao_say_skill_arg = DeclareLaunchArgument(
        "start_nao_say_skill",
        default_value=_profile_default(profile_defaults, "start_nao_say_skill", "true"),
        description="Launch the dedicated NAO say skill.",
    )
    start_nao_replay_motion_arg = DeclareLaunchArgument(
        "start_nao_replay_motion",
        default_value=_profile_default(profile_defaults, "start_nao_replay_motion", "true"),
        description="Launch replay_motion and retained head-motion servers.",
    )
    head_motion_allow_open_loop_without_joint_state_arg = DeclareLaunchArgument(
        "head_motion_allow_open_loop_without_joint_state",
        default_value=_profile_default(
            profile_defaults,
            "head_motion_allow_open_loop_without_joint_state",
            "false",
        ),
        description="Allow absolute head-motion goals to publish even before a head JointState arrives.",
    )
    posture_allow_open_loop_without_naoqi_arg = DeclareLaunchArgument(
        "posture_allow_open_loop_without_naoqi",
        default_value=_profile_default(
            profile_defaults,
            "posture_allow_open_loop_without_naoqi",
            "false",
        ),
        description="Acknowledge posture goals in open loop when NAOqi is unavailable.",
    )
    head_motion_assume_success_on_convergence_timeout_arg = DeclareLaunchArgument(
        "head_motion_assume_success_on_convergence_timeout",
        default_value=_profile_default(
            profile_defaults,
            "head_motion_assume_success_on_convergence_timeout",
            "false",
        ),
        description="Treat a published head-motion command as successful when convergence is not observable.",
    )
    start_nao_look_at_arg = DeclareLaunchArgument(
        "start_nao_look_at",
        default_value=_profile_default(profile_defaults, "start_nao_look_at", "true"),
        description="Launch the NAO look_at skill.",
    )
    start_rqt_console_arg = DeclareLaunchArgument(
        "start_rqt_console",
        default_value=_profile_default(profile_defaults, "start_rqt_console", "true"),
        description=(
            "Launch a single remapped rqt shell; when interaction_sim is enabled "
            "it loads the nao_chatbot debug-ready simulator perspective."
        ),
    )
    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value=_profile_default(profile_defaults, "start_rqt_chat", "false"),
        description=(
            "Launch the dialogue UI helper window. In interaction_sim profiles this "
            "opens rqt_dialogues; outside sim it opens standalone rqt_chat."
        ),
    )
    start_robot_speech_debug_arg = DeclareLaunchArgument(
        "start_robot_speech_debug",
        default_value=_profile_default(profile_defaults, "start_robot_speech_debug", "true"),
        description="Launch a logger that mirrors robot speech into ROS logs.",
    )
    start_interaction_trace_viewer_arg = DeclareLaunchArgument(
        "start_interaction_trace_viewer",
        default_value=_profile_default(
            profile_defaults,
            "start_interaction_trace_viewer",
            "false",
        ),
        description="Launch the simple interaction trace viewer observability node.",
    )
    interaction_trace_compact_mode_arg = DeclareLaunchArgument(
        "interaction_trace_compact_mode",
        default_value=_profile_default(profile_defaults, "interaction_trace_compact_mode", "true"),
        description="Render trace events in compact terminal mode.",
    )
    interaction_trace_write_jsonl_arg = DeclareLaunchArgument(
        "interaction_trace_write_jsonl",
        default_value=_profile_default(profile_defaults, "interaction_trace_write_jsonl", "true"),
        description="Persist interaction trace events to JSONL.",
    )
    interaction_trace_jsonl_output_dir_arg = DeclareLaunchArgument(
        "interaction_trace_jsonl_output_dir",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_jsonl_output_dir",
            "~/.ros/nao_ros4hri_traces",
        ),
        description="Directory where interaction trace JSONL files are written.",
    )
    interaction_trace_write_html_on_shutdown_arg = DeclareLaunchArgument(
        "interaction_trace_write_html_on_shutdown",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_write_html_on_shutdown",
            "true",
        ),
        description="Render an interaction trace HTML report when the node stops.",
    )
    interaction_trace_html_output_dir_arg = DeclareLaunchArgument(
        "interaction_trace_html_output_dir",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_html_output_dir",
            "~/.ros/nao_ros4hri_trace_reports",
        ),
        description="Directory where interaction trace HTML reports are written.",
    )
    interaction_trace_include_raw_payloads_arg = DeclareLaunchArgument(
        "interaction_trace_include_raw_payloads",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_include_raw_payloads",
            "false",
        ),
        description="Keep raw payload text in interaction trace events.",
    )
    interaction_trace_max_payload_chars_arg = DeclareLaunchArgument(
        "interaction_trace_max_payload_chars",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_max_payload_chars",
            "4000",
        ),
        description="Maximum summary characters per interaction trace event.",
    )
    interaction_trace_include_channels_csv_arg = DeclareLaunchArgument(
        "interaction_trace_include_channels_csv",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_include_channels_csv",
            "",
        ),
        description=(
            "Optional CSV allowlist of trace channels (without leading slash), "
            "for example planner/request,planner/execution_feedback."
        ),
    )
    interaction_trace_exclude_channels_csv_arg = DeclareLaunchArgument(
        "interaction_trace_exclude_channels_csv",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_exclude_channels_csv",
            "",
        ),
        description="Optional CSV denylist of trace channels (without leading slash).",
    )
    interaction_trace_include_event_types_csv_arg = DeclareLaunchArgument(
        "interaction_trace_include_event_types_csv",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_include_event_types_csv",
            "",
        ),
        description=(
            "Optional CSV allowlist of trace event types, for example "
            "planner_request,execution_feedback,chatbot_turn_trace."
        ),
    )
    interaction_trace_exclude_event_types_csv_arg = DeclareLaunchArgument(
        "interaction_trace_exclude_event_types_csv",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_exclude_event_types_csv",
            "",
        ),
        description="Optional CSV denylist of trace event types.",
    )
    interaction_trace_enable_scene_summary_channel_arg = DeclareLaunchArgument(
        "interaction_trace_enable_scene_summary_channel",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_enable_scene_summary_channel",
            "false",
        ),
        description="Enable /scene/summary ingestion in trace viewer (off by default to reduce perception flood).",
    )
    interaction_trace_scene_summary_emit_on_change_only_arg = DeclareLaunchArgument(
        "interaction_trace_scene_summary_emit_on_change_only",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_scene_summary_emit_on_change_only",
            "true",
        ),
        description="Emit scene summary events only when object-label snapshot changes.",
    )
    interaction_trace_scene_summary_min_interval_sec_arg = DeclareLaunchArgument(
        "interaction_trace_scene_summary_min_interval_sec",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_scene_summary_min_interval_sec",
            "1.0",
        ),
        description="Minimum interval between emitted /scene/summary events.",
    )
    interaction_trace_rosout_node_allowlist_csv_arg = DeclareLaunchArgument(
        "interaction_trace_rosout_node_allowlist_csv",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_rosout_node_allowlist_csv",
            (
                "chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,"
                "report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,"
                "head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug"
            ),
        ),
        description="CSV allowlist for rosout nodes shown by interaction trace viewer.",
    )
    interaction_trace_rosout_min_level_arg = DeclareLaunchArgument(
        "interaction_trace_rosout_min_level",
        default_value=_profile_default(
            profile_defaults,
            "interaction_trace_rosout_min_level",
            "warn",
        ),
        description="Minimum rosout severity for interaction trace viewer (debug|info|warn|error|fatal).",
    )
    start_demo_log_window_arg = DeclareLaunchArgument(
        "start_demo_log_window",
        default_value=_profile_default(profile_defaults, "start_demo_log_window", "false"),
        description="Print a filtered /rosout stream for demo-relevant chatbot/planner/executor nodes.",
    )
    start_managed_ollama_arg = DeclareLaunchArgument(
        "start_managed_ollama",
        default_value=_profile_default(profile_defaults, "start_managed_ollama", "false"),
        description=(
            "Start launch-managed Ollama servers for split chatbot/planner endpoints. "
            "Existing servers on the same hosts are reused."
        ),
    )
    managed_chatbot_ollama_host_arg = DeclareLaunchArgument(
        "managed_chatbot_ollama_host",
        default_value=_profile_default(profile_defaults, "managed_chatbot_ollama_host", "127.0.0.1:11434"),
        description="OLLAMA_HOST used by the launch-managed chatbot Ollama server.",
    )
    managed_planner_ollama_host_arg = DeclareLaunchArgument(
        "managed_planner_ollama_host",
        default_value=_profile_default(profile_defaults, "managed_planner_ollama_host", "127.0.0.1:11435"),
        description="OLLAMA_HOST used by the launch-managed planner Ollama server.",
    )
    managed_ollama_startup_delay_sec_arg = DeclareLaunchArgument(
        "managed_ollama_startup_delay_sec",
        default_value=_profile_default(profile_defaults, "managed_ollama_startup_delay_sec", "3.0"),
        description="Delay planner_llm startup briefly after launching managed Ollama servers.",
    )
    demo_log_nodes_arg = DeclareLaunchArgument(
        "demo_log_nodes",
        default_value=_profile_default(
            profile_defaults,
            "demo_log_nodes",
            _DEMO_LOG_NODES,
        ),
        description="Comma-separated node allowlist for the filtered demo log window.",
    )
    demo_log_min_level_arg = DeclareLaunchArgument(
        "demo_log_min_level",
        default_value=_profile_default(profile_defaults, "demo_log_min_level", "info"),
        description="Minimum severity for the filtered demo log window: debug, info, warn, error, or fatal.",
    )
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Temporary posture bridge topic used during migration.",
    )
    posture_bridge_connect_on_startup_arg = DeclareLaunchArgument(
        "posture_bridge_connect_on_startup",
        default_value=_profile_default(
            profile_defaults, "posture_bridge_connect_on_startup", "true"
        ),
        description=(
            "Connect the temporary posture bridge to NAOqi during launch without "
            "commanding a posture."
        ),
    )
    posture_bridge_disable_autonomous_life_on_connect_arg = DeclareLaunchArgument(
        "posture_bridge_disable_autonomous_life_on_connect",
        default_value=_profile_default(
            profile_defaults,
            "posture_bridge_disable_autonomous_life_on_connect",
            "false",
        ),
        description=(
            "Explicitly disable ALAutonomousLife when the temporary posture bridge "
            "connects."
        ),
    )
    posture_bridge_wake_up_on_connect_arg = DeclareLaunchArgument(
        "posture_bridge_wake_up_on_connect",
        default_value=_profile_default(
            profile_defaults, "posture_bridge_wake_up_on_connect", "false"
        ),
        description=(
            "Explicitly call ALMotion.wakeUp when the temporary posture bridge "
            "connects."
        ),
    )
    debug_tts_action_name_arg = DeclareLaunchArgument(
        "debug_tts_action_name",
        default_value="/debug/say",
        description="Debug-only TTS action used for rqt_chat and operator monitoring.",
    )
    tts_backend_action_name_arg = DeclareLaunchArgument(
        "tts_backend_action_name",
        default_value=_profile_default(profile_defaults, "tts_backend_action_name", ""),
        description=(
            "Downstream robot TTS action for nao_say_skill. Keep empty to use "
            "the /speech topic fallback."
        ),
    )
    sim_use_laptop_tts_arg = DeclareLaunchArgument(
        "sim_use_laptop_tts",
        default_value=_profile_default(profile_defaults, "sim_use_laptop_tts", "false"),
        description=(
            "Simulator helper: when true, route nao_say_skill speech through "
            "debug_tts_action_name (typically /debug/say) so laptop-side TTS can "
            "play utterances. Robot/demo profiles keep robot speech defaults."
        ),
    )
    dialogue_manager_chatbot_arg = DeclareLaunchArgument(
        "dialogue_manager_chatbot",
        default_value="chatbot_llm",
        description="Dialogue-manager chatbot backend prefix.",
    )
    dialogue_manager_enable_default_chat_arg = DeclareLaunchArgument(
        "dialogue_manager_enable_default_chat",
        default_value="true",
        description="Start a default dialogue so user speech routes to chatbot_llm immediately.",
    )
    dialogue_manager_default_chat_role_arg = DeclareLaunchArgument(
        "dialogue_manager_default_chat_role",
        default_value="__default__",
        description="Role used for the default dialogue session.",
    )
    dialogue_manager_default_chat_configuration_arg = DeclareLaunchArgument(
        "dialogue_manager_default_chat_configuration",
        default_value="",
        description="Optional JSON configuration passed to the default dialogue session.",
    )
    dialogue_manager_say_action_arg = DeclareLaunchArgument(
        "dialogue_manager_say_action",
        default_value="/nao/say",
        description="Say action endpoint used by dialogue_manager for speech delivery.",
    )
    chat_input_tracked_topic_arg = DeclareLaunchArgument(
        "chat_input_tracked_topic",
        default_value=_profile_default(
            profile_defaults,
            "chat_input_tracked_topic",
            "/nao_chatbot/humans/voices/tracked",
        ),
        description=(
            "Voice tracking topic used by rqt_chat and dialogue_manager. "
            "Non-ASR profiles default to a private topic to avoid stray DDS ASR input."
        ),
    )
    chat_input_speech_topic_arg = DeclareLaunchArgument(
        "chat_input_speech_topic",
        default_value=_profile_default(
            profile_defaults,
            "chat_input_speech_topic",
            "/nao_chatbot/humans/voices/anonymous_speaker/speech",
        ),
        description="LiveSpeech topic used by rqt_chat and dialogue_manager.",
    )
    chat_input_is_speaking_topic_arg = DeclareLaunchArgument(
        "chat_input_is_speaking_topic",
        default_value=_profile_default(
            profile_defaults,
            "chat_input_is_speaking_topic",
            "/nao_chatbot/humans/voices/anonymous_speaker/is_speaking",
        ),
        description="is_speaking topic used by rqt_chat debug input.",
    )
    chatbot_model_arg = DeclareLaunchArgument(
        "chatbot_model",
        default_value=_profile_default(profile_defaults, "chatbot_model", DEFAULT_VLLM_MODEL),
        description="Preferred public model argument for chatbot_llm response generation.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value=_profile_default(profile_defaults, "ollama_model", ""),
        description=(
            "Backward-compatible alias for chatbot_model. Leave empty for vLLM/"
            "OpenAI-compatible profiles."
        ),
    )
    chatbot_think_arg = DeclareLaunchArgument(
        "chatbot_think",
        default_value=_profile_default(profile_defaults, "chatbot_think", "false"),
        description="Forward Ollama think=false/true for chatbot_llm response and intent calls.",
    )
    chatbot_temperature_arg = DeclareLaunchArgument(
        "chatbot_temperature",
        default_value=_profile_default(profile_defaults, "chatbot_temperature", "0.2"),
        description="Sampling temperature for chatbot response and intent requests.",
    )
    chatbot_top_p_arg = DeclareLaunchArgument(
        "chatbot_top_p",
        default_value=_profile_default(profile_defaults, "chatbot_top_p", "0.9"),
        description="Nucleus-sampling probability for chatbot requests.",
    )
    chatbot_top_k_arg = DeclareLaunchArgument(
        "chatbot_top_k",
        default_value=_profile_default(profile_defaults, "chatbot_top_k", "0"),
        description="Top-k sampling limit for chatbot requests; zero leaves it unrestricted.",
    )
    chatbot_min_p_arg = DeclareLaunchArgument(
        "chatbot_min_p",
        default_value=_profile_default(profile_defaults, "chatbot_min_p", "0.0"),
        description="Minimum-token probability threshold for chatbot requests.",
    )
    chatbot_presence_penalty_arg = DeclareLaunchArgument(
        "chatbot_presence_penalty",
        default_value=_profile_default(profile_defaults, "chatbot_presence_penalty", "0.0"),
        description="Presence penalty for chatbot requests.",
    )
    chatbot_repetition_penalty_arg = DeclareLaunchArgument(
        "chatbot_repetition_penalty",
        default_value=_profile_default(profile_defaults, "chatbot_repetition_penalty", "1.0"),
        description="Repetition penalty for chatbot requests.",
    )
    chatbot_response_max_tokens_arg = DeclareLaunchArgument(
        "chatbot_response_max_tokens",
        default_value=_profile_default(profile_defaults, "chatbot_response_max_tokens", "192"),
        description="Maximum response tokens requested from chatbot_llm Ollama calls.",
    )
    chatbot_intent_max_tokens_arg = DeclareLaunchArgument(
        "chatbot_intent_max_tokens",
        default_value=_profile_default(profile_defaults, "chatbot_intent_max_tokens", "256"),
        description="Maximum intent tokens requested from chatbot_llm Ollama calls.",
    )
    chatbot_intent_model_arg = DeclareLaunchArgument(
        "chatbot_intent_model",
        default_value="",
        description="Optional dedicated model used by chatbot_llm for intent extraction.",
    )
    chatbot_turn_pipeline_mode_arg = DeclareLaunchArgument(
        "chatbot_turn_pipeline_mode",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_turn_pipeline_mode",
            "response_first",
        ),
        description=(
            "Chatbot turn pipeline: response_first for the current path or "
            "intent_first for route-locked runtime-review ablations."
        ),
    )
    chatbot_grounded_context_digest_enabled_arg = DeclareLaunchArgument(
        "chatbot_grounded_context_digest_enabled",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_grounded_context_digest_enabled",
            "true",
        ),
        description=(
            "Enable the compact natural-language scene digest before the "
            "authoritative grounded_context JSON. Set false for JSON-only "
            "runtime-review ablations."
        ),
    )
    grounded_context_digest_enabled_arg = DeclareLaunchArgument(
        "grounded_context_digest_enabled",
        default_value=_profile_default(
            profile_defaults,
            "grounded_context_digest_enabled",
            "true",
        ),
        description=(
            "Compatibility alias for chatbot_grounded_context_digest_enabled. "
            "Either flag set to false disables the compact scene digest."
        ),
    )
    ollama_intent_model_arg = DeclareLaunchArgument(
        "ollama_intent_model",
        default_value="",
        description="Backward-compatible alias for chatbot_intent_model.",
    )
    chatbot_server_url_arg = DeclareLaunchArgument(
        "chatbot_server_url",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_server_url",
            DEFAULT_VLLM_CHAT_URL,
        ),
        description="Backend HTTP endpoint used by chatbot_llm.",
    )
    chatbot_request_timeout_sec_arg = DeclareLaunchArgument(
        "chatbot_request_timeout_sec",
        default_value=_profile_default(profile_defaults, "chatbot_request_timeout_sec", "20.0"),
        description="Normal chatbot_llm response timeout in seconds.",
    )
    chatbot_first_request_timeout_sec_arg = DeclareLaunchArgument(
        "chatbot_first_request_timeout_sec",
        default_value=_profile_default(profile_defaults, "chatbot_first_request_timeout_sec", "60.0"),
        description="Timeout for the first chatbot_llm response request in a dialogue.",
    )
    chatbot_intent_request_timeout_sec_arg = DeclareLaunchArgument(
        "chatbot_intent_request_timeout_sec",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_intent_request_timeout_sec",
            "10.0",
        ),
        description="Timeout for chatbot_llm intent extraction requests.",
    )
    chatbot_preflight_required_arg = DeclareLaunchArgument(
        "chatbot_preflight_required",
        default_value=_profile_default(profile_defaults, "chatbot_preflight_required", "false"),
        description="Fail chatbot_llm configuration when its LLM preflight cannot return valid JSON.",
    )
    chatbot_preflight_timeout_sec_arg = DeclareLaunchArgument(
        "chatbot_preflight_timeout_sec",
        default_value=_profile_default(profile_defaults, "chatbot_preflight_timeout_sec", "45.0"),
        description="Timeout for chatbot_llm warmup/preflight requests.",
    )
    chatbot_preflight_attempts_arg = DeclareLaunchArgument(
        "chatbot_preflight_attempts",
        default_value=_profile_default(profile_defaults, "chatbot_preflight_attempts", "1"),
        description="Number of chatbot_llm readiness attempts before startup proceeds or fails.",
    )
    chatbot_preflight_realistic_enabled_arg = DeclareLaunchArgument(
        "chatbot_preflight_realistic_enabled",
        default_value=_profile_default(profile_defaults, "chatbot_preflight_realistic_enabled", "false"),
        description="Run an extra demo-shaped chatbot_llm readiness prompt during preflight.",
    )
    chatbot_preflight_keepalive_interval_sec_arg = DeclareLaunchArgument(
        "chatbot_preflight_keepalive_interval_sec",
        default_value=_profile_default(
            profile_defaults,
            "chatbot_preflight_keepalive_interval_sec",
            "0.0",
        ),
        description="Optional interval for low-cost chatbot_llm LLM keepalive pings; 0 disables it.",
    )
    chatbot_planner_mode_enabled_arg = DeclareLaunchArgument(
        "chatbot_planner_mode_enabled",
        default_value=_profile_default(profile_defaults, "chatbot_planner_mode_enabled", "false"),
        description=(
            "Enable planner-mode handoff in chatbot_llm so execution-oriented "
            "turns publish to /planner/request."
        ),
    )
    planner_llm_provider_arg = DeclareLaunchArgument(
        "planner_llm_provider",
        default_value=_profile_default(profile_defaults, "planner_llm_provider", "openai_compatible"),
        description="Planner backend provider: ollama or openai-compatible.",
    )
    planner_llm_model_arg = DeclareLaunchArgument(
        "planner_llm_model",
        default_value=_profile_default(
            profile_defaults,
            "planner_llm_model",
            _profile_default(profile_defaults, "chatbot_model", DEFAULT_VLLM_MODEL),
        ),
        description="Planner model name used by planner_llm.",
    )
    planner_llm_base_url_arg = DeclareLaunchArgument(
        "planner_llm_base_url",
        default_value=_profile_default(profile_defaults, "planner_llm_base_url", DEFAULT_VLLM_BASE_URL),
        description="Planner backend base URL. For Ollama this is the server root, not /api/chat.",
    )
    planner_llm_api_key_env_arg = DeclareLaunchArgument(
        "planner_llm_api_key_env",
        default_value=_profile_default(profile_defaults, "planner_llm_api_key_env", "OPENAI_API_KEY"),
        description="Environment variable read by planner_llm for OpenAI/vLLM-compatible API keys.",
    )
    planner_llm_temperature_arg = DeclareLaunchArgument(
        "planner_llm_temperature",
        default_value=_profile_default(profile_defaults, "planner_llm_temperature", "0.1"),
        description="Temperature forwarded to planner_llm.",
    )
    planner_llm_top_p_arg = DeclareLaunchArgument(
        "planner_llm_top_p",
        default_value=_profile_default(profile_defaults, "planner_llm_top_p", "1.0"),
        description="Nucleus-sampling probability forwarded to planner_llm.",
    )
    planner_llm_top_k_arg = DeclareLaunchArgument(
        "planner_llm_top_k",
        default_value=_profile_default(profile_defaults, "planner_llm_top_k", "0"),
        description="Top-k sampling limit forwarded to planner_llm.",
    )
    planner_llm_min_p_arg = DeclareLaunchArgument(
        "planner_llm_min_p",
        default_value=_profile_default(profile_defaults, "planner_llm_min_p", "0.0"),
        description="Minimum-token probability threshold forwarded to planner_llm.",
    )
    planner_llm_presence_penalty_arg = DeclareLaunchArgument(
        "planner_llm_presence_penalty",
        default_value=_profile_default(profile_defaults, "planner_llm_presence_penalty", "0.0"),
        description="Presence penalty forwarded to planner_llm.",
    )
    planner_llm_repetition_penalty_arg = DeclareLaunchArgument(
        "planner_llm_repetition_penalty",
        default_value=_profile_default(profile_defaults, "planner_llm_repetition_penalty", "1.0"),
        description="Repetition penalty forwarded to planner_llm.",
    )
    planner_llm_max_tokens_arg = DeclareLaunchArgument(
        "planner_llm_max_tokens",
        default_value="800",
        description="Maximum output tokens requested from planner_llm backends.",
    )
    planner_llm_timeout_sec_arg = DeclareLaunchArgument(
        "planner_llm_timeout_sec",
        default_value=_profile_default(profile_defaults, "planner_llm_timeout_sec", "20.0"),
        description="Planner backend timeout in seconds.",
    )
    planner_llm_think_arg = DeclareLaunchArgument(
        "planner_llm_think",
        default_value=_profile_default(profile_defaults, "planner_llm_think", "false"),
        description="Forward Ollama think=false/true for planner_llm calls.",
    )
    planner_llm_preflight_required_arg = DeclareLaunchArgument(
        "planner_llm_preflight_required",
        default_value=_profile_default(profile_defaults, "planner_llm_preflight_required", "false"),
        description="Fail planner_llm startup when its model preflight cannot return valid JSON.",
    )
    planner_llm_preflight_timeout_sec_arg = DeclareLaunchArgument(
        "planner_llm_preflight_timeout_sec",
        default_value=_profile_default(profile_defaults, "planner_llm_preflight_timeout_sec", "45.0"),
        description="Timeout for planner_llm model warmup/preflight requests.",
    )
    planner_llm_preflight_attempts_arg = DeclareLaunchArgument(
        "planner_llm_preflight_attempts",
        default_value=_profile_default(profile_defaults, "planner_llm_preflight_attempts", "1"),
        description="Number of planner_llm readiness attempts before startup proceeds or fails.",
    )
    planner_llm_preflight_realistic_enabled_arg = DeclareLaunchArgument(
        "planner_llm_preflight_realistic_enabled",
        default_value=_profile_default(profile_defaults, "planner_llm_preflight_realistic_enabled", "false"),
        description="Run an extra demo-shaped planner_llm readiness prompt during preflight.",
    )
    planner_llm_default_retry_budget_arg = DeclareLaunchArgument(
        "planner_llm_default_retry_budget",
        default_value="1",
        description="Default retry budget added to initial planner-generated plans.",
    )
    planner_llm_auto_replan_arg = DeclareLaunchArgument(
        "planner_llm_auto_replan",
        default_value="true",
        description="Automatically trigger replanning when nao_orchestrator reports failed or invalid plans.",
    )
    start_asr_arg = DeclareLaunchArgument(
        "start_asr",
        default_value=_profile_default(profile_defaults, "start_asr", "false"),
        description="Opt into local Vosk ASR; disabled by default for all main profiles.",
    )
    asr_vosk_model_path_arg = DeclareLaunchArgument(
        "asr_vosk_model_path",
        default_value="/models/vosk-model-small-en-us-0.15",
        description="Absolute path to the Vosk model.",
    )
    asr_audio_capture_device_arg = DeclareLaunchArgument(
        "asr_audio_capture_device",
        default_value="",
        description="Optional audio device identifier passed to simple_audio_capture.",
    )
    asr_audio_capture_enabled_arg = DeclareLaunchArgument(
        "asr_audio_capture_enabled",
        default_value=_profile_default(
            profile_defaults,
            "asr_audio_capture_enabled",
            "false",
        ),
        description=(
            "Launch simple_audio_capture for ASR microphone input. This is "
            "disabled by default because it starts a GStreamer audio source."
        ),
    )
    asr_push_to_talk_enabled_arg = DeclareLaunchArgument(
        "asr_push_to_talk_enabled",
        default_value=_profile_default(
            profile_defaults,
            "asr_push_to_talk_enabled",
            "true",
        ),
        description="Require an explicit Bool gate before ASR listens.",
    )

    chatbot_llm_bundle = _make_lifecycle_bundle(
        package_name="chatbot_llm",
        executable="start_node",
        node_name="chatbot_llm",
        condition=IfCondition(LaunchConfiguration("start_chatbot_llm")),
        extra_parameters=[
            {
                "model": ParameterValue(
                    _prefer_first_non_empty("chatbot_model", "ollama_model"),
                    value_type=str,
                )
            },
            {
                "intent_model": ParameterValue(
                    _prefer_first_non_empty(
                        "ollama_intent_model",
                        "chatbot_intent_model",
                        "chatbot_model",
                        "ollama_model",
                    ),
                    value_type=str,
                )
            },
            {
                "server_url": ParameterValue(
                    LaunchConfiguration("chatbot_server_url"),
                    value_type=str,
                )
            },
            {
                "think": ParameterValue(
                    LaunchConfiguration("chatbot_think"),
                    value_type=bool,
                )
            },
            {
                "temperature": ParameterValue(
                    LaunchConfiguration("chatbot_temperature"), value_type=float
                )
            },
            {
                "top_p": ParameterValue(LaunchConfiguration("chatbot_top_p"), value_type=float)
            },
            {
                "top_k": ParameterValue(LaunchConfiguration("chatbot_top_k"), value_type=int)
            },
            {
                "min_p": ParameterValue(LaunchConfiguration("chatbot_min_p"), value_type=float)
            },
            {
                "presence_penalty": ParameterValue(
                    LaunchConfiguration("chatbot_presence_penalty"), value_type=float
                )
            },
            {
                "repetition_penalty": ParameterValue(
                    LaunchConfiguration("chatbot_repetition_penalty"), value_type=float
                )
            },
            {
                "request_timeout_sec": ParameterValue(
                    LaunchConfiguration("chatbot_request_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "first_request_timeout_sec": ParameterValue(
                    LaunchConfiguration("chatbot_first_request_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "intent_request_timeout_sec": ParameterValue(
                    LaunchConfiguration("chatbot_intent_request_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "response_max_tokens": ParameterValue(
                    LaunchConfiguration("chatbot_response_max_tokens"),
                    value_type=int,
                )
            },
            {
                "intent_max_tokens": ParameterValue(
                    LaunchConfiguration("chatbot_intent_max_tokens"),
                    value_type=int,
                )
            },
            {
                "planner_mode_enabled": ParameterValue(
                    LaunchConfiguration("chatbot_planner_mode_enabled"),
                    value_type=bool,
                )
            },
            {
                "turn_pipeline_mode": ParameterValue(
                    LaunchConfiguration("chatbot_turn_pipeline_mode"),
                    value_type=str,
                )
            },
            {
                "grounded_context_digest_enabled": ParameterValue(
                    PythonExpression(
                        [
                            '"',
                            LaunchConfiguration("chatbot_grounded_context_digest_enabled"),
                            '".lower() == "true" and "',
                            LaunchConfiguration("grounded_context_digest_enabled"),
                            '".lower() == "true"',
                        ]
                    ),
                    value_type=bool,
                )
            },
            {
                "preflight_required": ParameterValue(
                    LaunchConfiguration("chatbot_preflight_required"),
                    value_type=bool,
                )
            },
            {
                "preflight_timeout_sec": ParameterValue(
                    LaunchConfiguration("chatbot_preflight_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "preflight_attempts": ParameterValue(
                    LaunchConfiguration("chatbot_preflight_attempts"),
                    value_type=int,
                )
            },
            {
                "preflight_realistic_enabled": ParameterValue(
                    LaunchConfiguration("chatbot_preflight_realistic_enabled"),
                    value_type=bool,
                )
            },
            {
                "preflight_keepalive_interval_sec": ParameterValue(
                    LaunchConfiguration("chatbot_preflight_keepalive_interval_sec"),
                    value_type=float,
                )
            },
            {
                "planner_request_topic": ParameterValue(
                    LaunchConfiguration("chatbot_planner_request_topic"),
                    value_type=str,
                )
            },
            {
                "planner_request_intent": ParameterValue(
                    LaunchConfiguration("planner_request_intent"),
                    value_type=str,
                )
            },
        ],
    )

    dialogue_manager_bundle = _make_lifecycle_bundle(
        package_name="dialogue_manager",
        executable="start_manager",
        node_name="dialogue_manager",
        condition=IfCondition(LaunchConfiguration("start_dialogue_manager")),
        extra_parameters=[
            {
                "chatbot": ParameterValue(
                    LaunchConfiguration("dialogue_manager_chatbot"),
                    value_type=str,
                )
            },
            {
                "enable_default_chat": ParameterValue(
                    LaunchConfiguration("dialogue_manager_enable_default_chat"),
                    value_type=bool,
                )
            },
            {
                "default_chat_role": ParameterValue(
                    LaunchConfiguration("dialogue_manager_default_chat_role"),
                    value_type=str,
                )
            },
            {
                "default_chat_configuration": ParameterValue(
                    LaunchConfiguration("dialogue_manager_default_chat_configuration"),
                    value_type=str,
                )
            },
            {
                "say_action": ParameterValue(
                    LaunchConfiguration("dialogue_manager_say_action"),
                    value_type=str,
                )
            },
            {
                "planner_dialogue_act_topic": ParameterValue(
                    LaunchConfiguration("planner_dialogue_relay_topic"),
                    value_type=str,
                )
            },
        ],
        remappings=[
            ("/humans/voices/tracked", LaunchConfiguration("chat_input_tracked_topic")),
            (
                "/humans/voices/anonymous_speaker/speech",
                LaunchConfiguration("chat_input_speech_topic"),
            ),
            (
                "/humans/voices/anonymous_speaker/is_speaking",
                LaunchConfiguration("chat_input_is_speaking_topic"),
            ),
        ],
    )

    nao_orchestrator_bundle = _make_lifecycle_bundle(
        package_name="nao_orchestrator",
        executable="run_app",
        node_name="nao_orchestrator",
        condition=IfCondition(LaunchConfiguration("start_nao_orchestrator")),
        extra_parameters=[
            {
                "posture_command_topic": ParameterValue(
                    LaunchConfiguration("posture_command_topic"),
                    value_type=str,
                )
            },
            {
                "scan_result_mode": ParameterValue(
                    LaunchConfiguration("scan_result_mode"),
                    value_type=str,
                )
            },
            {
                "scan_report_after_success": ParameterValue(
                    LaunchConfiguration("scan_report_after_success"),
                    value_type=bool,
                )
            },
            {
                "perform_motion_execution_mode": ParameterValue(
                    LaunchConfiguration("perform_motion_execution_mode"),
                    value_type=str,
                )
            },
            {
                "look_at_execution_mode": ParameterValue(
                    LaunchConfiguration("look_at_execution_mode"),
                    value_type=str,
                )
            },
            {
                "enable_planner_gate": ParameterValue(
                    LaunchConfiguration("enable_orchestrator_planner_gate"),
                    value_type=bool,
                )
            },
            {
                "planner_gate_request_topic": ParameterValue(
                    LaunchConfiguration("orchestrator_planner_gate_topic"),
                    value_type=str,
                )
            },
            {
                "planner_request_topic": ParameterValue(
                    LaunchConfiguration("planner_request_topic"),
                    value_type=str,
                )
            },
            {
                "planner_dialogue_act_topic": ParameterValue(
                    LaunchConfiguration("planner_dialogue_act_topic"),
                    value_type=str,
                )
            },
            {
                "planner_dialogue_relay_topic": ParameterValue(
                    LaunchConfiguration("planner_dialogue_relay_topic"),
                    value_type=str,
                )
            },
        ],
    )
    scan_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_orchestrator",
        executable="run_scan_skill",
        node_name="scan_skill_server",
        condition=IfCondition(LaunchConfiguration("start_scan_skill")),
        extra_parameters=[
            {
                "scan_result_mode": ParameterValue(
                    LaunchConfiguration("scan_result_mode"),
                    value_type=str,
                )
            },
            {
                "scan_summary": ParameterValue(
                    LaunchConfiguration("scan_summary"),
                    value_type=str,
                )
            },
        ],
    )
    report_result_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_orchestrator",
        executable="run_report_result_skill",
        node_name="report_result_skill_server",
        condition=IfCondition(LaunchConfiguration("start_report_result_skill")),
    )

    nao_say_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_say_skill",
        executable="start_skill",
        node_name="nao_say_skill",
        condition=IfCondition(LaunchConfiguration("start_nao_say_skill")),
        extra_parameters=[
            {
                "debug_tts_action_name": ParameterValue(
                    LaunchConfiguration("debug_tts_action_name"),
                    value_type=str,
                )
            },
            {
                "tts_backend_action_name": ParameterValue(
                    _nao_say_backend_action_name(),
                    value_type=str,
                )
            },
        ],
    )

    nao_replay_motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nao_replay_motion"),
                    "launch",
                    "nao_replay_motion.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_nao_replay_motion")),
        launch_arguments={
            "nao_ip": LaunchConfiguration("nao_ip"),
            "nao_port": LaunchConfiguration("nao_port"),
            "posture_command_topic": LaunchConfiguration("posture_command_topic"),
            "posture_bridge_connect_on_startup": LaunchConfiguration(
                "posture_bridge_connect_on_startup"
            ),
            "posture_bridge_disable_autonomous_life_on_connect": LaunchConfiguration(
                "posture_bridge_disable_autonomous_life_on_connect"
            ),
            "posture_bridge_wake_up_on_connect": _effective_posture_bridge_wake_up_on_connect(),
            "head_motion_allow_open_loop_without_joint_state": LaunchConfiguration(
                "head_motion_allow_open_loop_without_joint_state"
            ),
            "posture_allow_open_loop_without_naoqi": LaunchConfiguration(
                "posture_allow_open_loop_without_naoqi"
            ),
            "head_motion_assume_success_on_convergence_timeout": LaunchConfiguration(
                "head_motion_assume_success_on_convergence_timeout"
            ),
        }.items(),
    )

    naoqi_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("naoqi_driver"), "launch", "naoqi_driver.launch.py"]
            )
        ),
        condition=_standalone_naoqi_driver_condition(),
        launch_arguments={
            "nao_ip": LaunchConfiguration("nao_ip"),
            "nao_port": LaunchConfiguration("nao_port"),
            "network_interface": LaunchConfiguration("network_interface"),
            "qi_listen_url": LaunchConfiguration("qi_listen_url"),
        }.items(),
    )
    nao_robot_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_naoqi_driver"),
                    '" == "true" and "',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_naoqi_driver and start_nao_robot were both requested. "
            "Skipping the standalone naoqi_driver launch because nao_robot "
            "already includes it."
        ),
    )
    robot_perception_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_nao_robot and interaction_sim perception are both enabled. "
            "This can launch overlapping perception nodes. Prefer "
            "start_nao_robot:=true start_interaction_sim:=true "
            "start_interaction_sim_perception:=false when validating the real "
            "robot camera with simulator tools only."
        ),
    )
    robot_tools_only_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" != "true" and "',
                    LaunchConfiguration("start_interaction_sim_tools"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_nao_robot is enabled together with interaction_sim tools-only mode. "
            "This is the intended path for combining the real robot camera/TF with "
            "simulator-side operator tools such as rosbridge, rqt_human_radar, and UI helpers."
        ),
    )
    driver_perception_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_naoqi_driver"),
                    '" == "true" and "',
                    LaunchConfiguration("start_nao_robot"),
                    '" != "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_naoqi_driver is enabled together with interaction_sim perception. "
            "This keeps simulator perception on /camera/image_raw while the real robot "
            "camera stays on /camera/front/image_raw. Prefer "
            "start_interaction_sim_perception:=false for robot-camera validation, "
            "or override object_detection_input_image_topic and "
            "hri_visualization_image_topic to /camera/front/image_raw."
        ),
    )
    laptop_tts_robot_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("sim_use_laptop_tts"),
                    '" == "true" and ("',
                    LaunchConfiguration("start_naoqi_driver"),
                    '" == "true" or "',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true")',
                ]
            )
        ),
        msg=(
            "sim_use_laptop_tts:=true was requested while a real robot path is active. "
            "Ignoring debug laptop TTS routing and keeping nao_say_skill on robot speech backend."
        ),
    )
    posture_wakeup_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("posture_bridge_wake_up_on_connect"),
                    '" != "true" and ("',
                    LaunchConfiguration("start_naoqi_driver"),
                    '" == "true" or "',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true")',
                ]
            )
        ),
        msg=(
            "posture_bridge_wake_up_on_connect was not explicitly enabled while a real robot "
            "driver path is active. Forcing wake-up on connect to keep posture/motion actions responsive."
        ),
    )
    object_detection_camera_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_object_detection"),
                    '" == "true" and "',
                    LaunchConfiguration("object_detection_input_image_topic"),
                    '" == "/camera/image_raw" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" != "true" and "',
                    LaunchConfiguration("start_nao_robot"),
                    '" != "true"',
                ]
            )
        ),
        msg=(
            "object_detection is listening on /camera/image_raw, but interaction_sim "
            "perception is disabled and no robot camera profile is active. GScam is "
            "started by start_interaction_sim_perception:=true, not by start_naoqi_driver."
        ),
    )
    preloaded_environment = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    [
                        _service_wait_script("/kb/revise", timeout_sec=45),
                        " && exec preload_environment --environment-ids '",
                        LaunchConfiguration("preloaded_environment_ids"),
                        "' --fixture-path '",
                        LaunchConfiguration("preloaded_environment_fixtures_path"),
                        "' --kb-lifespan-sec '",
                        LaunchConfiguration("preloaded_environment_lifespan_sec"),
                        "' --kb-models '",
                        LaunchConfiguration("preloaded_environment_kb_models"),
                        "'",
                    ],
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            '"',
                            LaunchConfiguration("preloaded_environment_ids"),
                            '" != ""',
                        ]
                    )
                ),
            )
        ],
    )

    nao_look_at_bundle = _make_lifecycle_bundle(
        package_name="nao_look_at",
        executable="start_skill",
        node_name="nao_look_at",
        condition=IfCondition(LaunchConfiguration("start_nao_look_at")),
    )
    nao_scene_grounding_node = Node(
        package="nao_scene_grounding",
        executable="start_node",
        name="nao_scene_grounding",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("nao_scene_grounding"), "config", "00-defaults.yml"]
            ),
            {
                "detector_backend": ParameterValue(
                    LaunchConfiguration("object_detection_backend"),
                    value_type=str,
                )
            },
            {
                "detector_topic": ParameterValue(
                    LaunchConfiguration("scene_grounding_detector_topic"),
                    value_type=str,
                )
            },
            {
                "summary_topic": ParameterValue(
                    LaunchConfiguration("scene_grounding_summary_topic"),
                    value_type=str,
                )
            },
            {
                "spatial_overlay_topic": ParameterValue(
                    LaunchConfiguration("scene_grounding_spatial_overlay_topic"),
                    value_type=str,
                )
            },
            {
                "min_detection_score": ParameterValue(
                    LaunchConfiguration("object_detection_threshold"),
                    value_type=float,
                )
            },
            {
                "allowed_labels": ParameterValue(
                    LaunchConfiguration("scene_grounding_allowed_labels"),
                    value_type=str,
                )
            },
            {
                "knowledge_enabled": ParameterValue(
                    LaunchConfiguration("start_knowledge_core"),
                    value_type=bool,
                )
            },
            {
                "knowledge_lifespan_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_knowledge_lifespan_sec"),
                    value_type=float,
                )
            },
            {
                "knowledge_refresh_interval_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_knowledge_refresh_interval_sec"),
                    value_type=float,
                )
            },
            {
                "fallback_match_distance_px": ParameterValue(
                    LaunchConfiguration("scene_grounding_fallback_match_distance_px"),
                    value_type=float,
                )
            },
            {
                "fallback_match_max_age_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_fallback_match_max_age_sec"),
                    value_type=float,
                )
            },
            {
                "local_stale_after_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_local_stale_after_sec"),
                    value_type=float,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_scene_grounding")),
    )
    planner_llm_node = Node(
        package="planner_llm",
        executable="start_node",
        name="planner_llm",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("planner_llm"), "config", "00-defaults.yml"]
            ),
            {
                "planner_request_topic": ParameterValue(
                    LaunchConfiguration("planner_request_topic"),
                    value_type=str,
                )
            },
            {
                "planner_dialogue_act_topic": ParameterValue(
                    LaunchConfiguration("planner_dialogue_act_topic"),
                    value_type=str,
                )
            },
            {
                "skill_registry_path": ParameterValue(
                    LaunchConfiguration("planner_skill_registry_path"),
                    value_type=str,
                )
            },
            {
                "provider": ParameterValue(
                    LaunchConfiguration("planner_llm_provider"),
                    value_type=str,
                )
            },
            {
                "model": ParameterValue(
                    LaunchConfiguration("planner_llm_model"),
                    value_type=str,
                )
            },
            {
                "base_url": ParameterValue(
                    LaunchConfiguration("planner_llm_base_url"),
                    value_type=str,
                )
            },
            {
                "api_key_env": ParameterValue(
                    LaunchConfiguration("planner_llm_api_key_env"),
                    value_type=str,
                )
            },
            {
                "temperature": ParameterValue(
                    LaunchConfiguration("planner_llm_temperature"),
                    value_type=float,
                )
            },
            {
                "top_p": ParameterValue(
                    LaunchConfiguration("planner_llm_top_p"), value_type=float
                )
            },
            {
                "top_k": ParameterValue(
                    LaunchConfiguration("planner_llm_top_k"), value_type=int
                )
            },
            {
                "min_p": ParameterValue(
                    LaunchConfiguration("planner_llm_min_p"), value_type=float
                )
            },
            {
                "presence_penalty": ParameterValue(
                    LaunchConfiguration("planner_llm_presence_penalty"), value_type=float
                )
            },
            {
                "repetition_penalty": ParameterValue(
                    LaunchConfiguration("planner_llm_repetition_penalty"), value_type=float
                )
            },
            {
                "max_tokens": ParameterValue(
                    LaunchConfiguration("planner_llm_max_tokens"),
                    value_type=int,
                )
            },
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("planner_llm_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "think": ParameterValue(
                    LaunchConfiguration("planner_llm_think"),
                    value_type=bool,
                )
            },
            {
                "preflight_required": ParameterValue(
                    LaunchConfiguration("planner_llm_preflight_required"),
                    value_type=bool,
                )
            },
            {
                "preflight_timeout_sec": ParameterValue(
                    LaunchConfiguration("planner_llm_preflight_timeout_sec"),
                    value_type=float,
                )
            },
            {
                "preflight_attempts": ParameterValue(
                    LaunchConfiguration("planner_llm_preflight_attempts"),
                    value_type=int,
                )
            },
            {
                "preflight_realistic_enabled": ParameterValue(
                    LaunchConfiguration("planner_llm_preflight_realistic_enabled"),
                    value_type=bool,
                )
            },
            {
                "default_retry_budget": ParameterValue(
                    LaunchConfiguration("planner_llm_default_retry_budget"),
                    value_type=int,
                )
            },
            {
                "auto_replan": ParameterValue(
                    LaunchConfiguration("planner_llm_auto_replan"),
                    value_type=bool,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_planner_llm")),
    )

    rqt_console = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_console"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" != "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                _RQT_CONTAINER_ENV_GUARD,
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "exec rqt --clear-config --ros-args -r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                " -r /humans/voices/tracked:=",
                LaunchConfiguration("chat_input_tracked_topic"),
                " -r /humans/voices/anonymous_speaker/speech:=",
                LaunchConfiguration("chat_input_speech_topic"),
                " -r /humans/voices/anonymous_speaker/is_speaking:=",
                LaunchConfiguration("chat_input_is_speaking_topic"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    interaction_sim_rqt = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_console"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                _RQT_CONTAINER_ENV_GUARD,
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif ! ros2 pkg prefix interaction_sim >/dev/null 2>&1; then "
                "echo 'interaction_sim is not installed in this environment'; "
                "elif ! ros2 pkg prefix nao_chatbot >/dev/null 2>&1; then "
                "echo 'nao_chatbot is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "nao_prefix=\"$(ros2 pkg prefix nao_chatbot)\"; "
                "perspective=\"$nao_prefix/share/nao_chatbot/config/interaction_sim_debug.perspective\"; "
                "if [ ! -f \"$perspective\" ]; then "
                "sim_prefix=\"$(ros2 pkg prefix interaction_sim)\"; "
                "perspective=\"$sim_prefix/share/interaction_sim/config/simulator.perspective\"; "
                "fi; "
                "exec rqt --clear-config --perspective-file \"$perspective\" --ros-args -r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                " -r /humans/voices/tracked:=",
                LaunchConfiguration("chat_input_tracked_topic"),
                " -r /humans/voices/anonymous_speaker/speech:=",
                LaunchConfiguration("chat_input_speech_topic"),
                " -r /humans/voices/anonymous_speaker/is_speaking:=",
                LaunchConfiguration("chat_input_is_speaking_topic"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    interaction_sim_rqt_dialogues = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_chat"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                _RQT_CONTAINER_ENV_GUARD,
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif ! python3 -c 'import importlib.util,sys; "
                "sys.exit(0 if importlib.util.find_spec(\"rqt_dialogues\") else 1)' "
                ">/dev/null 2>&1; then "
                "echo 'rqt_dialogues is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt_dialogues launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "exec rqt --clear-config --standalone rqt_dialogues.plugin.DialoguesPlugin; "
                "fi",
            ],
        ],
        output="screen",
    )
    rqt_chat = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_chat"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" != "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                _RQT_CONTAINER_ENV_GUARD,
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif ! python3 -c 'import importlib.util,sys; "
                "sys.exit(0 if importlib.util.find_spec(\"rqt_chat\") else 1)' >/dev/null 2>&1; then "
                "echo 'rqt_chat is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt_chat launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "exec rqt --clear-config --standalone rqt_chat.chat.ChatPlugin --ros-args "
                "-r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                " -r /humans/voices/tracked:=",
                LaunchConfiguration("chat_input_tracked_topic"),
                " -r /humans/voices/anonymous_speaker/speech:=",
                LaunchConfiguration("chat_input_speech_topic"),
                " -r /humans/voices/anonymous_speaker/is_speaking:=",
                LaunchConfiguration("chat_input_is_speaking_topic"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    robot_speech_debug = Node(
        package="nao_chatbot",
        executable="robot_speech_debug",
        name="robot_speech_debug",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_robot_speech_debug")),
    )
    interaction_trace_viewer_node = Node(
        package="interaction_trace_viewer",
        executable="trace_node",
        name="interaction_trace_viewer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "trace_viewer_enabled": True,
                "compact_mode": LaunchConfiguration("interaction_trace_compact_mode"),
                "write_jsonl": LaunchConfiguration("interaction_trace_write_jsonl"),
                "jsonl_output_dir": LaunchConfiguration("interaction_trace_jsonl_output_dir"),
                "write_html_on_shutdown": LaunchConfiguration("interaction_trace_write_html_on_shutdown"),
                "html_output_dir": LaunchConfiguration("interaction_trace_html_output_dir"),
                "include_raw_payloads": LaunchConfiguration("interaction_trace_include_raw_payloads"),
                "max_payload_chars": LaunchConfiguration("interaction_trace_max_payload_chars"),
                "include_channels_csv": LaunchConfiguration("interaction_trace_include_channels_csv"),
                "exclude_channels_csv": LaunchConfiguration("interaction_trace_exclude_channels_csv"),
                "include_event_types_csv": LaunchConfiguration("interaction_trace_include_event_types_csv"),
                "exclude_event_types_csv": LaunchConfiguration("interaction_trace_exclude_event_types_csv"),
                "enable_scene_summary_channel": LaunchConfiguration(
                    "interaction_trace_enable_scene_summary_channel"
                ),
                "scene_summary_emit_on_change_only": LaunchConfiguration(
                    "interaction_trace_scene_summary_emit_on_change_only"
                ),
                "scene_summary_min_interval_sec": LaunchConfiguration(
                    "interaction_trace_scene_summary_min_interval_sec"
                ),
                "rosout_node_allowlist_csv": LaunchConfiguration(
                    "interaction_trace_rosout_node_allowlist_csv"
                ),
                "rosout_min_level": LaunchConfiguration("interaction_trace_rosout_min_level"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_interaction_trace_viewer")),
    )
    demo_log_window = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nao_chatbot",
            "demo_rosout_filter",
            "--nodes",
            LaunchConfiguration("demo_log_nodes"),
            "--min-level",
            LaunchConfiguration("demo_log_min_level"),
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_demo_log_window")),
    )
    managed_chatbot_ollama = ExecuteProcess(
        cmd=["bash", "-lc", _managed_ollama_script()],
        additional_env={"OLLAMA_HOST": LaunchConfiguration("managed_chatbot_ollama_host")},
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_managed_ollama")),
    )
    managed_planner_ollama = ExecuteProcess(
        cmd=["bash", "-lc", _managed_ollama_script()],
        additional_env={"OLLAMA_HOST": LaunchConfiguration("managed_planner_ollama_host")},
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_managed_ollama")),
    )
    asr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nao_chatbot"),
                    "launch",
                    "nao_chatbot_asr_only.launch.py",
                ]
            )
        ),
        launch_arguments={
            "asr_vosk_model_path": LaunchConfiguration("asr_vosk_model_path"),
            "asr_audio_capture_device": LaunchConfiguration(
                "asr_audio_capture_device"
            ),
            "asr_audio_capture_enabled": LaunchConfiguration(
                "asr_audio_capture_enabled"
            ),
            "asr_push_to_talk_enabled": LaunchConfiguration(
                "asr_push_to_talk_enabled"
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_asr")),
    )

    dialogue_manager_node = dialogue_manager_bundle[0]
    start_chatbot_condition = IfCondition(LaunchConfiguration("start_chatbot_llm"))
    start_dialogue_without_chatbot_condition = _not_launching_chatbot_llm_condition()
    start_dialogue_after_chatbot_condition = IfCondition(
        PythonExpression(
            [
                '"',
                LaunchConfiguration("start_dialogue_manager"),
                '" == "true" and "',
                LaunchConfiguration("start_chatbot_llm"),
                '" == "true"',
            ]
        )
    )
    chatbot_llm_configure = _configure_lifecycle_node(
        chatbot_llm_bundle[0],
        condition=start_chatbot_condition,
    )
    chatbot_llm_activate = _activate_lifecycle_node_on_inactive(
        chatbot_llm_bundle[0],
        condition=start_chatbot_condition,
    )
    dialogue_manager_configure_immediate = _configure_lifecycle_node(
        dialogue_manager_node,
        condition=start_dialogue_without_chatbot_condition,
    )
    dialogue_manager_configure_after_chatbot = _configure_lifecycle_node_after_active(
        chatbot_llm_bundle[0],
        dialogue_manager_node,
        condition=start_dialogue_after_chatbot_condition,
    )
    dialogue_manager_activate = _activate_lifecycle_node_on_inactive(
        dialogue_manager_node,
        condition=IfCondition(LaunchConfiguration("start_dialogue_manager")),
    )
    nao_orchestrator_node = nao_orchestrator_bundle[0]
    start_nao_orchestrator_condition = IfCondition(
        LaunchConfiguration("start_nao_orchestrator")
    )
    (
        nao_orchestrator_configure,
        nao_orchestrator_activate,
    ) = _configure_and_activate_lifecycle_node(
        nao_orchestrator_node,
        condition=start_nao_orchestrator_condition,
    )
    scan_skill_node = scan_skill_bundle[0]
    start_scan_skill_condition = IfCondition(LaunchConfiguration("start_scan_skill"))
    scan_skill_configure, scan_skill_activate = _configure_and_activate_lifecycle_node(
        scan_skill_node,
        condition=start_scan_skill_condition,
    )
    report_result_skill_node = report_result_skill_bundle[0]
    start_report_result_skill_condition = IfCondition(
        LaunchConfiguration("start_report_result_skill")
    )
    (
        report_result_skill_configure,
        report_result_skill_activate,
    ) = _configure_and_activate_lifecycle_node(
        report_result_skill_node,
        condition=start_report_result_skill_condition,
    )
    nao_say_skill_node = nao_say_skill_bundle[0]
    start_nao_say_skill_condition = IfCondition(
        LaunchConfiguration("start_nao_say_skill")
    )
    nao_say_skill_configure, nao_say_skill_activate = _configure_and_activate_lifecycle_node(
        nao_say_skill_node,
        condition=start_nao_say_skill_condition,
    )
    nao_orchestrator_recovery = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", _lifecycle_recovery_script("nao_orchestrator")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_nao_orchestrator")),
            )
        ],
    )
    scan_skill_recovery = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", _lifecycle_recovery_script("scan_skill_server")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_scan_skill")),
            )
        ],
    )
    report_result_skill_recovery = TimerAction(
        period=23.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", _lifecycle_recovery_script("report_result_skill_server")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_report_result_skill")),
            )
        ],
    )
    nao_look_at_recovery = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", _lifecycle_recovery_script("nao_look_at")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_nao_look_at")),
            )
        ],
    )
    stack_ready_after_dialogue = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dialogue_manager_node,
            goal_state='active',
            entities=[
                LogInfo(
                    msg=[
                        "[STACK READY] dialogue path active | chatbot_llm=",
                        LaunchConfiguration("start_chatbot_llm"),
                        " planner_llm=",
                        LaunchConfiguration("start_planner_llm"),
                        " planner_mode=",
                        LaunchConfiguration("chatbot_planner_mode_enabled"),
                        " turn_pipeline=",
                        LaunchConfiguration("chatbot_turn_pipeline_mode"),
                        " dialogue_manager=/dialogue_manager",
                    ]
                )
            ],
            handle_once=True,
        ),
        condition=IfCondition(LaunchConfiguration("start_dialogue_manager")),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            start_nao_robot_arg,
            start_nao_robot_hri_visualization_arg,
            start_rviz_arg,
            hri_visualization_image_topic_arg,
            start_object_detection_arg,
            object_detection_backend_arg,
            start_scene_grounding_arg,
            start_planner_llm_arg,
            start_chatbot_llm_arg,
            start_knowledge_core_arg,
            start_dialogue_manager_arg,
            start_interaction_sim_arg,
            start_interaction_sim_perception_arg,
            start_interaction_sim_tools_arg,
            start_interaction_sim_expressive_face_arg,
            start_interaction_sim_ui_arg,
            interaction_sim_hri_log_profile_arg,
            start_nao_orchestrator_arg,
            start_scan_skill_arg,
            start_report_result_skill_arg,
            start_fake_skills_arg,
            preloaded_environment_ids_arg,
            preloaded_environment_fixtures_path_arg,
            preloaded_environment_lifespan_sec_arg,
            preloaded_environment_kb_models_arg,
            fake_skill_scenario_file_arg,
            fake_skill_active_scenario_id_arg,
            fake_skill_global_mode_arg,
            fake_skill_random_failure_prob_arg,
            fake_skill_mode_overrides_json_arg,
            perform_motion_execution_mode_arg,
            look_at_execution_mode_arg,
            start_nao_say_skill_arg,
            start_nao_replay_motion_arg,
            head_motion_allow_open_loop_without_joint_state_arg,
            posture_allow_open_loop_without_naoqi_arg,
            head_motion_assume_success_on_convergence_timeout_arg,
            start_nao_look_at_arg,
            start_rqt_console_arg,
            start_rqt_chat_arg,
            start_robot_speech_debug_arg,
            start_interaction_trace_viewer_arg,
            interaction_trace_compact_mode_arg,
            interaction_trace_write_jsonl_arg,
            interaction_trace_jsonl_output_dir_arg,
            interaction_trace_write_html_on_shutdown_arg,
            interaction_trace_html_output_dir_arg,
            interaction_trace_include_raw_payloads_arg,
            interaction_trace_max_payload_chars_arg,
            interaction_trace_include_channels_csv_arg,
            interaction_trace_exclude_channels_csv_arg,
            interaction_trace_include_event_types_csv_arg,
            interaction_trace_exclude_event_types_csv_arg,
            interaction_trace_enable_scene_summary_channel_arg,
            interaction_trace_scene_summary_emit_on_change_only_arg,
            interaction_trace_scene_summary_min_interval_sec_arg,
            scene_grounding_local_stale_after_sec_arg,
            interaction_trace_rosout_node_allowlist_csv_arg,
            interaction_trace_rosout_min_level_arg,
            start_demo_log_window_arg,
            start_managed_ollama_arg,
            managed_chatbot_ollama_host_arg,
            managed_planner_ollama_host_arg,
            managed_ollama_startup_delay_sec_arg,
            demo_log_nodes_arg,
            demo_log_min_level_arg,
            interaction_sim_gscam_config_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            posture_command_topic_arg,
            posture_bridge_connect_on_startup_arg,
            posture_bridge_disable_autonomous_life_on_connect_arg,
            posture_bridge_wake_up_on_connect_arg,
            debug_tts_action_name_arg,
            tts_backend_action_name_arg,
            sim_use_laptop_tts_arg,
            dialogue_manager_chatbot_arg,
            dialogue_manager_enable_default_chat_arg,
            dialogue_manager_default_chat_role_arg,
            dialogue_manager_default_chat_configuration_arg,
            dialogue_manager_say_action_arg,
            chat_input_tracked_topic_arg,
            chat_input_speech_topic_arg,
            chat_input_is_speaking_topic_arg,
            chatbot_model_arg,
            ollama_model_arg,
            chatbot_think_arg,
            chatbot_temperature_arg,
            chatbot_top_p_arg,
            chatbot_top_k_arg,
            chatbot_min_p_arg,
            chatbot_presence_penalty_arg,
            chatbot_repetition_penalty_arg,
            chatbot_response_max_tokens_arg,
            chatbot_intent_max_tokens_arg,
            chatbot_intent_model_arg,
            chatbot_turn_pipeline_mode_arg,
            chatbot_grounded_context_digest_enabled_arg,
            grounded_context_digest_enabled_arg,
            ollama_intent_model_arg,
            chatbot_server_url_arg,
            chatbot_request_timeout_sec_arg,
            chatbot_first_request_timeout_sec_arg,
            chatbot_intent_request_timeout_sec_arg,
            chatbot_preflight_required_arg,
            chatbot_preflight_timeout_sec_arg,
            chatbot_preflight_attempts_arg,
            chatbot_preflight_realistic_enabled_arg,
            chatbot_preflight_keepalive_interval_sec_arg,
            chatbot_planner_mode_enabled_arg,
            planner_request_topic_arg,
            chatbot_planner_request_topic_arg,
            enable_orchestrator_planner_gate_arg,
            orchestrator_planner_gate_topic_arg,
            planner_request_intent_arg,
            planner_dialogue_act_topic_arg,
            planner_dialogue_relay_topic_arg,
            planner_skill_registry_path_arg,
            scan_result_mode_arg,
            scan_summary_arg,
            scan_report_after_success_arg,
            planner_llm_provider_arg,
            planner_llm_model_arg,
            planner_llm_base_url_arg,
            planner_llm_api_key_env_arg,
            planner_llm_temperature_arg,
            planner_llm_top_p_arg,
            planner_llm_top_k_arg,
            planner_llm_min_p_arg,
            planner_llm_presence_penalty_arg,
            planner_llm_repetition_penalty_arg,
            planner_llm_max_tokens_arg,
            planner_llm_timeout_sec_arg,
            planner_llm_think_arg,
            planner_llm_preflight_required_arg,
            planner_llm_preflight_timeout_sec_arg,
            planner_llm_preflight_attempts_arg,
            planner_llm_preflight_realistic_enabled_arg,
            planner_llm_default_retry_budget_arg,
            planner_llm_auto_replan_arg,
            start_asr_arg,
            asr_vosk_model_path_arg,
            asr_audio_capture_device_arg,
            asr_audio_capture_enabled_arg,
            asr_push_to_talk_enabled_arg,
            object_detection_namespace_arg,
            object_detection_model_arg,
            object_detection_device_arg,
            object_detection_log_level_arg,
            object_detection_threshold_arg,
            object_detection_input_image_topic_arg,
            object_detection_image_reliability_arg,
            scene_grounding_detector_topic_arg,
            scene_grounding_summary_topic_arg,
            scene_grounding_spatial_overlay_topic_arg,
            scene_grounding_allowed_labels_arg,
            scene_grounding_knowledge_lifespan_sec_arg,
            scene_grounding_knowledge_refresh_interval_sec_arg,
            scene_grounding_fallback_match_distance_px_arg,
            scene_grounding_fallback_match_max_age_sec_arg,
            LogInfo(
                msg=[
                    "[STACK] nao_chatbot launch | chatbot_model=",
                    _prefer_first_non_empty("chatbot_model", "ollama_model"),
                    " planner_model=",
                    LaunchConfiguration("planner_llm_model"),
                    " planner_mode=",
                    LaunchConfiguration("chatbot_planner_mode_enabled"),
                    " chatbot_url=",
                    LaunchConfiguration("chatbot_server_url"),
                    " planner_url=",
                    LaunchConfiguration("planner_llm_base_url"),
                    " planner_gate=",
                    LaunchConfiguration("enable_orchestrator_planner_gate"),
                    " scan=enabled",
                ]
            ),
            LogInfo(
                msg=[
                    "[STACK] enabled nodes | chatbot_llm=",
                    LaunchConfiguration("start_chatbot_llm"),
                    " dialogue_manager=",
                    LaunchConfiguration("start_dialogue_manager"),
                    " planner_llm=",
                    LaunchConfiguration("start_planner_llm"),
                    " nao_orchestrator=",
                    LaunchConfiguration("start_nao_orchestrator"),
                    " scan_skill=",
                    LaunchConfiguration("start_scan_skill"),
                    " report_result_skill=",
                    LaunchConfiguration("start_report_result_skill"),
                    " fake_skills=",
                    LaunchConfiguration("start_fake_skills"),
                    " scene_grounding=",
                    LaunchConfiguration("start_scene_grounding"),
                    " preloaded_environment=",
                    LaunchConfiguration("preloaded_environment_ids"),
                    " object_detection=",
                    LaunchConfiguration("start_object_detection"),
                    " trace_viewer=",
                    LaunchConfiguration("start_interaction_trace_viewer"),
                ]
            ),
            LogInfo(
                msg=[
                    "[LLM PREFLIGHT] launch policy | chatbot_required=",
                    LaunchConfiguration("chatbot_preflight_required"),
                    " planner_required=",
                    LaunchConfiguration("planner_llm_preflight_required"),
                    " chatbot_timeout=",
                    LaunchConfiguration("chatbot_preflight_timeout_sec"),
                    " planner_timeout=",
                    LaunchConfiguration("planner_llm_preflight_timeout_sec"),
                ]
            ),
            LogInfo(
                msg=(
                    "[STACK] lifecycle sequencing | chatbot_llm configures before "
                    "dialogue_manager; planner_llm and executor seams start independently"
                )
            ),
            LogInfo(
                msg=[
                    "[STACK] preloaded environment viewer | file://",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nao_chatbot"),
                            "config",
                            "preloaded_environment_viewer.html",
                        ]
                    ),
                    " or run: ros2 run nao_chatbot preloaded_environment_viewer --open",
                ]
            ),
            naoqi_driver_launch,
            nao_robot_note,
            robot_perception_note,
            robot_tools_only_note,
            driver_perception_note,
            laptop_tts_robot_note,
            posture_wakeup_note,
            object_detection_camera_note,
            preloaded_environment,
            rqt_console,
            interaction_sim_rqt,
            interaction_sim_rqt_dialogues,
            rqt_chat,
            robot_speech_debug,
            interaction_trace_viewer_node,
            demo_log_window,
            managed_chatbot_ollama,
            managed_planner_ollama,
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "knowledge_core",
                    "launch_file_name": "knowledge_core.launch.py",
                    "launch_arg_name": "start_knowledge_core",
                },
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "nao_robot",
                    "launch_file_name": "nao_robot.launch.py",
                    "launch_arg_name": "start_nao_robot",
                    "display_name": "nao_robot",
                    "launch_arguments": {
                        "nao_ip": LaunchConfiguration("nao_ip"),
                        "nao_port": LaunchConfiguration("nao_port"),
                    },
                    "required_packages": ["nao_robot"],
                },
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "hri_person_manager",
                    "launch_file_name": "person_manager.launch.py",
                    "launch_arg_name": "start_nao_robot",
                    "display_name": "hri_person_manager",
                    "launch_arguments": {
                        "reference_frame": "CameraTop_optical_frame",
                        "robot_reference_frame": "base_link",
                    },
                    "required_packages": ["hri_person_manager"],
                },
            ),
            OpaqueFunction(
                function=_optional_hri_visualization_launch,
            ),
            OpaqueFunction(
                function=_optional_rviz_launch,
                kwargs={
                    "launch_arg_name": "start_rviz",
                    "package_name": "nao_chatbot",
                    "config_relative_path": os.path.join("config", "nao_robot_safe.rviz"),
                    "display_name": "nao_chatbot rviz",
                },
            ),
            OpaqueFunction(
                function=_optional_object_detection_launch,
            ),
            OpaqueFunction(function=build_interaction_sim_actions),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "fake_skills",
                    "launch_file_name": "fake_skills.launch.py",
                    "launch_arg_name": "start_fake_skills",
                    "display_name": "fake_skills",
                    "required_packages": ["fake_skills"],
                    "launch_arguments": {
                        "fake_skill_scenario_file": LaunchConfiguration("fake_skill_scenario_file"),
                        "fake_skill_active_scenario_id": LaunchConfiguration("fake_skill_active_scenario_id"),
                        "fake_skill_global_mode": LaunchConfiguration("fake_skill_global_mode"),
                        "fake_skill_random_failure_prob": LaunchConfiguration(
                            "fake_skill_random_failure_prob"
                        ),
                        "fake_skill_mode_overrides_json": LaunchConfiguration(
                            "fake_skill_mode_overrides_json"
                        ),
                        "fake_skill_perform_motion_action": "/skill/fake/perform_motion",
                    },
                },
            ),
            chatbot_llm_bundle[0],
            chatbot_llm_configure,
            chatbot_llm_activate,
            dialogue_manager_node,
            dialogue_manager_configure_immediate,
            dialogue_manager_configure_after_chatbot,
            dialogue_manager_activate,
            nao_orchestrator_configure,
            nao_orchestrator_activate,
            scan_skill_configure,
            scan_skill_activate,
            report_result_skill_configure,
            report_result_skill_activate,
            nao_say_skill_configure,
            nao_say_skill_activate,
            nao_orchestrator_recovery,
            scan_skill_recovery,
            report_result_skill_recovery,
            nao_look_at_recovery,
            stack_ready_after_dialogue,
            *scan_skill_bundle,
            *report_result_skill_bundle,
            *nao_orchestrator_bundle,
            *nao_say_skill_bundle,
            nao_replay_motion_launch,
            *nao_look_at_bundle,
            nao_scene_grounding_node,
            TimerAction(
                period=LaunchConfiguration("managed_ollama_startup_delay_sec"),
                actions=[planner_llm_node],
            ),
            asr_launch,
        ]
    )
