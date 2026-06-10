"""
Helpers for composing the official interaction_sim layers into our stack.

The public launch surface stays small (`nao_chatbot_sim*.launch.py`), while the
optional simulator perception/tools wiring lives here as reusable actions.
"""

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


_PERCEPTION_PACKAGES = (
    "diagnostic_aggregator",
    "interaction_sim",
    "gscam",
    "hri_emotion_recognizer",
    "hri_face_detect_yunet",
    "hri_person_manager",
    "hri_visualization",
    "image_transport_plugins",
)

_TOOLS_PACKAGES = (
    "interaction_sim",
    "rosbridge_server",
    "ui_server",
)
_HRI_LOG_LEVELS_BY_PROFILE = {
    "quiet": "warn",
    "debug": "info",
}


# -----------------------------------------------------------------------------
# Small launch-time helpers
# -----------------------------------------------------------------------------


def _as_bool(context, name: str) -> bool:
    """Read a launch argument as a normalized boolean string."""
    return str(LaunchConfiguration(name).perform(context)).strip().lower() == "true"


def _required_packages(
    *,
    start_perception: bool,
    start_tools: bool,
    start_expressive_face: bool,
) -> list[str]:
    required_packages: list[str] = []
    if start_perception:
        required_packages.extend(_PERCEPTION_PACKAGES)
        if start_expressive_face:
            required_packages.append("expressive_face")
    if start_tools:
        required_packages.extend(_TOOLS_PACKAGES)
    return required_packages


def _missing_packages(package_names: list[str]) -> list[str]:
    missing_packages = []
    for package_name in package_names:
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            missing_packages.append(package_name)
    return sorted(set(missing_packages))


def _include_python_launch(
    package_name: str,
    launch_file_name: str,
    *,
    launch_arguments: dict[str, str] | None = None,
):
    source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            "launch",
            launch_file_name,
        )
    )
    if launch_arguments is None:
        return IncludeLaunchDescription(source)
    return IncludeLaunchDescription(
        source,
        launch_arguments=launch_arguments.items(),
    )


def _planner_friendly_hri_log_level(context) -> str:
    profile = str(
        LaunchConfiguration("interaction_sim_hri_log_profile").perform(context)
    ).strip().lower()
    return _HRI_LOG_LEVELS_BY_PROFILE.get(profile, "warn")


def _build_hri_lifecycle_actions(
    *,
    package_name: str,
    node_name: str,
    executable: str,
    log_level: str,
    shutdown_on_exit: bool = True,
    parameter_overrides: dict[str, object] | None = None,
) -> list:
    temp_ld = LaunchDescription()
    config = get_pal_configuration(pkg=package_name, node=node_name, ld=temp_ld)
    parameters = list(config["parameters"])
    if parameter_overrides:
        parameters.append(dict(parameter_overrides))
    arguments = list(config["arguments"])
    if log_level:
        arguments.extend(["--ros-args", "--log-level", log_level])

    lifecycle_kwargs = {
        "package": package_name,
        "executable": executable,
        "namespace": "",
        "name": node_name,
        "parameters": parameters,
        "remappings": config["remappings"],
        "arguments": arguments,
        "output": "both",
        "emulate_tty": True,
    }
    if shutdown_on_exit:
        lifecycle_kwargs["on_exit"] = Shutdown()

    node = LifecycleNode(**lifecycle_kwargs)
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
            handle_once=True,
        )
    )
    analyzer = Node(
        package="diagnostic_aggregator",
        executable="add_analyzer",
        namespace=package_name,
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(
                get_package_share_directory(package_name),
                "config",
                f"{package_name}_analyzers.yaml",
            )
        ],
    )
    return [node, configure_event, activate_event, analyzer]


def _interaction_sim_mode_description(
    *,
    start_perception: bool,
    start_tools: bool,
) -> str:
    if start_perception and start_tools:
        return "perception + tools"
    if start_perception:
        return "perception-only"
    return "tools-only"


def _interaction_sim_summary_logs(
    *,
    mode_description: str,
    start_expressive_face: bool,
    start_nao_say_skill: bool,
    hri_log_profile: str,
) -> list[LogInfo]:
    logs = [
        # These log lines give operators a quick summary of which simulator
        # layer is active without reading the full launch file.
        LogInfo(
            msg=(
                "interaction_sim %s layer enabled. "
                "This keeps simulator utilities separate from "
                "chatbot_llm, dialogue_manager, and knowledge_core."
            )
            % mode_description
        ),
        LogInfo(
            msg=(
                "The nao_chatbot sim profile loads a debug-ready rqt "
                "perspective with /debug/object_detection prewired into "
                "the spare image view."
            )
        ),
        LogInfo(
            msg=(
                "interaction_sim expressive_face is %s."
                % ("enabled" if start_expressive_face else "disabled")
            )
        ),
        LogInfo(
            msg=(
                "interaction_sim HRI perception log profile is %s."
                % hri_log_profile
            )
        ),
    ]
    if start_expressive_face and start_nao_say_skill:
        logs.append(
            LogInfo(
                msg=(
                    "interaction_sim expressive_face and nao_say_skill are both enabled. "
                    "If the simulator face exports /tts_engine/tts you may see duplicate "
                    "TTS action-server warnings."
                )
            )
        )
    return logs


def build_interaction_sim_actions(context):
    """Return the optional interaction_sim actions for the current profile."""
    if not _as_bool(context, "start_interaction_sim"):
        return []

    start_perception = _as_bool(context, "start_interaction_sim_perception")
    start_tools = _as_bool(context, "start_interaction_sim_tools")
    start_expressive_face = _as_bool(context, "start_interaction_sim_expressive_face")
    hri_log_profile = str(
        LaunchConfiguration("interaction_sim_hri_log_profile").perform(context)
    ).strip().lower() or "quiet"
    hri_log_level = _planner_friendly_hri_log_level(context)
    if not start_perception and not start_tools:
        return [
            LogInfo(
                msg=(
                    "interaction_sim bring-up skipped because both "
                    "start_interaction_sim_perception and "
                    "start_interaction_sim_tools are disabled."
                )
            )
        ]

    required_packages = _required_packages(
        start_perception=start_perception,
        start_tools=start_tools,
        start_expressive_face=start_expressive_face,
    )
    missing_packages = _missing_packages(required_packages)

    if missing_packages:
        return [
            LogInfo(
                msg=(
                    "interaction_sim bring-up skipped because the following official "
                    f"packages are missing: {', '.join(missing_packages)}"
                )
            )
        ]

    scoped_actions = []

    if start_perception:
        scoped_actions.extend(
            [
                SetRemap(src="image", dst="/camera/image_raw"),
                SetRemap(src="camera_info", dst="/camera/camera_info"),
            ]
        )
        if start_expressive_face:
            scoped_actions.extend(
                [
                    SetRemap(
                        src="/expressive_face/tts",
                        dst=LaunchConfiguration("debug_tts_action_name"),
                    ),
                    _include_python_launch(
                        "expressive_face",
                        "expressive_face.launch.py",
                        launch_arguments={"headless": "true"},
                    ),
                ]
            )
        scoped_actions.extend(
            [
                *_build_hri_lifecycle_actions(
                    package_name="hri_person_manager",
                    node_name="hri_person_manager",
                    executable="hri_person_manager",
                    log_level=hri_log_level,
                    shutdown_on_exit=True,
                    parameter_overrides={
                        "reference_frame": "base_link",
                        "robot_reference_frame": "base_link",
                    },
                ),
                *_build_hri_lifecycle_actions(
                    package_name="hri_face_detect_yunet",
                    node_name="hri_face_detect_yunet",
                    executable="hri_face_detect_yunet",
                    log_level=hri_log_level,
                    shutdown_on_exit=True,
                ),
                *_build_hri_lifecycle_actions(
                    package_name="hri_emotion_recognizer",
                    node_name="hri_emotion_recognizer",
                    executable="hri_emotion_recognizer",
                    log_level=hri_log_level,
                    shutdown_on_exit=False,
                ),
                Node(
                    package="gscam",
                    executable="gscam_node",
                    parameters=[
                        {
                            "gscam_config": LaunchConfiguration(
                                "interaction_sim_gscam_config"
                            ),
                            "use_sensor_data_qos": True,
                            "camera_name": "camera",
                            "camera_info_url": "package://interaction_sim/config/camera_info.yaml",
                            "frame_id": "camera",
                        }
                    ],
                    output="screen",
                ),
                _include_python_launch(
                    "hri_visualization",
                    "hri_visualization.launch.py",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        "0",
                        "0",
                        "0.1",
                        "-0.5",
                        "0.5",
                        "-0.5",
                        "0.5",
                        "sellion_link",
                        "camera",
                    ],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        "0",
                        "0",
                        "0.20",
                        "0",
                        "0",
                        "0",
                        "1",
                        "base_link",
                        "sellion_link",
                    ],
                    output="screen",
                ),
            ]
        )

    robot_driver_active = _as_bool(context, "start_nao_robot") or _as_bool(
        context, "start_naoqi_driver"
    )

    if start_tools:
        rosbridge_actions = []
        if robot_driver_active:
            rosbridge_actions.append(
                LogInfo(
                    msg=(
                        "interaction_sim tools detected a robot driver path "
                        "(start_nao_robot/start_naoqi_driver). Skipping the "
                        "extra rosbridge_server launch to avoid websocket-port conflicts."
                    )
                )
            )
        else:
            rosbridge_actions.append(
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("rosbridge_server"),
                                    "launch",
                                    "rosbridge_websocket_launch.xml",
                                ]
                            )
                        ]
                    )
                )
            )
        scoped_actions.extend(
            [
                Node(
                    package="ui_server",
                    executable="ui_server",
                    condition=IfCondition(
                        LaunchConfiguration("start_interaction_sim_ui")
                    ),
                    output="screen",
                ),
                *rosbridge_actions,
            ]
        )

    scoped_actions.extend(
        _interaction_sim_summary_logs(
            mode_description=_interaction_sim_mode_description(
                start_perception=start_perception,
                start_tools=start_tools,
            ),
            start_expressive_face=start_expressive_face,
            start_nao_say_skill=_as_bool(context, "start_nao_say_skill"),
            hri_log_profile=hri_log_profile,
        )
    )

    return [GroupAction(scoped=True, actions=scoped_actions)]
