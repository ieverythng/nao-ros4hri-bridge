import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


_REQUIRED_PACKAGES = (
    "interaction_sim",
    "expressive_face",
    "gscam",
    "hri_emotion_recognizer",
    "hri_face_detect_yunet",
    "hri_person_manager",
    "hri_visualization",
    "image_transport_plugins",
    "rosbridge_server",
    "rqt_chat",
    "rqt_human_radar",
    "rqt_image_view",
    "rqt_reconfigure",
    "ui_server",
)


def _generate_interaction_sim_actions(context):
    missing_packages = []
    for package_name in _REQUIRED_PACKAGES:
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            missing_packages.append(package_name)

    if missing_packages:
        return [
            LogInfo(
                msg=(
                    "interaction_sim bring-up skipped because the following official "
                    f"packages are missing: {', '.join(missing_packages)}"
                )
            )
        ]

    interaction_sim_share = get_package_share_directory("interaction_sim")

    return [
        GroupAction(
            scoped=True,
            actions=[
                SetRemap(src="image", dst="/camera/image_raw"),
                SetRemap(src="camera_info", dst="/camera/camera_info"),
                SetRemap(
                    src="/expressive_face/tts",
                    dst=LaunchConfiguration("debug_tts_action_name"),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("expressive_face"),
                            "launch",
                            "expressive_face.launch.py",
                        )
                    ),
                    launch_arguments={"headless": "true"}.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("hri_person_manager"),
                            "launch",
                            "person_manager.launch.py",
                        )
                    ),
                    launch_arguments={
                        "reference_frame": "camera",
                        "robot_reference_frame": "sellion_link",
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("hri_face_detect_yunet"),
                            "launch",
                            "hri_face_detect_yunet.launch.py",
                        )
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("hri_emotion_recognizer"),
                            "launch",
                            "emotion_recognizer.launch.py",
                        )
                    )
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
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("hri_visualization"),
                            "launch",
                            "hri_visualization.launch.py",
                        )
                    )
                ),
                Node(
                    package="ui_server",
                    executable="ui_server",
                    condition=IfCondition(
                        LaunchConfiguration("start_interaction_sim_ui")
                    ),
                    output="screen",
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
                ),
                LogInfo(
                    msg=(
                        "interaction_sim perception/UI layer enabled. "
                        "This launches the official simulator-side webcam, "
                        "face/person/emotion, visualization, expressive_face, "
                        "and rosbridge components without duplicating "
                        "chatbot_llm, dialogue_manager, or knowledge_core."
                    )
                ),
                LogInfo(
                    msg=(
                        "The interaction_sim perspective is available at "
                        f"{os.path.join(interaction_sim_share, 'config', 'simulator.perspective')}"
                    )
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug_tts_action_name",
                default_value="/debug/say",
                description="Debug-only TTS action used for operator-facing simulator tools.",
            ),
            DeclareLaunchArgument(
                "interaction_sim_gscam_config",
                default_value="v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert",
                description="GStreamer pipeline used by gscam for webcam-driven simulator tests.",
            ),
            DeclareLaunchArgument(
                "start_interaction_sim_ui",
                default_value="false",
                description="Start ui_server together with the interaction_sim perception stack.",
            ),
            OpaqueFunction(function=_generate_interaction_sim_actions),
        ]
    )
