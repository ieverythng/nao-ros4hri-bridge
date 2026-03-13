from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _make_lifecycle_bundle(
    *,
    package_name,
    executable,
    node_name,
    condition,
    extra_parameters=None,
    configure_delay_sec=1.0,
    activate_delay_sec=2.5,
):
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
        output="both",
        emulate_tty=True,
        condition=condition,
    )
    configure = TimerAction(
        period=configure_delay_sec,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
        condition=condition,
    )
    activate = TimerAction(
        period=activate_delay_sec,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )
        ],
        condition=condition,
    )
    return [node, configure, activate]


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="false",
        description="Optionally launch naoqi_driver alongside the migrated ROS4HRI stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="172.26.112.62",
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
        default_value="true",
        description="Launch the upstream-aligned chatbot_llm backend.",
    )
    start_dialogue_manager_arg = DeclareLaunchArgument(
        "start_dialogue_manager",
        default_value="true",
        description="Launch the upstream dialogue_manager lifecycle node.",
    )
    start_nao_orchestrator_arg = DeclareLaunchArgument(
        "start_nao_orchestrator",
        default_value="true",
        description="Launch the NAO orchestrator scaffold.",
    )
    start_nao_say_skill_arg = DeclareLaunchArgument(
        "start_nao_say_skill",
        default_value="true",
        description="Launch the dedicated NAO say skill.",
    )
    start_nao_replay_motion_arg = DeclareLaunchArgument(
        "start_nao_replay_motion",
        default_value="true",
        description="Launch replay_motion and retained head-motion servers.",
    )
    start_nao_look_at_arg = DeclareLaunchArgument(
        "start_nao_look_at",
        default_value="true",
        description="Launch the scaffolded look_at skill.",
    )
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Temporary posture bridge topic used during migration.",
    )
    dialogue_manager_chatbot_arg = DeclareLaunchArgument(
        "dialogue_manager_chatbot",
        default_value="chatbot_llm",
        description="Dialogue-manager chatbot backend prefix.",
    )

    chatbot_llm_bundle = _make_lifecycle_bundle(
        package_name="chatbot_llm",
        executable="start_node",
        node_name="chatbot_llm",
        condition=IfCondition(LaunchConfiguration("start_chatbot_llm")),
        configure_delay_sec=1.5,
        activate_delay_sec=3.0,
    )

    dialogue_manager_bundle = _make_lifecycle_bundle(
        package_name="dialogue_manager",
        executable="start_manager",
        node_name="dialogue_manager",
        condition=IfCondition(LaunchConfiguration("start_dialogue_manager")),
        extra_parameters=[
            {"chatbot": LaunchConfiguration("dialogue_manager_chatbot")}
        ],
        configure_delay_sec=3.0,
        activate_delay_sec=5.5,
    )

    nao_orchestrator_bundle = _make_lifecycle_bundle(
        package_name="nao_orchestrator",
        executable="run_app",
        node_name="nao_orchestrator",
        condition=IfCondition(LaunchConfiguration("start_nao_orchestrator")),
        extra_parameters=[
            {"posture_command_topic": LaunchConfiguration("posture_command_topic")}
        ],
        configure_delay_sec=2.5,
        activate_delay_sec=5.0,
    )

    nao_say_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_say_skill",
        executable="start_skill",
        node_name="nao_say_skill",
        condition=IfCondition(LaunchConfiguration("start_nao_say_skill")),
        configure_delay_sec=1.0,
        activate_delay_sec=3.0,
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
        }.items(),
    )

    naoqi_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("naoqi_driver"), "launch", "naoqi_driver.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_naoqi_driver")),
        launch_arguments={
            "nao_ip": LaunchConfiguration("nao_ip"),
            "nao_port": LaunchConfiguration("nao_port"),
            "network_interface": LaunchConfiguration("network_interface"),
            "qi_listen_url": LaunchConfiguration("qi_listen_url"),
        }.items(),
    )

    nao_look_at_bundle = _make_lifecycle_bundle(
        package_name="nao_look_at",
        executable="start_skill",
        node_name="nao_look_at",
        condition=IfCondition(LaunchConfiguration("start_nao_look_at")),
        configure_delay_sec=1.0,
        activate_delay_sec=3.0,
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            start_chatbot_llm_arg,
            start_dialogue_manager_arg,
            start_nao_orchestrator_arg,
            start_nao_say_skill_arg,
            start_nao_replay_motion_arg,
            start_nao_look_at_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            posture_command_topic_arg,
            dialogue_manager_chatbot_arg,
            naoqi_driver_launch,
            *chatbot_llm_bundle,
            *dialogue_manager_bundle,
            *nao_orchestrator_bundle,
            *nao_say_skill_bundle,
            nao_replay_motion_launch,
            *nao_look_at_bundle,
        ]
    )
