from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
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
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                TimerAction(
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
                ),
                TimerAction(
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
                ),
            ],
        ),
    )
    return [node, configure]


def _prefer_first_non_empty(*names: str):
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
    start_rqt_console_arg = DeclareLaunchArgument(
        "start_rqt_console",
        default_value="true",
        description="Launch the full rqt shell for runtime tools.",
    )
    start_robot_speech_debug_arg = DeclareLaunchArgument(
        "start_robot_speech_debug",
        default_value="true",
        description="Launch a logger that mirrors robot speech into ROS logs.",
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
    chatbot_model_arg = DeclareLaunchArgument(
        "chatbot_model",
        default_value="llama3.2:1b",
        description="Model used by chatbot_llm for response generation.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="",
        description="Backward-compatible alias for chatbot_model.",
    )
    chatbot_intent_model_arg = DeclareLaunchArgument(
        "chatbot_intent_model",
        default_value="",
        description="Optional dedicated model used by chatbot_llm for intent extraction.",
    )
    ollama_intent_model_arg = DeclareLaunchArgument(
        "ollama_intent_model",
        default_value="",
        description="Backward-compatible alias for chatbot_intent_model.",
    )
    chatbot_server_url_arg = DeclareLaunchArgument(
        "chatbot_server_url",
        default_value="http://localhost:11434/api/chat",
        description="Backend HTTP endpoint used by chatbot_llm.",
    )

    chatbot_llm_bundle = _make_lifecycle_bundle(
        package_name="chatbot_llm",
        executable="start_node",
        node_name="chatbot_llm",
        condition=IfCondition(LaunchConfiguration("start_chatbot_llm")),
        extra_parameters=[
            {
                "model": ParameterValue(
                    _prefer_first_non_empty("ollama_model", "chatbot_model"),
                    value_type=str,
                )
            },
            {
                "intent_model": ParameterValue(
                    _prefer_first_non_empty(
                        "ollama_intent_model",
                        "chatbot_intent_model",
                        "ollama_model",
                        "chatbot_model",
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
        ],
        configure_delay_sec=3.0,
        activate_delay_sec=6.5,
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
        ],
        configure_delay_sec=4.5,
        activate_delay_sec=8.0,
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
            }
        ],
        configure_delay_sec=4.0,
        activate_delay_sec=7.0,
    )

    nao_say_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_say_skill",
        executable="start_skill",
        node_name="nao_say_skill",
        condition=IfCondition(LaunchConfiguration("start_nao_say_skill")),
        configure_delay_sec=5.0,
        activate_delay_sec=9.0,
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
        configure_delay_sec=2.0,
        activate_delay_sec=5.0,
    )

    rqt_console = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_rqt_console")),
        cmd=[
            "bash",
            "-lc",
            "if ! command -v rqt >/dev/null 2>&1; then "
            "echo 'rqt is not installed in this environment'; "
            "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
            "echo 'rqt launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
            "else "
            "exec rqt; "
            "fi",
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

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            start_chatbot_llm_arg,
            start_dialogue_manager_arg,
            start_nao_orchestrator_arg,
            start_nao_say_skill_arg,
            start_nao_replay_motion_arg,
            start_nao_look_at_arg,
            start_rqt_console_arg,
            start_robot_speech_debug_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            posture_command_topic_arg,
            dialogue_manager_chatbot_arg,
            dialogue_manager_enable_default_chat_arg,
            dialogue_manager_default_chat_role_arg,
            dialogue_manager_default_chat_configuration_arg,
            chatbot_model_arg,
            ollama_model_arg,
            chatbot_intent_model_arg,
            ollama_intent_model_arg,
            chatbot_server_url_arg,
            naoqi_driver_launch,
            rqt_console,
            robot_speech_debug,
            *chatbot_llm_bundle,
            *dialogue_manager_bundle,
            *nao_orchestrator_bundle,
            *nao_say_skill_bundle,
            nao_replay_motion_launch,
            *nao_look_at_bundle,
        ]
    )
