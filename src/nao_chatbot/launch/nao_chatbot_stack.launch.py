from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="false",
        description="Whether to launch naoqi_driver together with chatbot stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="10.10.200.149",
        description="NAO robot IP address used by naoqi_driver.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port used by naoqi_driver.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description=(
            "Network interface for naoqi_driver (must exist in host/container, "
            "eg wlp1s0 or enx...)."
        ),
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL used by naoqi_driver audio extractor.",
    )
    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value="true",
        description="Whether to auto-start rqt_chat UI together with this stack.",
    )
    rqt_chat_start_delay_sec_arg = DeclareLaunchArgument(
        "rqt_chat_start_delay_sec",
        default_value="1.5",
        description="Delay before launching rqt_chat (helps when stack is still booting).",
    )
    mission_mode_arg = DeclareLaunchArgument(
        "mission_mode",
        default_value="rules",
        description='Mission controller mode: "rules" or "backend".',
    )
    backend_fallback_to_rules_arg = DeclareLaunchArgument(
        "backend_fallback_to_rules",
        default_value="false",
        description="If true, mission controller emits rule fallback on backend timeout.",
    )
    backend_response_timeout_sec_arg = DeclareLaunchArgument(
        "backend_response_timeout_sec",
        default_value="30.0",
        description="Seconds to wait for backend response before fallback.",
    )
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Topic where posture intents are published as commands.",
    )
    posture_bridge_enabled_arg = DeclareLaunchArgument(
        "posture_bridge_enabled",
        default_value="true",
        description="Whether to bridge /chatbot/posture_command to NAO ALRobotPosture.",
    )
    posture_bridge_speed_arg = DeclareLaunchArgument(
        "posture_bridge_speed",
        default_value="0.8",
        description="Speed used for ALRobotPosture.goToPosture calls.",
    )
    posture_bridge_stand_posture_name_arg = DeclareLaunchArgument(
        "posture_bridge_stand_posture_name",
        default_value="Stand",
        description='Posture name used when command is "stand" (e.g. Stand or StandInit).',
    )
    posture_bridge_kneel_posture_name_arg = DeclareLaunchArgument(
        "posture_bridge_kneel_posture_name",
        default_value="Crouch",
        description='Posture name used when command is "kneel" (e.g. Crouch).',
    )
    posture_bridge_stand_speed_arg = DeclareLaunchArgument(
        "posture_bridge_stand_speed",
        default_value="0.8",
        description='Speed used for "stand" posture calls.',
    )
    posture_bridge_kneel_speed_arg = DeclareLaunchArgument(
        "posture_bridge_kneel_speed",
        default_value="0.8",
        description='Speed used for "kneel"/"crouch" posture calls.',
    )
    posture_bridge_sit_speed_arg = DeclareLaunchArgument(
        "posture_bridge_sit_speed",
        default_value="0.8",
        description='Speed used for "sit" posture calls.',
    )
    posture_bridge_command_dedupe_window_sec_arg = DeclareLaunchArgument(
        "posture_bridge_command_dedupe_window_sec",
        default_value="1.5",
        description="Ignore duplicate posture commands within this time window (seconds).",
    )
    ollama_enabled_arg = DeclareLaunchArgument(
        "ollama_enabled",
        default_value="false",
        description="Whether to enable Ollama backend responses.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="llama3.2:1b",
        description="Ollama model name to use for chat backend.",
    )
    ollama_url_arg = DeclareLaunchArgument(
        "ollama_url",
        default_value="http://localhost:11434/api/chat",
        description="Ollama chat endpoint URL.",
    )
    ollama_request_timeout_sec_arg = DeclareLaunchArgument(
        "ollama_request_timeout_sec",
        default_value="20.0",
        description="Timeout for regular Ollama requests.",
    )
    ollama_first_request_timeout_sec_arg = DeclareLaunchArgument(
        "ollama_first_request_timeout_sec",
        default_value="60.0",
        description="Timeout for first Ollama request (model warm-up).",
    )
    ollama_context_window_tokens_arg = DeclareLaunchArgument(
        "ollama_context_window_tokens",
        default_value="4096",
        description="Ollama context window (num_ctx) in tokens.",
    )
    ollama_temperature_arg = DeclareLaunchArgument(
        "ollama_temperature",
        default_value="0.2",
        description="Ollama temperature for response variability.",
    )
    ollama_top_p_arg = DeclareLaunchArgument(
        "ollama_top_p",
        default_value="0.9",
        description="Ollama top_p sampling parameter.",
    )
    ollama_robot_name_arg = DeclareLaunchArgument(
        "ollama_robot_name",
        default_value="NAO",
        description="Robot name injected into system prompt.",
    )
    ollama_persona_prompt_path_arg = DeclareLaunchArgument(
        "ollama_persona_prompt_path",
        default_value="",
        description="Optional absolute path to persona prompt text file.",
    )
    ollama_prompt_addendum_arg = DeclareLaunchArgument(
        "ollama_prompt_addendum",
        default_value="",
        description="Extra instruction text appended to system prompt.",
    )

    start_naoqi_driver = LaunchConfiguration("start_naoqi_driver")
    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")
    network_interface = LaunchConfiguration("network_interface")
    qi_listen_url = LaunchConfiguration("qi_listen_url")
    start_rqt_chat = LaunchConfiguration("start_rqt_chat")
    rqt_chat_start_delay_sec = LaunchConfiguration("rqt_chat_start_delay_sec")
    mission_mode = LaunchConfiguration("mission_mode")
    backend_fallback_to_rules = LaunchConfiguration("backend_fallback_to_rules")
    backend_response_timeout_sec = LaunchConfiguration("backend_response_timeout_sec")
    posture_command_topic = LaunchConfiguration("posture_command_topic")
    posture_bridge_enabled = LaunchConfiguration("posture_bridge_enabled")
    posture_bridge_speed = LaunchConfiguration("posture_bridge_speed")
    posture_bridge_stand_posture_name = LaunchConfiguration(
        "posture_bridge_stand_posture_name"
    )
    posture_bridge_kneel_posture_name = LaunchConfiguration(
        "posture_bridge_kneel_posture_name"
    )
    posture_bridge_stand_speed = LaunchConfiguration("posture_bridge_stand_speed")
    posture_bridge_kneel_speed = LaunchConfiguration("posture_bridge_kneel_speed")
    posture_bridge_sit_speed = LaunchConfiguration("posture_bridge_sit_speed")
    posture_bridge_command_dedupe_window_sec = LaunchConfiguration(
        "posture_bridge_command_dedupe_window_sec"
    )
    ollama_enabled = LaunchConfiguration("ollama_enabled")
    ollama_model = LaunchConfiguration("ollama_model")
    ollama_url = LaunchConfiguration("ollama_url")
    ollama_request_timeout_sec = LaunchConfiguration("ollama_request_timeout_sec")
    ollama_first_request_timeout_sec = LaunchConfiguration(
        "ollama_first_request_timeout_sec"
    )
    ollama_context_window_tokens = LaunchConfiguration("ollama_context_window_tokens")
    ollama_temperature = LaunchConfiguration("ollama_temperature")
    ollama_top_p = LaunchConfiguration("ollama_top_p")
    ollama_robot_name = LaunchConfiguration("ollama_robot_name")
    ollama_persona_prompt_path = LaunchConfiguration("ollama_persona_prompt_path")
    ollama_prompt_addendum = LaunchConfiguration("ollama_prompt_addendum")

    naoqi_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("naoqi_driver"), "launch", "naoqi_driver.launch.py"]
            )
        ),
        condition=IfCondition(start_naoqi_driver),
        launch_arguments={
            "nao_ip": nao_ip,
            "nao_port": nao_port,
            "network_interface": network_interface,
            "qi_listen_url": qi_listen_url,
        }.items(),
    )

    bridge = Node(
        package="nao_chatbot",
        executable="nao_rqt_bridge_node",
        name="nao_rqt_bridge",
        output="screen",
    )
    mission = Node(
        package="nao_chatbot",
        executable="mission_controller_node",
        name="mission_controller",
        output="screen",
        parameters=[
            {
                "mode": mission_mode,
                "backend_fallback_to_rules": backend_fallback_to_rules,
                "backend_response_timeout_sec": backend_response_timeout_sec,
                "posture_command_topic": posture_command_topic,
            }
        ],
    )
    ollama = Node(
        package="nao_chatbot",
        executable="ollama_responder_node",
        name="ollama_responder",
        output="screen",
        parameters=[
            {
                "enabled": ollama_enabled,
                "model": ollama_model,
                "ollama_url": ollama_url,
                "request_timeout_sec": ollama_request_timeout_sec,
                "first_request_timeout_sec": ollama_first_request_timeout_sec,
                "context_window_tokens": ollama_context_window_tokens,
                "temperature": ollama_temperature,
                "top_p": ollama_top_p,
                "robot_name": ollama_robot_name,
                "persona_prompt_path": ollama_persona_prompt_path,
                "prompt_addendum": ollama_prompt_addendum,
            }
        ],
    )
    posture_bridge = Node(
        package="nao_posture_bridge",
        executable="nao_posture_bridge_node",
        name="nao_posture_bridge",
        output="screen",
        condition=IfCondition(posture_bridge_enabled),
        parameters=[
            {
                "posture_command_topic": posture_command_topic,
                "nao_ip": nao_ip,
                "nao_port": nao_port,
                "posture_speed": posture_bridge_speed,
                "stand_posture_name": posture_bridge_stand_posture_name,
                "kneel_posture_name": posture_bridge_kneel_posture_name,
                "stand_speed": posture_bridge_stand_speed,
                "kneel_speed": posture_bridge_kneel_speed,
                "sit_speed": posture_bridge_sit_speed,
                "command_dedupe_window_sec": posture_bridge_command_dedupe_window_sec,
            }
        ],
    )

    rqt_chat_process = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_gui",
            "rqt_gui",
            "--standalone",
            "rqt_chat.chat.ChatPlugin",
            "--force-discover",
        ],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_X11_NO_MITSHM": "1",
        },
        output="screen",
        condition=IfCondition(start_rqt_chat),
    )

    rqt_chat = TimerAction(
        period=rqt_chat_start_delay_sec,
        actions=[rqt_chat_process],
        condition=IfCondition(start_rqt_chat),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            start_rqt_chat_arg,
            rqt_chat_start_delay_sec_arg,
            mission_mode_arg,
            backend_fallback_to_rules_arg,
            backend_response_timeout_sec_arg,
            posture_command_topic_arg,
            posture_bridge_enabled_arg,
            posture_bridge_speed_arg,
            posture_bridge_stand_posture_name_arg,
            posture_bridge_kneel_posture_name_arg,
            posture_bridge_stand_speed_arg,
            posture_bridge_kneel_speed_arg,
            posture_bridge_sit_speed_arg,
            posture_bridge_command_dedupe_window_sec_arg,
            ollama_enabled_arg,
            ollama_model_arg,
            ollama_url_arg,
            ollama_request_timeout_sec_arg,
            ollama_first_request_timeout_sec_arg,
            ollama_context_window_tokens_arg,
            ollama_temperature_arg,
            ollama_top_p_arg,
            ollama_robot_name_arg,
            ollama_persona_prompt_path_arg,
            ollama_prompt_addendum_arg,
            naoqi_driver,
            bridge,
            mission,
            ollama,
            posture_bridge,
            rqt_chat,
        ]
    )
