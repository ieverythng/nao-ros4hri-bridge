from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="true",
        description="Whether to launch naoqi_driver.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="172.26.112.62",
        description="NAO robot IP.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface used by naoqi_driver.",
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL for naoqi_driver.",
    )
    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value="true",
        description="Start rqt_chat UI.",
    )
    mission_mode_arg = DeclareLaunchArgument(
        "mission_mode",
        default_value="backend",
        description='Mission controller mode: "rules" or "backend".',
    )
    ollama_enabled_arg = DeclareLaunchArgument(
        "ollama_enabled",
        default_value="true",
        description="Enable Ollama backend node.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="llama3.2:1b",
        description="Ollama model name.",
    )
    posture_skill_speed_arg = DeclareLaunchArgument(
        "posture_skill_speed",
        default_value="0.9",
        description="Posture skill speed sent by mission controller.",
    )
    posture_skill_server_fallback_to_topic_arg = DeclareLaunchArgument(
        "posture_skill_server_fallback_to_topic",
        default_value="true",
        description="Allow skill server topic fallback when direct NAOqi is unavailable.",
    )

    start_naoqi_driver = LaunchConfiguration("start_naoqi_driver")
    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")
    network_interface = LaunchConfiguration("network_interface")
    qi_listen_url = LaunchConfiguration("qi_listen_url")
    start_rqt_chat = LaunchConfiguration("start_rqt_chat")
    mission_mode = LaunchConfiguration("mission_mode")
    ollama_enabled = LaunchConfiguration("ollama_enabled")
    ollama_model = LaunchConfiguration("ollama_model")
    posture_skill_speed = LaunchConfiguration("posture_skill_speed")
    posture_skill_server_fallback_to_topic = LaunchConfiguration(
        "posture_skill_server_fallback_to_topic"
    )

    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nao_chatbot"), "launch", "nao_chatbot_stack.launch.py"]
            )
        ),
        launch_arguments={
            "start_naoqi_driver": start_naoqi_driver,
            "nao_ip": nao_ip,
            "nao_port": nao_port,
            "network_interface": network_interface,
            "qi_listen_url": qi_listen_url,
            "start_rqt_chat": start_rqt_chat,
            "mission_mode": mission_mode,
            "ollama_enabled": ollama_enabled,
            "ollama_model": ollama_model,
            "use_posture_skill": "true",
            "posture_skill_speed": posture_skill_speed,
            "posture_skill_server_enabled": "true",
            "posture_skill_server_fallback_to_topic": posture_skill_server_fallback_to_topic,
            "posture_bridge_enabled": "true",
            "backend_execute_posture_after_response": "true",
            "backend_posture_from_response_enabled": "false",
            "laptop_asr_enabled": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            start_rqt_chat_arg,
            mission_mode_arg,
            ollama_enabled_arg,
            ollama_model_arg,
            posture_skill_speed_arg,
            posture_skill_server_fallback_to_topic_arg,
            stack_launch,
        ]
    )
