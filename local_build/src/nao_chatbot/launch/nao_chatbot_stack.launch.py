from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_mode_arg = DeclareLaunchArgument(
        "mission_mode",
        default_value="rules",
        description='Mission controller mode: "rules" or "backend".',
    )
    ollama_enabled_arg = DeclareLaunchArgument(
        "ollama_enabled",
        default_value="false",
        description="Whether to enable Ollama backend responses.",
    )

    mission_mode = LaunchConfiguration("mission_mode")
    ollama_enabled = LaunchConfiguration("ollama_enabled")

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
        parameters=[{"mode": mission_mode}],
    )
    ollama = Node(
        package="nao_chatbot",
        executable="ollama_responder_node",
        name="ollama_responder",
        output="screen",
        parameters=[{"enabled": ollama_enabled}],
    )

    # TODO: Add rqt_chat launch immediate after bridge is ready

    return LaunchDescription([mission_mode_arg, ollama_enabled_arg, bridge, mission, ollama])
