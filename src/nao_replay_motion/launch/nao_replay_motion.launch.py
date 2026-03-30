from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nao_ip_arg = DeclareLaunchArgument("nao_ip", default_value="172.26.112.62")
    nao_port_arg = DeclareLaunchArgument("nao_port", default_value="9559")
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
    )
    posture_bridge_connect_on_startup_arg = DeclareLaunchArgument(
        "posture_bridge_connect_on_startup",
        default_value="true",
    )
    posture_bridge_disable_autonomous_life_on_connect_arg = DeclareLaunchArgument(
        "posture_bridge_disable_autonomous_life_on_connect",
        default_value="false",
    )
    posture_bridge_wake_up_on_connect_arg = DeclareLaunchArgument(
        "posture_bridge_wake_up_on_connect",
        default_value="false",
    )

    replay_motion = Node(
        package="nao_replay_motion",
        executable="replay_motion_skill_server_node",
        name="replay_motion_skill_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "nao_ip": LaunchConfiguration("nao_ip"),
                "nao_port": LaunchConfiguration("nao_port"),
                "posture_command_topic": LaunchConfiguration("posture_command_topic"),
            }
        ],
    )

    head_motion = Node(
        package="nao_replay_motion",
        executable="head_motion_skill_server_node",
        name="head_motion_skill_server",
        output="screen",
        emulate_tty=True,
    )

    posture_bridge = Node(
        package="nao_replay_motion",
        executable="nao_posture_bridge_node",
        name="nao_posture_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "nao_ip": LaunchConfiguration("nao_ip"),
                "nao_port": LaunchConfiguration("nao_port"),
                "posture_command_topic": LaunchConfiguration("posture_command_topic"),
                "connect_on_startup": ParameterValue(
                    LaunchConfiguration("posture_bridge_connect_on_startup"),
                    value_type=bool,
                ),
                "disable_autonomous_life_on_connect": ParameterValue(
                    LaunchConfiguration(
                        "posture_bridge_disable_autonomous_life_on_connect"
                    ),
                    value_type=bool,
                ),
                "wake_up_on_connect": ParameterValue(
                    LaunchConfiguration("posture_bridge_wake_up_on_connect"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            nao_ip_arg,
            nao_port_arg,
            posture_command_topic_arg,
            posture_bridge_connect_on_startup_arg,
            posture_bridge_disable_autonomous_life_on_connect_arg,
            posture_bridge_wake_up_on_connect_arg,
            replay_motion,
            head_motion,
            posture_bridge,
        ]
    )
