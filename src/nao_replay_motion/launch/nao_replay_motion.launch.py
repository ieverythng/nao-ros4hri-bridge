from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_launch_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    nao_ip_arg = DeclareLaunchArgument("nao_ip", default_value="172.26.112.62")
    nao_port_arg = DeclareLaunchArgument("nao_port", default_value="9559")
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
    )
    posture_result_topic_arg = DeclareLaunchArgument(
        "posture_result_topic",
        default_value="/chatbot/posture_command_result",
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
    posture_allow_open_loop_without_naoqi_arg = DeclareLaunchArgument(
        "posture_allow_open_loop_without_naoqi",
        default_value="false",
    )
    head_motion_retry_on_convergence_timeout_arg = DeclareLaunchArgument(
        "head_motion_retry_on_convergence_timeout",
        default_value="true",
    )
    head_motion_retry_convergence_timeout_sec_arg = DeclareLaunchArgument(
        "head_motion_retry_convergence_timeout_sec",
        default_value="1.5",
    )
    head_motion_allow_open_loop_without_joint_state_arg = DeclareLaunchArgument(
        "head_motion_allow_open_loop_without_joint_state",
        default_value="false",
    )
    head_motion_assume_success_on_convergence_timeout_arg = DeclareLaunchArgument(
        "head_motion_assume_success_on_convergence_timeout",
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
                "posture_result_topic": LaunchConfiguration("posture_result_topic"),
                "allow_open_loop_without_naoqi": _bool_launch_config(
                    "posture_allow_open_loop_without_naoqi"
                ),
            }
        ],
    )

    head_motion = Node(
        package="nao_replay_motion",
        executable="head_motion_skill_server_node",
        name="head_motion_skill_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "retry_on_convergence_timeout": _bool_launch_config(
                    "head_motion_retry_on_convergence_timeout"
                ),
                "retry_convergence_timeout_sec": LaunchConfiguration(
                    "head_motion_retry_convergence_timeout_sec"
                ),
                "allow_open_loop_without_joint_state": _bool_launch_config(
                    "head_motion_allow_open_loop_without_joint_state"
                ),
                "assume_success_on_convergence_timeout": _bool_launch_config(
                    "head_motion_assume_success_on_convergence_timeout"
                ),
            }
        ],
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
                "posture_result_topic": LaunchConfiguration("posture_result_topic"),
                "connect_on_startup": _bool_launch_config(
                    "posture_bridge_connect_on_startup"
                ),
                "disable_autonomous_life_on_connect": _bool_launch_config(
                    "posture_bridge_disable_autonomous_life_on_connect"
                ),
                "wake_up_on_connect": _bool_launch_config(
                    "posture_bridge_wake_up_on_connect"
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            nao_ip_arg,
            nao_port_arg,
            posture_command_topic_arg,
            posture_result_topic_arg,
            posture_bridge_connect_on_startup_arg,
            posture_bridge_disable_autonomous_life_on_connect_arg,
            posture_bridge_wake_up_on_connect_arg,
            posture_allow_open_loop_without_naoqi_arg,
            head_motion_retry_on_convergence_timeout_arg,
            head_motion_retry_convergence_timeout_sec_arg,
            head_motion_allow_open_loop_without_joint_state_arg,
            head_motion_assume_success_on_convergence_timeout_arg,
            replay_motion,
            head_motion,
            posture_bridge,
        ]
    )
