"""Launch the NAO dashboard backend node and web UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('dashboard_enabled', default_value='true'),
        DeclareLaunchArgument('http_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('http_port', default_value='8765'),
        DeclareLaunchArgument('discovery_period_sec', default_value='1.0'),
        DeclareLaunchArgument('graph_refresh_period_sec', default_value='1.5'),
        DeclareLaunchArgument('registry_refresh_period_sec', default_value='3.0'),
        DeclareLaunchArgument('max_events', default_value='400'),
        DeclareLaunchArgument('max_payload_chars', default_value='4000'),
    ]

    node = Node(
        package='nao_dashboard',
        executable='dashboard_node',
        name='nao_dashboard',
        output='screen',
        parameters=[
            {
                'dashboard_enabled': LaunchConfiguration('dashboard_enabled'),
                'http_host': LaunchConfiguration('http_host'),
                'http_port': LaunchConfiguration('http_port'),
                'discovery_period_sec': LaunchConfiguration('discovery_period_sec'),
                'graph_refresh_period_sec': LaunchConfiguration('graph_refresh_period_sec'),
                'registry_refresh_period_sec': LaunchConfiguration('registry_refresh_period_sec'),
                'max_events': LaunchConfiguration('max_events'),
                'max_payload_chars': LaunchConfiguration('max_payload_chars'),
            }
        ],
    )

    return LaunchDescription(args + [node])
