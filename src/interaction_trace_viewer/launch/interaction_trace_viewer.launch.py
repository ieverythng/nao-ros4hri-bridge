"""Launch the interaction trace viewer node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('trace_viewer_enabled', default_value='true'),
        DeclareLaunchArgument('compact_mode', default_value='true'),
        DeclareLaunchArgument('write_jsonl', default_value='true'),
        DeclareLaunchArgument('jsonl_output_dir', default_value='~/.ros/nao_ros4hri_traces'),
        DeclareLaunchArgument('write_html_on_shutdown', default_value='true'),
        DeclareLaunchArgument('html_output_dir', default_value='~/.ros/nao_ros4hri_trace_reports'),
        DeclareLaunchArgument('include_raw_payloads', default_value='true'),
        DeclareLaunchArgument('max_payload_chars', default_value='4000'),
    ]

    node = Node(
        package='interaction_trace_viewer',
        executable='trace_node',
        name='interaction_trace_viewer',
        output='screen',
        parameters=[
            {
                'trace_viewer_enabled': LaunchConfiguration('trace_viewer_enabled'),
                'compact_mode': LaunchConfiguration('compact_mode'),
                'write_jsonl': LaunchConfiguration('write_jsonl'),
                'jsonl_output_dir': LaunchConfiguration('jsonl_output_dir'),
                'write_html_on_shutdown': LaunchConfiguration('write_html_on_shutdown'),
                'html_output_dir': LaunchConfiguration('html_output_dir'),
                'include_raw_payloads': LaunchConfiguration('include_raw_payloads'),
                'max_payload_chars': LaunchConfiguration('max_payload_chars'),
            }
        ],
    )

    return LaunchDescription(args + [node])
