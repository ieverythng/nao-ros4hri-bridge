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
        DeclareLaunchArgument('include_raw_payloads', default_value='false'),
        DeclareLaunchArgument('max_payload_chars', default_value='4000'),
        DeclareLaunchArgument('include_channels_csv', default_value=''),
        DeclareLaunchArgument('exclude_channels_csv', default_value=''),
        DeclareLaunchArgument('include_event_types_csv', default_value=''),
        DeclareLaunchArgument('exclude_event_types_csv', default_value=''),
        DeclareLaunchArgument('enable_scene_summary_channel', default_value='false'),
        DeclareLaunchArgument('scene_summary_emit_on_change_only', default_value='true'),
        DeclareLaunchArgument('scene_summary_min_interval_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'rosout_node_allowlist_csv',
            default_value=(
                'chatbot_llm,planner_llm,nao_orchestrator,scan_skill_server,'
                'report_result_skill_server,fake_skill_server,dialogue_manager,nao_say_skill,'
                'head_motion_skill_server,replay_motion_skill_server,nao_look_at,robot_speech_debug'
            ),
        ),
        DeclareLaunchArgument('rosout_min_level', default_value='warn'),
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
                'include_channels_csv': LaunchConfiguration('include_channels_csv'),
                'exclude_channels_csv': LaunchConfiguration('exclude_channels_csv'),
                'include_event_types_csv': LaunchConfiguration('include_event_types_csv'),
                'exclude_event_types_csv': LaunchConfiguration('exclude_event_types_csv'),
                'enable_scene_summary_channel': LaunchConfiguration('enable_scene_summary_channel'),
                'scene_summary_emit_on_change_only': LaunchConfiguration('scene_summary_emit_on_change_only'),
                'scene_summary_min_interval_sec': LaunchConfiguration('scene_summary_min_interval_sec'),
                'rosout_node_allowlist_csv': LaunchConfiguration('rosout_node_allowlist_csv'),
                'rosout_min_level': LaunchConfiguration('rosout_min_level'),
            }
        ],
    )

    return LaunchDescription(args + [node])
