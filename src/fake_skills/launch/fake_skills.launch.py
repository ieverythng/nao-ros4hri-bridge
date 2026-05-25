"""Launch fake skill action servers for planner validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scenario_file_arg = DeclareLaunchArgument(
        'fake_skill_scenario_file',
        default_value='',
        description='Optional YAML scenario file for fake skill outcomes.',
    )
    default_delay_arg = DeclareLaunchArgument(
        'fake_skill_default_delay_sec',
        default_value='0.75',
        description='Default simulated duration for fake skill execution.',
    )
    publish_events_arg = DeclareLaunchArgument(
        'fake_skill_publish_events',
        default_value='true',
        description='Publish fake skill lifecycle events.',
    )
    event_topic_arg = DeclareLaunchArgument(
        'fake_skill_event_topic',
        default_value='/fake_skills/events',
        description='Topic for fake skill lifecycle events.',
    )
    seed_arg = DeclareLaunchArgument(
        'fake_skill_deterministic_seed',
        default_value='42',
        description='Deterministic seed for fail_once tracking.',
    )

    fake_skill_server = Node(
        package='fake_skills',
        executable='run_fake_skill_server',
        name='fake_skill_server',
        output='screen',
        parameters=[
            {
                'scenario_file': LaunchConfiguration('fake_skill_scenario_file'),
                'default_delay_sec': LaunchConfiguration('fake_skill_default_delay_sec'),
                'publish_events': LaunchConfiguration('fake_skill_publish_events'),
                'event_topic': LaunchConfiguration('fake_skill_event_topic'),
                'deterministic_seed': LaunchConfiguration('fake_skill_deterministic_seed'),
            }
        ],
    )

    return LaunchDescription(
        [
            scenario_file_arg,
            default_delay_arg,
            publish_events_arg,
            event_topic_arg,
            seed_arg,
            fake_skill_server,
        ]
    )
