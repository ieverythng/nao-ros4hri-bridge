"""Launch fake skill action servers for planner validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    scenario_file_arg = DeclareLaunchArgument(
        'fake_skill_scenario_file',
        default_value='',
        description='Optional YAML scenario file for fake skill outcomes.',
    )
    active_scenario_arg = DeclareLaunchArgument(
        'fake_skill_active_scenario_id',
        default_value='',
        description='Optional named scenario id applied by default to every fake skill request.',
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
    global_mode_arg = DeclareLaunchArgument(
        'fake_skill_global_mode',
        default_value='every_other',
        description='Global fake-skill policy: scenario|always_success|always_fail|every_other|random_seeded.',
    )
    random_failure_prob_arg = DeclareLaunchArgument(
        'fake_skill_random_failure_prob',
        default_value='0.50',
        description='Failure probability used by random_seeded mode.',
    )
    mode_overrides_arg = DeclareLaunchArgument(
        'fake_skill_mode_overrides_json',
        default_value='{}',
        description='JSON map for per-skill mode overrides, e.g. {"find_object":"always_fail"}.',
    )
    perform_motion_action_arg = DeclareLaunchArgument(
        'fake_skill_perform_motion_action',
        default_value='/skill/fake/perform_motion',
        description='Action endpoint for fake perform_motion.',
    )
    pick_object_action_arg = DeclareLaunchArgument(
        'fake_skill_pick_object_action',
        default_value='/skill/fake/pick_object',
        description='Action endpoint for fake pick_object.',
    )
    place_object_action_arg = DeclareLaunchArgument(
        'fake_skill_place_object_action',
        default_value='/skill/fake/place_object',
        description='Action endpoint for fake place_object.',
    )
    bring_object_action_arg = DeclareLaunchArgument(
        'fake_skill_bring_object_action',
        default_value='/skill/fake/bring_object',
        description='Action endpoint for fake bring_object.',
    )

    fake_skill_server = Node(
        package='fake_skills',
        executable='run_fake_skill_server',
        name='fake_skill_server',
        output='screen',
        parameters=[
            {
                'scenario_file': LaunchConfiguration('fake_skill_scenario_file'),
                'active_scenario_id': LaunchConfiguration('fake_skill_active_scenario_id'),
                'default_delay_sec': LaunchConfiguration('fake_skill_default_delay_sec'),
                'publish_events': LaunchConfiguration('fake_skill_publish_events'),
                'event_topic': LaunchConfiguration('fake_skill_event_topic'),
                'deterministic_seed': LaunchConfiguration('fake_skill_deterministic_seed'),
                'global_mode': LaunchConfiguration('fake_skill_global_mode'),
                'random_failure_prob': LaunchConfiguration('fake_skill_random_failure_prob'),
                'perform_motion_action_name': LaunchConfiguration(
                    'fake_skill_perform_motion_action'
                ),
                'pick_object_action_name': LaunchConfiguration(
                    'fake_skill_pick_object_action'
                ),
                'place_object_action_name': LaunchConfiguration(
                    'fake_skill_place_object_action'
                ),
                'bring_object_action_name': LaunchConfiguration(
                    'fake_skill_bring_object_action'
                ),
                'mode_overrides_json': ParameterValue(
                    LaunchConfiguration('fake_skill_mode_overrides_json'),
                    value_type=str,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            scenario_file_arg,
            active_scenario_arg,
            default_delay_arg,
            publish_events_arg,
            event_topic_arg,
            seed_arg,
            global_mode_arg,
            random_failure_prob_arg,
            mode_overrides_arg,
            perform_motion_action_arg,
            pick_object_action_arg,
            place_object_action_arg,
            bring_object_action_arg,
            fake_skill_server,
        ]
    )
