from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'asr_vosk'
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        '00-defaults.yml'
    )
    audio_rate_arg = DeclareLaunchArgument(
        'audio_rate',
        default_value='16000',
        description='Device sampling rate',
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='/models/vosk-model-small-en-us-0.15',
        description='Absolute path to Vosk model directory',
    )
    microphone_topic_arg = DeclareLaunchArgument(
        'microphone_topic',
        default_value='/laptop/microphone0',
        description='Microphone topic with AudioData payload',
    )
    start_listening_arg = DeclareLaunchArgument(
        'start_listening',
        default_value='true',
        description='Whether the node starts listening on activate',
    )
    output_speech_topic_arg = DeclareLaunchArgument(
        'output_speech_topic',
        default_value='/humans/voices/anonymous_speaker/speech',
        description='LiveSpeech output topic',
    )
    speech_locale_arg = DeclareLaunchArgument(
        'speech_locale',
        default_value='en_US',
        description='LiveSpeech locale field',
    )
    publish_partials_arg = DeclareLaunchArgument(
        'publish_partials',
        default_value='true',
        description='Publish incremental partial hypotheses.',
    )
    min_final_chars_arg = DeclareLaunchArgument(
        'min_final_chars',
        default_value='2',
        description='Drop final hypotheses shorter than this number of characters.',
    )
    min_final_words_arg = DeclareLaunchArgument(
        'min_final_words',
        default_value='1',
        description='Drop final hypotheses with fewer words than this value.',
    )
    min_final_confidence_arg = DeclareLaunchArgument(
        'min_final_confidence',
        default_value='0.0',
        description='Minimum average confidence [0-1] required for final hypotheses.',
    )
    ignore_single_token_fillers_arg = DeclareLaunchArgument(
        'ignore_single_token_fillers',
        default_value='true',
        description='Drop single-token fillers (eg. uh, um, huh).',
    )
    single_token_fillers_csv_arg = DeclareLaunchArgument(
        'single_token_fillers_csv',
        default_value='uh,um,hmm,huh,erm,ah,eh',
        description='Comma-separated fillers removed when single-token filtering is enabled.',
    )
    debug_log_results_arg = DeclareLaunchArgument(
        'debug_log_results',
        default_value='false',
        description='Enable debug logs for final hypotheses and filtering decisions.',
    )

    vosk_node = LifecycleNode(
        package=pkg_name,
        executable='asr_vosk',
        name='asr_vosk',
        namespace='',
        parameters=[
            config_path,
            {
                'audio_rate': LaunchConfiguration('audio_rate'),
                'model': LaunchConfiguration('model'),
                'microphone_topic': LaunchConfiguration('microphone_topic'),
                'start_listening': LaunchConfiguration('start_listening'),
                'output_speech_topic': LaunchConfiguration('output_speech_topic'),
                'speech_locale': LaunchConfiguration('speech_locale'),
                'publish_partials': LaunchConfiguration('publish_partials'),
                'min_final_chars': LaunchConfiguration('min_final_chars'),
                'min_final_words': LaunchConfiguration('min_final_words'),
                'min_final_confidence': LaunchConfiguration('min_final_confidence'),
                'ignore_single_token_fillers': LaunchConfiguration('ignore_single_token_fillers'),
                'single_token_fillers_csv': LaunchConfiguration('single_token_fillers_csv'),
                'debug_log_results': LaunchConfiguration('debug_log_results'),
            },
        ],
        output='screen',
        emulate_tty=True
    )

    # Trigger lifecycle transitions
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(vosk_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=vosk_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(vosk_node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
        handle_once=True))

    ld = LaunchDescription()
    ld.add_action(audio_rate_arg)
    ld.add_action(model_arg)
    ld.add_action(microphone_topic_arg)
    ld.add_action(start_listening_arg)
    ld.add_action(output_speech_topic_arg)
    ld.add_action(speech_locale_arg)
    ld.add_action(publish_partials_arg)
    ld.add_action(min_final_chars_arg)
    ld.add_action(min_final_words_arg)
    ld.add_action(min_final_confidence_arg)
    ld.add_action(ignore_single_token_fillers_arg)
    ld.add_action(single_token_fillers_csv_arg)
    ld.add_action(debug_log_results_arg)
    ld.add_action(vosk_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
