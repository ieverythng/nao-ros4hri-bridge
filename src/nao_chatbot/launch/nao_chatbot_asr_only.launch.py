from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    asr_vosk_enabled_arg = DeclareLaunchArgument(
        "asr_vosk_enabled",
        default_value="true",
        description="Enable Vosk ASR lifecycle node.",
    )
    asr_microphone_topic_arg = DeclareLaunchArgument(
        "asr_microphone_topic",
        default_value="/laptop/microphone0",
        description="Audio topic consumed by asr_vosk.",
    )
    asr_output_speech_topic_arg = DeclareLaunchArgument(
        "asr_output_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description="LiveSpeech topic where ASR publishes transcripts.",
    )
    asr_speech_locale_arg = DeclareLaunchArgument(
        "asr_speech_locale",
        default_value="en_US",
        description="Locale string published in LiveSpeech messages.",
    )
    asr_vosk_model_path_arg = DeclareLaunchArgument(
        "asr_vosk_model_path",
        default_value="/models/vosk-model-small-en-us-0.15",
        description="Absolute path to Vosk model used by asr_vosk.",
    )
    asr_sample_rate_hz_arg = DeclareLaunchArgument(
        "asr_sample_rate_hz",
        default_value="16000",
        description="Input sample rate consumed by asr_vosk.",
    )
    asr_start_listening_arg = DeclareLaunchArgument(
        "asr_start_listening",
        default_value="true",
        description="Whether ASR starts listening immediately after activation.",
    )
    asr_publish_partials_arg = DeclareLaunchArgument(
        "asr_publish_partials",
        default_value="false",
        description="Publish incremental partial hypotheses from ASR.",
    )
    asr_min_final_chars_arg = DeclareLaunchArgument(
        "asr_min_final_chars",
        default_value="2",
        description="Drop final hypotheses shorter than this number of characters.",
    )
    asr_min_final_words_arg = DeclareLaunchArgument(
        "asr_min_final_words",
        default_value="1",
        description="Drop final hypotheses with fewer words than this value.",
    )
    asr_min_final_confidence_arg = DeclareLaunchArgument(
        "asr_min_final_confidence",
        default_value="0.0",
        description="Minimum average confidence [0-1] required for final ASR hypotheses.",
    )
    asr_ignore_single_token_fillers_arg = DeclareLaunchArgument(
        "asr_ignore_single_token_fillers",
        default_value="true",
        description="Drop one-token filler words (eg. uh, um, huh).",
    )
    asr_single_token_fillers_csv_arg = DeclareLaunchArgument(
        "asr_single_token_fillers_csv",
        default_value="uh,um,hmm,huh,erm,ah,eh",
        description="Comma-separated fillers dropped when one-token filler filtering is enabled.",
    )
    asr_debug_log_results_arg = DeclareLaunchArgument(
        "asr_debug_log_results",
        default_value="false",
        description="Enable debug logs of final ASR hypotheses and filtering decisions.",
    )
    asr_push_to_talk_enabled_arg = DeclareLaunchArgument(
        "asr_push_to_talk_enabled",
        default_value="true",
        description="Require an explicit Bool gate before ASR listens.",
    )
    asr_push_to_talk_topic_arg = DeclareLaunchArgument(
        "asr_push_to_talk_topic",
        default_value="/asr_vosk/push_to_talk",
        description="Bool topic used to enable/disable listening in push-to-talk mode.",
    )
    asr_audio_capture_enabled_arg = DeclareLaunchArgument(
        "asr_audio_capture_enabled",
        default_value="true",
        description="Launch simple_audio_capture together with asr_vosk.",
    )
    asr_audio_capture_source_type_arg = DeclareLaunchArgument(
        "asr_audio_capture_source_type",
        default_value="pulsesrc",
        description='GStreamer source type ("pulsesrc" or "alsasrc").',
    )
    asr_audio_capture_device_arg = DeclareLaunchArgument(
        "asr_audio_capture_device",
        default_value="",
        description="Optional audio device identifier passed to simple_audio_capture.",
    )
    asr_audio_capture_sample_rate_arg = DeclareLaunchArgument(
        "asr_audio_capture_sample_rate",
        default_value="16000",
        description="Audio capture sample rate.",
    )
    asr_audio_capture_channels_arg = DeclareLaunchArgument(
        "asr_audio_capture_channels",
        default_value="1",
        description="Audio capture channels.",
    )
    asr_audio_capture_sample_format_arg = DeclareLaunchArgument(
        "asr_audio_capture_sample_format",
        default_value="S16LE",
        description="Audio capture sample format.",
    )
    asr_audio_capture_chunk_size_arg = DeclareLaunchArgument(
        "asr_audio_capture_chunk_size",
        default_value="2048",
        description="Audio capture chunk size in bytes.",
    )

    asr_vosk_enabled = LaunchConfiguration("asr_vosk_enabled")
    asr_microphone_topic = LaunchConfiguration("asr_microphone_topic")
    asr_output_speech_topic = LaunchConfiguration("asr_output_speech_topic")
    asr_speech_locale = LaunchConfiguration("asr_speech_locale")
    asr_vosk_model_path = LaunchConfiguration("asr_vosk_model_path")
    asr_sample_rate_hz = LaunchConfiguration("asr_sample_rate_hz")
    asr_start_listening = LaunchConfiguration("asr_start_listening")
    asr_publish_partials = LaunchConfiguration("asr_publish_partials")
    asr_min_final_chars = LaunchConfiguration("asr_min_final_chars")
    asr_min_final_words = LaunchConfiguration("asr_min_final_words")
    asr_min_final_confidence = LaunchConfiguration("asr_min_final_confidence")
    asr_ignore_single_token_fillers = LaunchConfiguration(
        "asr_ignore_single_token_fillers"
    )
    asr_single_token_fillers_csv = LaunchConfiguration("asr_single_token_fillers_csv")
    asr_debug_log_results = LaunchConfiguration("asr_debug_log_results")
    asr_push_to_talk_enabled = LaunchConfiguration("asr_push_to_talk_enabled")
    asr_push_to_talk_topic = LaunchConfiguration("asr_push_to_talk_topic")
    asr_audio_capture_enabled = LaunchConfiguration("asr_audio_capture_enabled")
    asr_audio_capture_source_type = LaunchConfiguration("asr_audio_capture_source_type")
    asr_audio_capture_device = LaunchConfiguration("asr_audio_capture_device")
    asr_audio_capture_sample_rate = LaunchConfiguration("asr_audio_capture_sample_rate")
    asr_audio_capture_channels = LaunchConfiguration("asr_audio_capture_channels")
    asr_audio_capture_sample_format = LaunchConfiguration(
        "asr_audio_capture_sample_format"
    )
    asr_audio_capture_chunk_size = LaunchConfiguration("asr_audio_capture_chunk_size")

    asr_audio_capture = Node(
        package="simple_audio_capture",
        executable="audio_capture_node",
        name="simple_audio_capture",
        output="screen",
        condition=IfCondition(asr_audio_capture_enabled),
        parameters=[
            {
                "source_type": asr_audio_capture_source_type,
                "device": asr_audio_capture_device,
                "sample_rate": asr_audio_capture_sample_rate,
                "channels": asr_audio_capture_channels,
                "sample_format": asr_audio_capture_sample_format,
                "chunk_size": asr_audio_capture_chunk_size,
                "audio_topic": asr_microphone_topic,
            }
        ],
    )

    asr_vosk_condition = IfCondition(asr_vosk_enabled)
    asr_vosk = LifecycleNode(
        package="asr_vosk",
        executable="asr_vosk",
        name="asr_vosk",
        namespace="",
        output="screen",
        emulate_tty=True,
        condition=asr_vosk_condition,
        parameters=[
            {
                "audio_rate": asr_sample_rate_hz,
                "model": asr_vosk_model_path,
                "microphone_topic": asr_microphone_topic,
                "start_listening": asr_start_listening,
                "output_speech_topic": asr_output_speech_topic,
                "speech_locale": asr_speech_locale,
                "publish_partials": asr_publish_partials,
                "min_final_chars": asr_min_final_chars,
                "min_final_words": asr_min_final_words,
                "min_final_confidence": asr_min_final_confidence,
                "ignore_single_token_fillers": asr_ignore_single_token_fillers,
                "single_token_fillers_csv": asr_single_token_fillers_csv,
                "debug_log_results": asr_debug_log_results,
                "push_to_talk_enabled": asr_push_to_talk_enabled,
                "push_to_talk_topic": asr_push_to_talk_topic,
            }
        ],
    )
    asr_vosk_configure = EmitEvent(
        condition=asr_vosk_condition,
        event=ChangeState(
            lifecycle_node_matcher=matches_action(asr_vosk),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    asr_vosk_activate = RegisterEventHandler(
        condition=asr_vosk_condition,
        event_handler=OnStateTransition(
            target_lifecycle_node=asr_vosk,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(asr_vosk),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
            handle_once=True,
        ),
    )

    return LaunchDescription(
        [
            asr_vosk_enabled_arg,
            asr_microphone_topic_arg,
            asr_output_speech_topic_arg,
            asr_speech_locale_arg,
            asr_vosk_model_path_arg,
            asr_sample_rate_hz_arg,
            asr_start_listening_arg,
            asr_publish_partials_arg,
            asr_min_final_chars_arg,
            asr_min_final_words_arg,
            asr_min_final_confidence_arg,
            asr_ignore_single_token_fillers_arg,
            asr_single_token_fillers_csv_arg,
            asr_debug_log_results_arg,
            asr_push_to_talk_enabled_arg,
            asr_push_to_talk_topic_arg,
            asr_audio_capture_enabled_arg,
            asr_audio_capture_source_type_arg,
            asr_audio_capture_device_arg,
            asr_audio_capture_sample_rate_arg,
            asr_audio_capture_channels_arg,
            asr_audio_capture_sample_format_arg,
            asr_audio_capture_chunk_size_arg,
            asr_audio_capture,
            asr_vosk,
            asr_vosk_configure,
            asr_vosk_activate,
        ]
    )
