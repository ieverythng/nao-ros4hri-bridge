from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'start_naoqi_driver',
            default_value='false',
            description='Optionally launch naoqi_driver alongside the migrated stack.',
        ),
        DeclareLaunchArgument(
            'nao_ip',
            default_value='172.26.112.62',
            description='NAO robot IP passed to migrated motion and driver nodes.',
        ),
        DeclareLaunchArgument(
            'nao_port',
            default_value='9559',
            description='NAOqi port passed to migrated motion and driver nodes.',
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Network interface used by naoqi_driver when enabled.',
        ),
        DeclareLaunchArgument(
            'qi_listen_url',
            default_value='tcp://0.0.0.0:0',
            description='QI listen URL used by naoqi_driver when enabled.',
        ),
        DeclareLaunchArgument(
            'posture_command_topic',
            default_value='/chatbot/posture_command',
            description='Temporary posture bridge topic used during migration.',
        ),
        DeclareLaunchArgument(
            'dialogue_manager_chatbot',
            default_value='chatbot_llm',
            description='Dialogue-manager chatbot backend prefix.',
        ),
        DeclareLaunchArgument(
            'dialogue_manager_enable_default_chat',
            default_value='true',
            description='Start a default dialogue so user speech routes to chatbot_llm immediately.',
        ),
        DeclareLaunchArgument(
            'dialogue_manager_default_chat_role',
            default_value='__default__',
            description='Role used for the default dialogue session.',
        ),
        DeclareLaunchArgument(
            'dialogue_manager_default_chat_configuration',
            default_value='',
            description='Optional JSON configuration passed to the default dialogue session.',
        ),
        DeclareLaunchArgument(
            'chatbot_model',
            default_value='llama3.2:1b',
            description='Model used by chatbot_llm for response generation.',
        ),
        DeclareLaunchArgument(
            'ollama_model',
            default_value='',
            description='Backward-compatible alias for chatbot_model.',
        ),
        DeclareLaunchArgument(
            'chatbot_intent_model',
            default_value='',
            description='Optional dedicated model used by chatbot_llm for intent extraction.',
        ),
        DeclareLaunchArgument(
            'ollama_intent_model',
            default_value='',
            description='Backward-compatible alias for chatbot_intent_model.',
        ),
        DeclareLaunchArgument(
            'chatbot_server_url',
            default_value='http://localhost:11434/api/chat',
            description='Backend HTTP endpoint used by chatbot_llm.',
        ),
        DeclareLaunchArgument(
            'start_rqt_console',
            default_value='true',
            description='Launch the full rqt shell for runtime tools.',
        ),
        DeclareLaunchArgument(
            'start_rqt_chat',
            default_value='true',
            description='Launch rqt_chat in passive mode against the migrated stack.',
        ),
        DeclareLaunchArgument(
            'start_robot_speech_debug',
            default_value='true',
            description='Launch a logger that mirrors robot speech into ROS logs.',
        ),
        DeclareLaunchArgument(
            'asr_vosk_enabled',
            default_value='true',
            description='Enable Vosk ASR lifecycle node.',
        ),
        DeclareLaunchArgument(
            'asr_vosk_model_path',
            default_value='/models/vosk-model-small-en-us-0.15',
            description='Absolute path to the Vosk model.',
        ),
        DeclareLaunchArgument(
            'asr_audio_capture_enabled',
            default_value='true',
            description='Launch simple_audio_capture together with asr_vosk.',
        ),
        DeclareLaunchArgument(
            'asr_audio_capture_source_type',
            default_value='pulsesrc',
            description='GStreamer source type for audio capture.',
        ),
        DeclareLaunchArgument(
            'asr_audio_capture_device',
            default_value='',
            description='Optional audio device identifier.',
        ),
        DeclareLaunchArgument(
            'asr_push_to_talk_enabled',
            default_value='true',
            description='Require an explicit Bool gate before ASR listens.',
        ),
        DeclareLaunchArgument(
            'asr_push_to_talk_topic',
            default_value='/asr_vosk/push_to_talk',
            description='Bool topic used to enable or disable listening.',
        ),
    ]

    migration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('nao_chatbot'),
                    'launch',
                    'nao_chatbot_ros4hri_migration.launch.py',
                ]
            )
        ),
        launch_arguments={
            'start_naoqi_driver': LaunchConfiguration('start_naoqi_driver'),
            'nao_ip': LaunchConfiguration('nao_ip'),
            'nao_port': LaunchConfiguration('nao_port'),
            'network_interface': LaunchConfiguration('network_interface'),
            'qi_listen_url': LaunchConfiguration('qi_listen_url'),
            'posture_command_topic': LaunchConfiguration('posture_command_topic'),
            'dialogue_manager_chatbot': LaunchConfiguration('dialogue_manager_chatbot'),
            'dialogue_manager_enable_default_chat': LaunchConfiguration(
                'dialogue_manager_enable_default_chat'
            ),
            'dialogue_manager_default_chat_role': LaunchConfiguration(
                'dialogue_manager_default_chat_role'
            ),
            'dialogue_manager_default_chat_configuration': LaunchConfiguration(
                'dialogue_manager_default_chat_configuration'
            ),
            'chatbot_model': LaunchConfiguration('chatbot_model'),
            'ollama_model': LaunchConfiguration('ollama_model'),
            'chatbot_intent_model': LaunchConfiguration('chatbot_intent_model'),
            'ollama_intent_model': LaunchConfiguration('ollama_intent_model'),
            'chatbot_server_url': LaunchConfiguration('chatbot_server_url'),
            'start_rqt_console': LaunchConfiguration('start_rqt_console'),
            'start_rqt_chat': LaunchConfiguration('start_rqt_chat'),
            'start_robot_speech_debug': LaunchConfiguration(
                'start_robot_speech_debug'
            ),
        }.items(),
    )

    asr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('nao_chatbot'),
                    'launch',
                    'nao_chatbot_asr_only.launch.py',
                ]
            )
        ),
        launch_arguments={
            'asr_vosk_enabled': LaunchConfiguration('asr_vosk_enabled'),
            'asr_vosk_model_path': LaunchConfiguration('asr_vosk_model_path'),
            'asr_audio_capture_enabled': LaunchConfiguration('asr_audio_capture_enabled'),
            'asr_audio_capture_source_type': LaunchConfiguration(
                'asr_audio_capture_source_type'
            ),
            'asr_audio_capture_device': LaunchConfiguration(
                'asr_audio_capture_device'
            ),
            'asr_push_to_talk_enabled': LaunchConfiguration('asr_push_to_talk_enabled'),
            'asr_push_to_talk_topic': LaunchConfiguration('asr_push_to_talk_topic'),
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            migration_launch,
            asr_launch,
        ]
    )
