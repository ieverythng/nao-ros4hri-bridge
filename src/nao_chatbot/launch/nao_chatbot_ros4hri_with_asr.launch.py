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
            'start_knowledge_core',
            default_value='true',
            description='Optionally launch KnowledgeCore together with the migrated stack when it is installed in the environment.',
        ),
        DeclareLaunchArgument(
            'start_nao_robot',
            default_value='false',
            description='Launch the packaged nao_robot bring-up for real-robot camera validation.',
        ),
        DeclareLaunchArgument(
            'start_nao_robot_hri_visualization',
            default_value='true',
            description='Launch hri_visualization together with nao_robot.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Launch rviz2 using the packaged nao_robot RViz config.',
        ),
        DeclareLaunchArgument(
            'start_object_detection',
            default_value='false',
            description='Optionally launch the configured external object detector backend.',
        ),
        DeclareLaunchArgument(
            'object_detection_backend',
            default_value='emorobcare_cv',
            description='External detector backend to launch: emorobcare_cv or yolo_ros.',
        ),
        DeclareLaunchArgument(
            'start_scene_grounding',
            default_value='false',
            description='Launch the local object-grounding node.',
        ),
        DeclareLaunchArgument(
            'start_interaction_sim',
            default_value='false',
            description='Optionally launch the official interaction_sim support launch for simulator testing.',
        ),
        DeclareLaunchArgument(
            'start_interaction_sim_perception',
            default_value='true',
            description='Launch the interaction_sim webcam/person/emotion perception components.',
        ),
        DeclareLaunchArgument(
            'start_interaction_sim_tools',
            default_value='true',
            description='Launch interaction_sim support tools such as rosbridge and ui_server.',
        ),
        DeclareLaunchArgument(
            'start_interaction_sim_ui',
            default_value='false',
            description='Start ui_server together with interaction_sim support tools.',
        ),
        DeclareLaunchArgument(
            'interaction_sim_gscam_config',
            default_value='v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert',
            description='GStreamer pipeline used by gscam when interaction_sim support is enabled.',
        ),
        DeclareLaunchArgument(
            'object_detection_namespace',
            default_value='yolo',
            description='Namespace used by the external detector stack.',
        ),
        DeclareLaunchArgument(
            'object_detection_model',
            default_value='yolov8n.pt',
            description='Detector model forwarded to yolo_ros.',
        ),
        DeclareLaunchArgument(
            'object_detection_device',
            default_value='cpu',
            description='Detector device forwarded to yolo_ros.',
        ),
        DeclareLaunchArgument(
            'object_detection_threshold',
            default_value='0.35',
            description='Detector score threshold forwarded to yolo_ros and scene grounding.',
        ),
        DeclareLaunchArgument(
            'object_detection_input_image_topic',
            default_value='/nao_robot/camera/front/image_raw',
            description='Image topic forwarded to yolo_ros.',
        ),
        DeclareLaunchArgument(
            'object_detection_image_reliability',
            default_value='2',
            description='Image QoS reliability forwarded to yolo_ros.',
        ),
        DeclareLaunchArgument(
            'scene_grounding_detector_topic',
            default_value='/detected_objects',
            description='Detection topic consumed by nao_scene_grounding.',
        ),
        DeclareLaunchArgument(
            'scene_grounding_summary_topic',
            default_value='/scene/summary',
            description='Summary topic published by nao_scene_grounding.',
        ),
        DeclareLaunchArgument(
            'scene_grounding_allowed_labels',
            default_value='bottle,cup,book,cell phone,backpack,remote,laptop,keyboard,mouse,chair,blueberry,corn,pear,tomato,zucchini',
            description='Comma-separated object labels that should be grounded.',
        ),
        DeclareLaunchArgument(
            'scene_grounding_knowledge_lifespan_sec',
            default_value='4.0',
            description='KnowledgeCore lifespan for grounded object facts.',
        ),
        DeclareLaunchArgument(
            'scene_grounding_knowledge_refresh_interval_sec',
            default_value='1.0',
            description='Refresh interval for grounded object facts.',
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
            description='Launch a single remapped rqt shell; when interaction_sim is enabled it loads the official simulator perspective.',
        ),
        DeclareLaunchArgument(
            'start_rqt_chat',
            default_value='false',
            description='Optionally launch a separate rqt_chat window remapped onto the debug TTS action when the simulator perspective is not in use.',
        ),
        DeclareLaunchArgument(
            'debug_tts_action_name',
            default_value='/debug/say',
            description='Debug-only TTS action used for rqt_chat and operator monitoring.',
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
            'start_nao_robot': LaunchConfiguration('start_nao_robot'),
            'start_nao_robot_hri_visualization': LaunchConfiguration('start_nao_robot_hri_visualization'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'start_object_detection': LaunchConfiguration('start_object_detection'),
            'object_detection_backend': LaunchConfiguration('object_detection_backend'),
            'start_scene_grounding': LaunchConfiguration('start_scene_grounding'),
            'start_knowledge_core': LaunchConfiguration('start_knowledge_core'),
            'start_interaction_sim': LaunchConfiguration('start_interaction_sim'),
            'start_interaction_sim_perception': LaunchConfiguration(
                'start_interaction_sim_perception'
            ),
            'start_interaction_sim_tools': LaunchConfiguration(
                'start_interaction_sim_tools'
            ),
            'start_interaction_sim_ui': LaunchConfiguration('start_interaction_sim_ui'),
            'nao_ip': LaunchConfiguration('nao_ip'),
            'nao_port': LaunchConfiguration('nao_port'),
            'network_interface': LaunchConfiguration('network_interface'),
            'qi_listen_url': LaunchConfiguration('qi_listen_url'),
            'interaction_sim_gscam_config': LaunchConfiguration(
                'interaction_sim_gscam_config'
            ),
            'object_detection_namespace': LaunchConfiguration(
                'object_detection_namespace'
            ),
            'object_detection_model': LaunchConfiguration('object_detection_model'),
            'object_detection_device': LaunchConfiguration('object_detection_device'),
            'object_detection_threshold': LaunchConfiguration(
                'object_detection_threshold'
            ),
            'object_detection_input_image_topic': LaunchConfiguration(
                'object_detection_input_image_topic'
            ),
            'object_detection_image_reliability': LaunchConfiguration(
                'object_detection_image_reliability'
            ),
            'scene_grounding_detector_topic': LaunchConfiguration(
                'scene_grounding_detector_topic'
            ),
            'scene_grounding_summary_topic': LaunchConfiguration(
                'scene_grounding_summary_topic'
            ),
            'scene_grounding_allowed_labels': LaunchConfiguration(
                'scene_grounding_allowed_labels'
            ),
            'scene_grounding_knowledge_lifespan_sec': LaunchConfiguration(
                'scene_grounding_knowledge_lifespan_sec'
            ),
            'scene_grounding_knowledge_refresh_interval_sec': LaunchConfiguration(
                'scene_grounding_knowledge_refresh_interval_sec'
            ),
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
            'debug_tts_action_name': LaunchConfiguration('debug_tts_action_name'),
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
