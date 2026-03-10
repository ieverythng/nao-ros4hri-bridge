from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="false",
        description="Whether to launch naoqi_driver together with chatbot stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="172.26.112.62",
        description="NAO robot IP address used by naoqi_driver and posture nodes.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface for naoqi_driver.",
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL used by naoqi_driver audio extractor.",
    )

    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value="true",
        description="Whether to start rqt_chat UI.",
    )
    rqt_chat_start_delay_sec_arg = DeclareLaunchArgument(
        "rqt_chat_start_delay_sec",
        default_value="1.5",
        description="Delay before launching rqt_chat.",
    )

    bridge_input_speech_topic_arg = DeclareLaunchArgument(
        "bridge_input_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description="LiveSpeech input consumed by dialogue_manager.",
    )

    asr_vosk_enabled_arg = DeclareLaunchArgument(
        "asr_vosk_enabled",
        default_value="false",
        description="Enable local Vosk ASR node.",
    )
    asr_output_speech_topic_arg = DeclareLaunchArgument(
        "asr_output_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description="LiveSpeech topic where local ASR publishes transcripts.",
    )
    asr_microphone_topic_arg = DeclareLaunchArgument(
        "asr_microphone_topic",
        default_value="/laptop/microphone0",
        description="Audio topic consumed by the ASR lifecycle node.",
    )
    asr_speech_locale_arg = DeclareLaunchArgument(
        "asr_speech_locale",
        default_value="en_US",
        description="Locale string published in LiveSpeech messages.",
    )
    asr_vosk_model_path_arg = DeclareLaunchArgument(
        "asr_vosk_model_path",
        default_value="",
        description="Absolute path to Vosk model directory.",
    )
    asr_sample_rate_hz_arg = DeclareLaunchArgument(
        "asr_sample_rate_hz",
        default_value="16000",
        description="Preferred sample rate for local ASR.",
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
        default_value="false",
        description="Require an explicit Bool gate before ASR listens.",
    )
    asr_push_to_talk_topic_arg = DeclareLaunchArgument(
        "asr_push_to_talk_topic",
        default_value="/asr_vosk/push_to_talk",
        description="Bool topic used to enable/disable listening in push-to-talk mode.",
    )
    asr_robot_speech_topic_arg = DeclareLaunchArgument(
        "asr_robot_speech_topic",
        default_value="/speech",
        description="Robot speech topic used by dialogue/say components.",
    )
    asr_audio_capture_enabled_arg = DeclareLaunchArgument(
        "asr_audio_capture_enabled",
        default_value="false",
        description="Launch simple_audio_capture node for ASR audio_topic backend.",
    )
    asr_audio_capture_source_type_arg = DeclareLaunchArgument(
        "asr_audio_capture_source_type",
        default_value="pulsesrc",
        description='GStreamer source type for audio capture ("pulsesrc" or "alsasrc").',
    )
    asr_audio_capture_device_arg = DeclareLaunchArgument(
        "asr_audio_capture_device",
        default_value="",
        description="Optional audio capture device identifier.",
    )
    asr_audio_capture_sample_rate_arg = DeclareLaunchArgument(
        "asr_audio_capture_sample_rate",
        default_value="16000",
        description="Audio capture sample rate in Hz.",
    )
    asr_audio_capture_channels_arg = DeclareLaunchArgument(
        "asr_audio_capture_channels",
        default_value="1",
        description="Audio capture channel count.",
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
    dialogue_accept_incremental_speech_arg = DeclareLaunchArgument(
        "dialogue_accept_incremental_speech",
        default_value="false",
        description="Allow dialogue_manager to consume incremental ASR hypotheses.",
    )
    dialogue_ignore_user_speech_while_busy_arg = DeclareLaunchArgument(
        "dialogue_ignore_user_speech_while_busy",
        default_value="true",
        description="Ignore new ASR input while a user turn is already awaiting an assistant response.",
    )
    dialogue_user_turn_holdoff_sec_arg = DeclareLaunchArgument(
        "dialogue_user_turn_holdoff_sec",
        default_value="0.6",
        description="Seconds to buffer/merge consecutive ASR finals before forwarding one user turn.",
    )
    dialogue_user_turn_min_chars_arg = DeclareLaunchArgument(
        "dialogue_user_turn_min_chars",
        default_value="2",
        description="Minimum character length accepted for forwarded ASR user turns.",
    )
    dialogue_user_turn_min_words_arg = DeclareLaunchArgument(
        "dialogue_user_turn_min_words",
        default_value="1",
        description="Minimum word count accepted for forwarded ASR user turns.",
    )

    mission_mode_arg = DeclareLaunchArgument(
        "mission_mode",
        default_value="rules",
        description='Mission controller mode: "rules" or "backend".',
    )
    backend_fallback_to_rules_arg = DeclareLaunchArgument(
        "backend_fallback_to_rules",
        default_value="false",
        description="Enable fallback rule response when backend/chat times out.",
    )
    backend_response_timeout_sec_arg = DeclareLaunchArgument(
        "backend_response_timeout_sec",
        default_value="30.0",
        description="Seconds to wait for backend/chat response.",
    )
    backend_execute_posture_after_response_arg = DeclareLaunchArgument(
        "backend_execute_posture_after_response",
        default_value="true",
        description="Execute posture intent after backend/chat response in backend mode.",
    )
    backend_posture_from_response_enabled_arg = DeclareLaunchArgument(
        "backend_posture_from_response_enabled",
        default_value="false",
        description="Derive posture intent from backend response text.",
    )
    use_chat_skill_arg = DeclareLaunchArgument(
        "use_chat_skill",
        default_value="true",
        description="Use `/skill/chat` action client from mission_controller.",
    )
    chat_skill_action_arg = DeclareLaunchArgument(
        "chat_skill_action",
        default_value="/skill/chat",
        description="Chat skill action name.",
    )
    chat_skill_dispatch_wait_sec_arg = DeclareLaunchArgument(
        "chat_skill_dispatch_wait_sec",
        default_value="1.0",
        description="Wait timeout before chat skill dispatch fallback.",
    )
    chat_history_max_entries_arg = DeclareLaunchArgument(
        "chat_history_max_entries",
        default_value="24",
        description="Maximum serialized chat history entries kept by mission controller.",
    )
    chat_skill_server_enabled_arg = DeclareLaunchArgument(
        "chat_skill_server_enabled",
        default_value="true",
        description="Launch `/skill/chat` action server.",
    )

    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Topic used for posture bridge fallback commands.",
    )
    use_posture_skill_arg = DeclareLaunchArgument(
        "use_posture_skill",
        default_value="true",
        description="Use posture action client from mission controller.",
    )
    posture_skill_action_arg = DeclareLaunchArgument(
        "posture_skill_action",
        default_value="/skill/do_posture",
        description="Posture action name.",
    )
    posture_skill_speed_arg = DeclareLaunchArgument(
        "posture_skill_speed",
        default_value="0.9",
        description="Default posture speed sent by mission controller.",
    )
    posture_skill_dispatch_wait_sec_arg = DeclareLaunchArgument(
        "posture_skill_dispatch_wait_sec",
        default_value="1.0",
        description="Wait timeout before posture skill dispatch fallback.",
    )
    use_head_motion_skill_arg = DeclareLaunchArgument(
        "use_head_motion_skill",
        default_value="true",
        description="Use head-motion action client from mission controller.",
    )
    head_motion_skill_action_arg = DeclareLaunchArgument(
        "head_motion_skill_action",
        default_value="/skill/do_head_motion",
        description="Head-motion action name.",
    )
    head_motion_skill_speed_arg = DeclareLaunchArgument(
        "head_motion_skill_speed",
        default_value="0.25",
        description="Default head-motion speed sent by mission controller.",
    )
    head_motion_skill_dispatch_wait_sec_arg = DeclareLaunchArgument(
        "head_motion_skill_dispatch_wait_sec",
        default_value="1.0",
        description="Wait timeout before head-motion skill dispatch fallback.",
    )
    head_motion_fallback_to_joint_topic_arg = DeclareLaunchArgument(
        "head_motion_fallback_to_joint_topic",
        default_value="true",
        description="Allow mission controller head-motion fallback to `/joint_angles`.",
    )
    head_motion_joint_angles_topic_arg = DeclareLaunchArgument(
        "head_motion_joint_angles_topic",
        default_value="/joint_angles",
        description="Joint-angle topic for mission fallback and head-motion skill server.",
    )

    posture_skill_server_enabled_arg = DeclareLaunchArgument(
        "posture_skill_server_enabled",
        default_value="true",
        description="Launch posture skill action server.",
    )
    posture_skill_server_default_speed_arg = DeclareLaunchArgument(
        "posture_skill_server_default_speed",
        default_value="0.8",
        description="Default speed inside posture skill server.",
    )
    posture_skill_server_fallback_to_topic_arg = DeclareLaunchArgument(
        "posture_skill_server_fallback_to_topic",
        default_value="true",
        description="Allow posture skill server to fallback to posture command topic.",
    )
    head_motion_skill_server_enabled_arg = DeclareLaunchArgument(
        "head_motion_skill_server_enabled",
        default_value="true",
        description="Launch head-motion skill action server.",
    )
    head_motion_skill_server_default_speed_arg = DeclareLaunchArgument(
        "head_motion_skill_server_default_speed",
        default_value="0.2",
        description="Default speed inside head-motion skill server.",
    )
    posture_bridge_enabled_arg = DeclareLaunchArgument(
        "posture_bridge_enabled",
        default_value="true",
        description="Launch posture bridge node for topic-fallback execution.",
    )
    posture_bridge_speed_arg = DeclareLaunchArgument(
        "posture_bridge_speed",
        default_value="0.8",
        description="Default posture speed for posture bridge fallback node.",
    )

    use_say_skill_arg = DeclareLaunchArgument(
        "use_say_skill",
        default_value="true",
        description="Use `/skill/say` action client from dialogue_manager.",
    )
    say_skill_action_arg = DeclareLaunchArgument(
        "say_skill_action",
        default_value="/skill/say",
        description="Say skill action name.",
    )
    say_skill_language_arg = DeclareLaunchArgument(
        "say_skill_language",
        default_value="en-US",
        description="Default language passed to `/skill/say` goals.",
    )
    say_skill_volume_arg = DeclareLaunchArgument(
        "say_skill_volume",
        default_value="1.0",
        description="Default volume passed to `/skill/say` goals.",
    )
    say_skill_dispatch_wait_sec_arg = DeclareLaunchArgument(
        "say_skill_dispatch_wait_sec",
        default_value="0.8",
        description="Wait timeout before say skill dispatch fallback.",
    )
    dialogue_publish_speech_topic_arg = DeclareLaunchArgument(
        "dialogue_publish_speech_topic",
        default_value="true",
        description="Publish assistant speech to robot speech topic from dialogue manager.",
    )
    dialogue_fallback_publish_speech_topic_arg = DeclareLaunchArgument(
        "dialogue_fallback_publish_speech_topic",
        default_value="true",
        description="Fallback to speech topic if say skill is unavailable/failed.",
    )
    say_skill_server_enabled_arg = DeclareLaunchArgument(
        "say_skill_server_enabled",
        default_value="true",
        description="Launch `/skill/say` action server.",
    )
    say_skill_tts_action_name_arg = DeclareLaunchArgument(
        "say_skill_tts_action_name",
        default_value="/tts_engine/tts",
        description="Underlying TTS action endpoint used by say skill server.",
    )
    say_skill_server_fallback_to_topic_arg = DeclareLaunchArgument(
        "say_skill_server_fallback_to_topic",
        default_value="true",
        description="Fallback to speech topic if TTS action is unavailable.",
    )
    say_skill_server_publish_speech_topic_arg = DeclareLaunchArgument(
        "say_skill_server_publish_speech_topic",
        default_value="false",
        description="Publish `/speech` directly from say skill server.",
    )

    ollama_enabled_arg = DeclareLaunchArgument(
        "ollama_enabled",
        default_value="false",
        description="Enable Ollama requests in the `/skill/chat` action server.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="llama3.2:1b",
        description="Ollama model name.",
    )
    ollama_intent_detection_mode_arg = DeclareLaunchArgument(
        "ollama_intent_detection_mode",
        default_value="llm_with_rules_fallback",
        description='Intent extraction mode in `/skill/chat`: "rules", "llm", or "llm_with_rules_fallback".',
    )
    ollama_intent_model_arg = DeclareLaunchArgument(
        "ollama_intent_model",
        default_value="",
        description="Optional dedicated model for intent extraction (empty uses ollama_model).",
    )
    ollama_intent_request_timeout_sec_arg = DeclareLaunchArgument(
        "ollama_intent_request_timeout_sec",
        default_value="10.0",
        description="Intent extraction timeout for `/skill/chat` Ollama requests.",
    )
    ollama_prompt_pack_path_arg = DeclareLaunchArgument(
        "ollama_prompt_pack_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("nao_chatbot"), "config", "chat_prompt_pack.yaml"]
        ),
        description="YAML prompt-pack path loaded by `/skill/chat` server.",
    )
    ollama_use_skill_catalog_arg = DeclareLaunchArgument(
        "ollama_use_skill_catalog",
        default_value="true",
        description="Inject compact skill catalog into chat/intent prompts.",
    )
    ollama_skill_catalog_packages_arg = DeclareLaunchArgument(
        "ollama_skill_catalog_packages",
        default_value="communication_skills,nao_skills",
        description="CSV package allowlist for skill-catalog extraction.",
    )
    ollama_skill_catalog_max_entries_arg = DeclareLaunchArgument(
        "ollama_skill_catalog_max_entries",
        default_value="16",
        description="Maximum number of skill entries injected into prompts.",
    )
    ollama_skill_catalog_max_chars_arg = DeclareLaunchArgument(
        "ollama_skill_catalog_max_chars",
        default_value="3000",
        description="Maximum serialized skill-catalog size in characters.",
    )

    start_naoqi_driver = LaunchConfiguration("start_naoqi_driver")
    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")
    network_interface = LaunchConfiguration("network_interface")
    qi_listen_url = LaunchConfiguration("qi_listen_url")
    start_rqt_chat = LaunchConfiguration("start_rqt_chat")
    rqt_chat_start_delay_sec = LaunchConfiguration("rqt_chat_start_delay_sec")

    bridge_input_speech_topic = LaunchConfiguration("bridge_input_speech_topic")
    asr_vosk_enabled = LaunchConfiguration("asr_vosk_enabled")
    asr_output_speech_topic = LaunchConfiguration("asr_output_speech_topic")
    asr_microphone_topic = LaunchConfiguration("asr_microphone_topic")
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
    asr_robot_speech_topic = LaunchConfiguration("asr_robot_speech_topic")
    asr_audio_capture_enabled = LaunchConfiguration("asr_audio_capture_enabled")
    asr_audio_capture_source_type = LaunchConfiguration("asr_audio_capture_source_type")
    asr_audio_capture_device = LaunchConfiguration("asr_audio_capture_device")
    asr_audio_capture_sample_rate = LaunchConfiguration("asr_audio_capture_sample_rate")
    asr_audio_capture_channels = LaunchConfiguration("asr_audio_capture_channels")
    asr_audio_capture_sample_format = LaunchConfiguration(
        "asr_audio_capture_sample_format"
    )
    asr_audio_capture_chunk_size = LaunchConfiguration("asr_audio_capture_chunk_size")

    mission_mode = LaunchConfiguration("mission_mode")
    backend_fallback_to_rules = LaunchConfiguration("backend_fallback_to_rules")
    backend_response_timeout_sec = LaunchConfiguration("backend_response_timeout_sec")
    backend_execute_posture_after_response = LaunchConfiguration(
        "backend_execute_posture_after_response"
    )
    backend_posture_from_response_enabled = LaunchConfiguration(
        "backend_posture_from_response_enabled"
    )
    use_chat_skill = LaunchConfiguration("use_chat_skill")
    chat_skill_action = LaunchConfiguration("chat_skill_action")
    chat_skill_dispatch_wait_sec = LaunchConfiguration("chat_skill_dispatch_wait_sec")
    chat_history_max_entries = LaunchConfiguration("chat_history_max_entries")
    chat_skill_server_enabled = LaunchConfiguration("chat_skill_server_enabled")

    posture_command_topic = LaunchConfiguration("posture_command_topic")
    use_posture_skill = LaunchConfiguration("use_posture_skill")
    posture_skill_action = LaunchConfiguration("posture_skill_action")
    posture_skill_speed = LaunchConfiguration("posture_skill_speed")
    posture_skill_dispatch_wait_sec = LaunchConfiguration("posture_skill_dispatch_wait_sec")
    use_head_motion_skill = LaunchConfiguration("use_head_motion_skill")
    head_motion_skill_action = LaunchConfiguration("head_motion_skill_action")
    head_motion_skill_speed = LaunchConfiguration("head_motion_skill_speed")
    head_motion_skill_dispatch_wait_sec = LaunchConfiguration(
        "head_motion_skill_dispatch_wait_sec"
    )
    head_motion_fallback_to_joint_topic = LaunchConfiguration(
        "head_motion_fallback_to_joint_topic"
    )
    head_motion_joint_angles_topic = LaunchConfiguration(
        "head_motion_joint_angles_topic"
    )

    posture_skill_server_enabled = LaunchConfiguration("posture_skill_server_enabled")
    posture_skill_server_default_speed = LaunchConfiguration(
        "posture_skill_server_default_speed"
    )
    posture_skill_server_fallback_to_topic = LaunchConfiguration(
        "posture_skill_server_fallback_to_topic"
    )
    head_motion_skill_server_enabled = LaunchConfiguration(
        "head_motion_skill_server_enabled"
    )
    head_motion_skill_server_default_speed = LaunchConfiguration(
        "head_motion_skill_server_default_speed"
    )
    posture_bridge_enabled = LaunchConfiguration("posture_bridge_enabled")
    posture_bridge_speed = LaunchConfiguration("posture_bridge_speed")

    use_say_skill = LaunchConfiguration("use_say_skill")
    say_skill_action = LaunchConfiguration("say_skill_action")
    say_skill_language = LaunchConfiguration("say_skill_language")
    say_skill_volume = LaunchConfiguration("say_skill_volume")
    say_skill_dispatch_wait_sec = LaunchConfiguration("say_skill_dispatch_wait_sec")
    dialogue_publish_speech_topic = LaunchConfiguration("dialogue_publish_speech_topic")
    dialogue_fallback_publish_speech_topic = LaunchConfiguration(
        "dialogue_fallback_publish_speech_topic"
    )
    dialogue_accept_incremental_speech = LaunchConfiguration(
        "dialogue_accept_incremental_speech"
    )
    dialogue_ignore_user_speech_while_busy = LaunchConfiguration(
        "dialogue_ignore_user_speech_while_busy"
    )
    dialogue_user_turn_holdoff_sec = LaunchConfiguration(
        "dialogue_user_turn_holdoff_sec"
    )
    dialogue_user_turn_min_chars = LaunchConfiguration(
        "dialogue_user_turn_min_chars"
    )
    dialogue_user_turn_min_words = LaunchConfiguration(
        "dialogue_user_turn_min_words"
    )
    say_skill_server_enabled = LaunchConfiguration("say_skill_server_enabled")
    say_skill_tts_action_name = LaunchConfiguration("say_skill_tts_action_name")
    say_skill_server_fallback_to_topic = LaunchConfiguration(
        "say_skill_server_fallback_to_topic"
    )
    say_skill_server_publish_speech_topic = LaunchConfiguration(
        "say_skill_server_publish_speech_topic"
    )

    ollama_enabled = LaunchConfiguration("ollama_enabled")
    ollama_model = LaunchConfiguration("ollama_model")
    ollama_intent_detection_mode = LaunchConfiguration("ollama_intent_detection_mode")
    ollama_intent_model = LaunchConfiguration("ollama_intent_model")
    ollama_intent_request_timeout_sec = LaunchConfiguration(
        "ollama_intent_request_timeout_sec"
    )
    ollama_prompt_pack_path = LaunchConfiguration("ollama_prompt_pack_path")
    ollama_use_skill_catalog = LaunchConfiguration("ollama_use_skill_catalog")
    ollama_skill_catalog_packages = LaunchConfiguration(
        "ollama_skill_catalog_packages"
    )
    ollama_skill_catalog_max_entries = LaunchConfiguration(
        "ollama_skill_catalog_max_entries"
    )
    ollama_skill_catalog_max_chars = LaunchConfiguration(
        "ollama_skill_catalog_max_chars"
    )

    naoqi_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("naoqi_driver"), "launch", "naoqi_driver.launch.py"]
            )
        ),
        condition=IfCondition(start_naoqi_driver),
        launch_arguments={
            "nao_ip": nao_ip,
            "nao_port": nao_port,
            "network_interface": network_interface,
            "qi_listen_url": qi_listen_url,
        }.items(),
    )

    dialogue_manager = Node(
        package="dialogue_manager",
        executable="dialogue_manager_node",
        name="dialogue_manager",
        output="screen",
        parameters=[
            {
                "input_speech_topic": bridge_input_speech_topic,
                "naoqi_speech_topic": asr_robot_speech_topic,
                "use_say_skill": use_say_skill,
                "say_skill_action": say_skill_action,
                "say_skill_language": say_skill_language,
                "say_skill_volume": say_skill_volume,
                "say_skill_dispatch_wait_sec": say_skill_dispatch_wait_sec,
                "also_publish_speech_topic": dialogue_publish_speech_topic,
                "fallback_publish_speech_topic": dialogue_fallback_publish_speech_topic,
                "accept_incremental_speech": dialogue_accept_incremental_speech,
                "ignore_user_speech_while_busy": dialogue_ignore_user_speech_while_busy,
                "user_turn_holdoff_sec": dialogue_user_turn_holdoff_sec,
                "user_turn_min_chars": dialogue_user_turn_min_chars,
                "user_turn_min_words": dialogue_user_turn_min_words,
            }
        ],
    )

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

    mission = Node(
        package="nao_chatbot",
        executable="mission_controller_node",
        name="mission_controller",
        output="screen",
        parameters=[
            {
                "mode": mission_mode,
                "backend_fallback_to_rules": backend_fallback_to_rules,
                "backend_response_timeout_sec": backend_response_timeout_sec,
                "backend_execute_posture_after_response": backend_execute_posture_after_response,
                "backend_posture_from_response_enabled": backend_posture_from_response_enabled,
                "use_chat_skill": use_chat_skill,
                "chat_skill_action": chat_skill_action,
                "chat_skill_dispatch_wait_sec": chat_skill_dispatch_wait_sec,
                "chat_history_max_entries": chat_history_max_entries,
                "posture_command_topic": posture_command_topic,
                "use_posture_skill": use_posture_skill,
                "posture_skill_action": posture_skill_action,
                "posture_skill_speed": posture_skill_speed,
                "posture_skill_dispatch_wait_sec": posture_skill_dispatch_wait_sec,
                "use_head_motion_skill": use_head_motion_skill,
                "head_motion_skill_action": head_motion_skill_action,
                "head_motion_skill_speed": head_motion_skill_speed,
                "head_motion_skill_dispatch_wait_sec": head_motion_skill_dispatch_wait_sec,
                "head_motion_joint_angles_topic": head_motion_joint_angles_topic,
                "head_motion_fallback_to_joint_topic": head_motion_fallback_to_joint_topic,
            }
        ],
    )

    chat_skill_server = Node(
        package="nao_chatbot",
        executable="ollama_chatbot_node",
        name="ollama_chatbot",
        output="screen",
        condition=IfCondition(chat_skill_server_enabled),
        parameters=[
            {
                "action_name": chat_skill_action,
                "enabled": ollama_enabled,
                "model": ollama_model,
                "intent_detection_mode": ollama_intent_detection_mode,
                "intent_model": ollama_intent_model,
                "intent_request_timeout_sec": ollama_intent_request_timeout_sec,
                "prompt_pack_path": ollama_prompt_pack_path,
                "use_skill_catalog": ollama_use_skill_catalog,
                "skill_catalog_packages": ollama_skill_catalog_packages,
                "skill_catalog_max_entries": ollama_skill_catalog_max_entries,
                "skill_catalog_max_chars": ollama_skill_catalog_max_chars,
            }
        ],
    )

    posture_bridge = Node(
        package="nao_skill_servers",
        executable="nao_posture_bridge_node",
        name="nao_posture_bridge",
        output="screen",
        condition=IfCondition(posture_bridge_enabled),
        parameters=[
            {
                "posture_command_topic": posture_command_topic,
                "nao_ip": nao_ip,
                "nao_port": nao_port,
                "posture_speed": posture_bridge_speed,
            }
        ],
    )

    posture_skill_server = Node(
        package="nao_skill_servers",
        executable="posture_skill_server_node",
        name="posture_skill_server",
        output="screen",
        condition=IfCondition(posture_skill_server_enabled),
        parameters=[
            {
                "nao_ip": nao_ip,
                "nao_port": nao_port,
                "action_name": posture_skill_action,
                "default_speed": posture_skill_server_default_speed,
                "fallback_to_posture_topic": posture_skill_server_fallback_to_topic,
                "posture_command_topic": posture_command_topic,
            }
        ],
    )

    head_motion_skill_server = Node(
        package="nao_skill_servers",
        executable="head_motion_skill_server_node",
        name="head_motion_skill_server",
        output="screen",
        condition=IfCondition(head_motion_skill_server_enabled),
        parameters=[
            {
                "action_name": head_motion_skill_action,
                "default_speed": head_motion_skill_server_default_speed,
                "joint_angles_topic": head_motion_joint_angles_topic,
            }
        ],
    )

    say_skill_server = Node(
        package="nao_skill_servers",
        executable="say_skill_server_node",
        name="say_skill_server",
        output="screen",
        condition=IfCondition(say_skill_server_enabled),
        parameters=[
            {
                "action_name": say_skill_action,
                "tts_action_name": say_skill_tts_action_name,
                "naoqi_speech_topic": asr_robot_speech_topic,
                "fallback_to_speech_topic": say_skill_server_fallback_to_topic,
                "also_publish_speech_topic": say_skill_server_publish_speech_topic,
            }
        ],
    )

    rqt_chat_process = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_gui",
            "rqt_gui",
            "--standalone",
            "rqt_chat.chat.ChatPlugin",
        ],
        output="screen",
    )
    rqt_chat = TimerAction(
        period=rqt_chat_start_delay_sec,
        actions=[rqt_chat_process],
        condition=IfCondition(start_rqt_chat),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            start_rqt_chat_arg,
            rqt_chat_start_delay_sec_arg,
            bridge_input_speech_topic_arg,
            asr_vosk_enabled_arg,
            asr_output_speech_topic_arg,
            asr_microphone_topic_arg,
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
            asr_robot_speech_topic_arg,
            asr_audio_capture_enabled_arg,
            asr_audio_capture_source_type_arg,
            asr_audio_capture_device_arg,
            asr_audio_capture_sample_rate_arg,
            asr_audio_capture_channels_arg,
            asr_audio_capture_sample_format_arg,
            asr_audio_capture_chunk_size_arg,
            dialogue_accept_incremental_speech_arg,
            dialogue_ignore_user_speech_while_busy_arg,
            dialogue_user_turn_holdoff_sec_arg,
            dialogue_user_turn_min_chars_arg,
            dialogue_user_turn_min_words_arg,
            mission_mode_arg,
            backend_fallback_to_rules_arg,
            backend_response_timeout_sec_arg,
            backend_execute_posture_after_response_arg,
            backend_posture_from_response_enabled_arg,
            use_chat_skill_arg,
            chat_skill_action_arg,
            chat_skill_dispatch_wait_sec_arg,
            chat_history_max_entries_arg,
            chat_skill_server_enabled_arg,
            posture_command_topic_arg,
            use_posture_skill_arg,
            posture_skill_action_arg,
            posture_skill_speed_arg,
            posture_skill_dispatch_wait_sec_arg,
            use_head_motion_skill_arg,
            head_motion_skill_action_arg,
            head_motion_skill_speed_arg,
            head_motion_skill_dispatch_wait_sec_arg,
            head_motion_fallback_to_joint_topic_arg,
            head_motion_joint_angles_topic_arg,
            posture_skill_server_enabled_arg,
            posture_skill_server_default_speed_arg,
            posture_skill_server_fallback_to_topic_arg,
            head_motion_skill_server_enabled_arg,
            head_motion_skill_server_default_speed_arg,
            posture_bridge_enabled_arg,
            posture_bridge_speed_arg,
            use_say_skill_arg,
            say_skill_action_arg,
            say_skill_language_arg,
            say_skill_volume_arg,
            say_skill_dispatch_wait_sec_arg,
            dialogue_publish_speech_topic_arg,
            dialogue_fallback_publish_speech_topic_arg,
            say_skill_server_enabled_arg,
            say_skill_tts_action_name_arg,
            say_skill_server_fallback_to_topic_arg,
            say_skill_server_publish_speech_topic_arg,
            ollama_enabled_arg,
            ollama_model_arg,
            ollama_intent_detection_mode_arg,
            ollama_intent_model_arg,
            ollama_intent_request_timeout_sec_arg,
            ollama_prompt_pack_path_arg,
            ollama_use_skill_catalog_arg,
            ollama_skill_catalog_packages_arg,
            ollama_skill_catalog_max_entries_arg,
            ollama_skill_catalog_max_chars_arg,
            naoqi_driver,
            dialogue_manager,
            asr_audio_capture,
            asr_vosk,
            asr_vosk_configure,
            asr_vosk_activate,
            posture_skill_server,
            head_motion_skill_server,
            say_skill_server,
            chat_skill_server,
            mission,
            posture_bridge,
            rqt_chat,
        ]
    )
