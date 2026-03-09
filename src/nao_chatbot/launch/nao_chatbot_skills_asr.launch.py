from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="true",
        description="Whether to launch naoqi_driver.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="172.26.112.62",
        description="NAO robot IP.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface used by naoqi_driver.",
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL for naoqi_driver.",
    )
    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value="true",
        description="Start rqt_chat UI.",
    )
    mission_mode_arg = DeclareLaunchArgument(
        "mission_mode",
        default_value="backend",
        description='Mission controller mode: "rules" or "backend".',
    )
    ollama_enabled_arg = DeclareLaunchArgument(
        "ollama_enabled",
        default_value="true",
        description="Enable Ollama-backed `/skill/chat` action server.",
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
    posture_skill_speed_arg = DeclareLaunchArgument(
        "posture_skill_speed",
        default_value="0.9",
        description="Posture skill speed sent by mission controller.",
    )
    head_motion_skill_speed_arg = DeclareLaunchArgument(
        "head_motion_skill_speed",
        default_value="0.25",
        description="Head-motion skill speed sent by mission controller.",
    )
    posture_skill_server_fallback_to_topic_arg = DeclareLaunchArgument(
        "posture_skill_server_fallback_to_topic",
        default_value="true",
        description="Allow skill server topic fallback when direct NAOqi is unavailable.",
    )
    asr_vosk_enabled_arg = DeclareLaunchArgument(
        "asr_vosk_enabled",
        default_value="true",
        description="Enable local Vosk ASR node.",
    )
    asr_microphone_topic_arg = DeclareLaunchArgument(
        "asr_microphone_topic",
        default_value="/laptop/microphone0",
        description="Audio topic consumed by asr_vosk.",
    )
    asr_output_speech_topic_arg = DeclareLaunchArgument(
        "asr_output_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description="LiveSpeech topic where local ASR publishes transcripts.",
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
        description="Microphone sample rate for asr_vosk.",
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
    bridge_input_speech_topic_arg = DeclareLaunchArgument(
        "bridge_input_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description=(
            "Speech topic consumed by dialogue_manager. Keep default for asr_vosk, "
            "override for robot-mic ASR sources."
        ),
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

    start_naoqi_driver = LaunchConfiguration("start_naoqi_driver")
    nao_ip = LaunchConfiguration("nao_ip")
    nao_port = LaunchConfiguration("nao_port")
    network_interface = LaunchConfiguration("network_interface")
    qi_listen_url = LaunchConfiguration("qi_listen_url")
    start_rqt_chat = LaunchConfiguration("start_rqt_chat")
    mission_mode = LaunchConfiguration("mission_mode")
    ollama_enabled = LaunchConfiguration("ollama_enabled")
    ollama_model = LaunchConfiguration("ollama_model")
    ollama_intent_detection_mode = LaunchConfiguration("ollama_intent_detection_mode")
    ollama_intent_model = LaunchConfiguration("ollama_intent_model")
    ollama_prompt_pack_path = LaunchConfiguration("ollama_prompt_pack_path")
    ollama_use_skill_catalog = LaunchConfiguration("ollama_use_skill_catalog")
    ollama_skill_catalog_packages = LaunchConfiguration("ollama_skill_catalog_packages")
    ollama_skill_catalog_max_entries = LaunchConfiguration(
        "ollama_skill_catalog_max_entries"
    )
    ollama_skill_catalog_max_chars = LaunchConfiguration("ollama_skill_catalog_max_chars")
    posture_skill_speed = LaunchConfiguration("posture_skill_speed")
    head_motion_skill_speed = LaunchConfiguration("head_motion_skill_speed")
    posture_skill_server_fallback_to_topic = LaunchConfiguration(
        "posture_skill_server_fallback_to_topic"
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
    bridge_input_speech_topic = LaunchConfiguration("bridge_input_speech_topic")
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

    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nao_chatbot"), "launch", "nao_chatbot_stack.launch.py"]
            )
        ),
        launch_arguments={
            "start_naoqi_driver": start_naoqi_driver,
            "nao_ip": nao_ip,
            "nao_port": nao_port,
            "network_interface": network_interface,
            "qi_listen_url": qi_listen_url,
            "start_rqt_chat": start_rqt_chat,
            "mission_mode": mission_mode,
            "ollama_enabled": ollama_enabled,
            "ollama_model": ollama_model,
            "ollama_intent_detection_mode": ollama_intent_detection_mode,
            "ollama_intent_model": ollama_intent_model,
            "ollama_prompt_pack_path": ollama_prompt_pack_path,
            "ollama_use_skill_catalog": ollama_use_skill_catalog,
            "ollama_skill_catalog_packages": ollama_skill_catalog_packages,
            "ollama_skill_catalog_max_entries": ollama_skill_catalog_max_entries,
            "ollama_skill_catalog_max_chars": ollama_skill_catalog_max_chars,
            "use_chat_skill": "true",
            "chat_skill_server_enabled": "true",
            "use_posture_skill": "true",
            "posture_skill_speed": posture_skill_speed,
            "use_head_motion_skill": "true",
            "head_motion_skill_speed": head_motion_skill_speed,
            "posture_skill_server_enabled": "true",
            "posture_skill_server_fallback_to_topic": posture_skill_server_fallback_to_topic,
            "head_motion_skill_server_enabled": "true",
            "use_say_skill": "true",
            "say_skill_server_enabled": "true",
            "say_skill_server_fallback_to_topic": "true",
            "posture_bridge_enabled": "true",
            "backend_execute_posture_after_response": "true",
            "backend_posture_from_response_enabled": "false",
            "asr_vosk_enabled": asr_vosk_enabled,
            "asr_microphone_topic": asr_microphone_topic,
            "asr_output_speech_topic": asr_output_speech_topic,
            "asr_speech_locale": asr_speech_locale,
            "asr_vosk_model_path": asr_vosk_model_path,
            "asr_sample_rate_hz": asr_sample_rate_hz,
            "asr_start_listening": asr_start_listening,
            "asr_publish_partials": asr_publish_partials,
            "asr_min_final_chars": asr_min_final_chars,
            "asr_min_final_words": asr_min_final_words,
            "asr_min_final_confidence": asr_min_final_confidence,
            "asr_ignore_single_token_fillers": asr_ignore_single_token_fillers,
            "asr_single_token_fillers_csv": asr_single_token_fillers_csv,
            "asr_debug_log_results": asr_debug_log_results,
            "asr_push_to_talk_enabled": asr_push_to_talk_enabled,
            "asr_push_to_talk_topic": asr_push_to_talk_topic,
            "asr_audio_capture_enabled": asr_audio_capture_enabled,
            "asr_audio_capture_source_type": asr_audio_capture_source_type,
            "asr_audio_capture_device": asr_audio_capture_device,
            "asr_audio_capture_sample_rate": asr_audio_capture_sample_rate,
            "asr_audio_capture_channels": asr_audio_capture_channels,
            "asr_audio_capture_sample_format": asr_audio_capture_sample_format,
            "asr_audio_capture_chunk_size": asr_audio_capture_chunk_size,
            "bridge_input_speech_topic": bridge_input_speech_topic,
            "dialogue_accept_incremental_speech": dialogue_accept_incremental_speech,
            "dialogue_ignore_user_speech_while_busy": dialogue_ignore_user_speech_while_busy,
            "dialogue_user_turn_holdoff_sec": dialogue_user_turn_holdoff_sec,
            "dialogue_user_turn_min_chars": dialogue_user_turn_min_chars,
            "dialogue_user_turn_min_words": dialogue_user_turn_min_words,
        }.items(),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            start_rqt_chat_arg,
            mission_mode_arg,
            ollama_enabled_arg,
            ollama_model_arg,
            ollama_intent_detection_mode_arg,
            ollama_intent_model_arg,
            ollama_prompt_pack_path_arg,
            ollama_use_skill_catalog_arg,
            ollama_skill_catalog_packages_arg,
            ollama_skill_catalog_max_entries_arg,
            ollama_skill_catalog_max_chars_arg,
            posture_skill_speed_arg,
            head_motion_skill_speed_arg,
            posture_skill_server_fallback_to_topic_arg,
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
            bridge_input_speech_topic_arg,
            dialogue_accept_incremental_speech_arg,
            dialogue_ignore_user_speech_while_busy_arg,
            dialogue_user_turn_holdoff_sec_arg,
            dialogue_user_turn_min_chars_arg,
            dialogue_user_turn_min_words_arg,
            stack_launch,
        ]
    )
