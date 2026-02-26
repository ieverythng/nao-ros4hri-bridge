from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="false",
        description="Whether to launch naoqi_driver together with chatbot stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="10.10.200.149",
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
    laptop_asr_enabled_arg = DeclareLaunchArgument(
        "laptop_asr_enabled",
        default_value="false",
        description="Deprecated alias for asr_vosk_enabled.",
    )
    asr_output_speech_topic_arg = DeclareLaunchArgument(
        "asr_output_speech_topic",
        default_value="/humans/voices/anonymous_speaker/speech",
        description="LiveSpeech topic where local ASR publishes transcripts.",
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
    asr_block_duration_ms_arg = DeclareLaunchArgument(
        "asr_block_duration_ms",
        default_value="250",
        description="Audio block duration in ms used by local ASR.",
    )
    asr_device_index_arg = DeclareLaunchArgument(
        "asr_device_index",
        default_value="-1",
        description="Microphone device index; -1 uses default.",
    )
    asr_min_chars_arg = DeclareLaunchArgument(
        "asr_min_chars",
        default_value="2",
        description="Minimum transcript length before publishing.",
    )
    asr_dedupe_window_sec_arg = DeclareLaunchArgument(
        "asr_dedupe_window_sec",
        default_value="0.8",
        description="Ignore duplicate ASR outputs within this window.",
    )
    asr_publish_partial_arg = DeclareLaunchArgument(
        "asr_publish_partial",
        default_value="false",
        description="Publish partial hypotheses from Vosk.",
    )
    asr_allow_sample_rate_fallback_arg = DeclareLaunchArgument(
        "asr_allow_sample_rate_fallback",
        default_value="true",
        description="Try device-default/common rates if requested rate is unsupported.",
    )
    asr_min_words_arg = DeclareLaunchArgument(
        "asr_min_words",
        default_value="1",
        description="Minimum number of words required before publishing transcript.",
    )
    asr_ignore_single_token_fillers_arg = DeclareLaunchArgument(
        "asr_ignore_single_token_fillers",
        default_value="true",
        description='Drop one-word fillers like "um"/"huh".',
    )
    asr_suppress_during_robot_speech_arg = DeclareLaunchArgument(
        "asr_suppress_during_robot_speech",
        default_value="true",
        description="Mute ASR while robot speech is active to avoid overlap/echo.",
    )
    asr_robot_speech_topic_arg = DeclareLaunchArgument(
        "asr_robot_speech_topic",
        default_value="/speech",
        description="Topic used to detect robot speech and apply ASR mute window.",
    )
    asr_speech_guard_sec_per_word_arg = DeclareLaunchArgument(
        "asr_speech_guard_sec_per_word",
        default_value="0.33",
        description="Estimated robot speech seconds per word used by ASR mute guard.",
    )
    asr_speech_guard_min_sec_arg = DeclareLaunchArgument(
        "asr_speech_guard_min_sec",
        default_value="1.0",
        description="Minimum ASR mute window in seconds after robot speech starts.",
    )
    asr_speech_guard_extra_sec_arg = DeclareLaunchArgument(
        "asr_speech_guard_extra_sec",
        default_value="0.5",
        description="Extra mute seconds added after estimated robot speech duration.",
    )
    asr_status_warn_period_sec_arg = DeclareLaunchArgument(
        "asr_status_warn_period_sec",
        default_value="2.0",
        description="Minimum interval between repeated ASR stream warnings.",
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
    chat_skill_fallback_to_backend_topic_arg = DeclareLaunchArgument(
        "chat_skill_fallback_to_backend_topic",
        default_value="true",
        description="Fallback to backend topic when chat skill is unavailable.",
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
    legacy_backend_node_enabled_arg = DeclareLaunchArgument(
        "legacy_backend_node_enabled",
        default_value="false",
        description="Launch legacy backend topic responder (ollama_responder).",
    )

    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Topic used for legacy posture bridge commands.",
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

    posture_bridge_enabled_arg = DeclareLaunchArgument(
        "posture_bridge_enabled",
        default_value="true",
        description="Launch legacy posture bridge node.",
    )
    posture_bridge_speed_arg = DeclareLaunchArgument(
        "posture_bridge_speed",
        default_value="0.8",
        description="Default posture speed for legacy posture bridge.",
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
        description="Enable Ollama requests in chat/legacy backend servers.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="llama3.2:1b",
        description="Ollama model name.",
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
    laptop_asr_enabled = LaunchConfiguration("laptop_asr_enabled")
    asr_output_speech_topic = LaunchConfiguration("asr_output_speech_topic")
    asr_vosk_model_path = LaunchConfiguration("asr_vosk_model_path")
    asr_sample_rate_hz = LaunchConfiguration("asr_sample_rate_hz")
    asr_block_duration_ms = LaunchConfiguration("asr_block_duration_ms")
    asr_device_index = LaunchConfiguration("asr_device_index")
    asr_min_chars = LaunchConfiguration("asr_min_chars")
    asr_dedupe_window_sec = LaunchConfiguration("asr_dedupe_window_sec")
    asr_publish_partial = LaunchConfiguration("asr_publish_partial")
    asr_allow_sample_rate_fallback = LaunchConfiguration("asr_allow_sample_rate_fallback")
    asr_min_words = LaunchConfiguration("asr_min_words")
    asr_ignore_single_token_fillers = LaunchConfiguration(
        "asr_ignore_single_token_fillers"
    )
    asr_suppress_during_robot_speech = LaunchConfiguration(
        "asr_suppress_during_robot_speech"
    )
    asr_robot_speech_topic = LaunchConfiguration("asr_robot_speech_topic")
    asr_speech_guard_sec_per_word = LaunchConfiguration(
        "asr_speech_guard_sec_per_word"
    )
    asr_speech_guard_min_sec = LaunchConfiguration("asr_speech_guard_min_sec")
    asr_speech_guard_extra_sec = LaunchConfiguration("asr_speech_guard_extra_sec")
    asr_status_warn_period_sec = LaunchConfiguration("asr_status_warn_period_sec")

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
    chat_skill_fallback_to_backend_topic = LaunchConfiguration(
        "chat_skill_fallback_to_backend_topic"
    )
    chat_history_max_entries = LaunchConfiguration("chat_history_max_entries")
    chat_skill_server_enabled = LaunchConfiguration("chat_skill_server_enabled")
    legacy_backend_node_enabled = LaunchConfiguration("legacy_backend_node_enabled")

    posture_command_topic = LaunchConfiguration("posture_command_topic")
    use_posture_skill = LaunchConfiguration("use_posture_skill")
    posture_skill_action = LaunchConfiguration("posture_skill_action")
    posture_skill_speed = LaunchConfiguration("posture_skill_speed")
    posture_skill_dispatch_wait_sec = LaunchConfiguration("posture_skill_dispatch_wait_sec")

    posture_skill_server_enabled = LaunchConfiguration("posture_skill_server_enabled")
    posture_skill_server_default_speed = LaunchConfiguration(
        "posture_skill_server_default_speed"
    )
    posture_skill_server_fallback_to_topic = LaunchConfiguration(
        "posture_skill_server_fallback_to_topic"
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
        package="nao_chatbot",
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
            }
        ],
    )

    asr_vosk_condition = IfCondition(
        PythonExpression(
            [
                "('",
                asr_vosk_enabled,
                "' == 'true') or ('",
                laptop_asr_enabled,
                "' == 'true')",
            ]
        )
    )
    asr_vosk = Node(
        package="nao_chatbot",
        executable="asr_vosk_node",
        name="asr_vosk",
        output="screen",
        condition=asr_vosk_condition,
        parameters=[
            {
                "enabled": True,
                "output_speech_topic": asr_output_speech_topic,
                "vosk_model_path": asr_vosk_model_path,
                "sample_rate_hz": asr_sample_rate_hz,
                "block_duration_ms": asr_block_duration_ms,
                "device_index": asr_device_index,
                "min_chars": asr_min_chars,
                "min_words": asr_min_words,
                "dedupe_window_sec": asr_dedupe_window_sec,
                "publish_partial": asr_publish_partial,
                "allow_sample_rate_fallback": asr_allow_sample_rate_fallback,
                "ignore_single_token_fillers": asr_ignore_single_token_fillers,
                "suppress_during_robot_speech": asr_suppress_during_robot_speech,
                "robot_speech_topic": asr_robot_speech_topic,
                "speech_guard_sec_per_word": asr_speech_guard_sec_per_word,
                "speech_guard_min_sec": asr_speech_guard_min_sec,
                "speech_guard_extra_sec": asr_speech_guard_extra_sec,
                "status_warn_period_sec": asr_status_warn_period_sec,
            }
        ],
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
                "chat_skill_fallback_to_backend_topic": chat_skill_fallback_to_backend_topic,
                "chat_history_max_entries": chat_history_max_entries,
                "posture_command_topic": posture_command_topic,
                "use_posture_skill": use_posture_skill,
                "posture_skill_action": posture_skill_action,
                "posture_skill_speed": posture_skill_speed,
                "posture_skill_dispatch_wait_sec": posture_skill_dispatch_wait_sec,
            }
        ],
    )

    chat_skill_server = Node(
        package="nao_chatbot",
        executable="chat_skill_server_node",
        name="chat_skill_server",
        output="screen",
        condition=IfCondition(chat_skill_server_enabled),
        parameters=[
            {
                "action_name": chat_skill_action,
                "enabled": ollama_enabled,
                "model": ollama_model,
            }
        ],
    )

    legacy_backend_condition = IfCondition(
        PythonExpression(
            [
                "('",
                legacy_backend_node_enabled,
                "' == 'true') or ('",
                use_chat_skill,
                "' == 'false')",
            ]
        )
    )
    ollama = Node(
        package="nao_chatbot",
        executable="ollama_responder_node",
        name="ollama_responder",
        output="screen",
        condition=legacy_backend_condition,
        parameters=[{"enabled": ollama_enabled, "model": ollama_model}],
    )

    posture_bridge = Node(
        package="nao_posture_bridge",
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
        package="nao_posture_bridge",
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

    say_skill_server = Node(
        package="nao_posture_bridge",
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
            laptop_asr_enabled_arg,
            asr_output_speech_topic_arg,
            asr_vosk_model_path_arg,
            asr_sample_rate_hz_arg,
            asr_block_duration_ms_arg,
            asr_device_index_arg,
            asr_min_chars_arg,
            asr_dedupe_window_sec_arg,
            asr_publish_partial_arg,
            asr_allow_sample_rate_fallback_arg,
            asr_min_words_arg,
            asr_ignore_single_token_fillers_arg,
            asr_suppress_during_robot_speech_arg,
            asr_robot_speech_topic_arg,
            asr_speech_guard_sec_per_word_arg,
            asr_speech_guard_min_sec_arg,
            asr_speech_guard_extra_sec_arg,
            asr_status_warn_period_sec_arg,
            mission_mode_arg,
            backend_fallback_to_rules_arg,
            backend_response_timeout_sec_arg,
            backend_execute_posture_after_response_arg,
            backend_posture_from_response_enabled_arg,
            use_chat_skill_arg,
            chat_skill_action_arg,
            chat_skill_dispatch_wait_sec_arg,
            chat_skill_fallback_to_backend_topic_arg,
            chat_history_max_entries_arg,
            chat_skill_server_enabled_arg,
            legacy_backend_node_enabled_arg,
            posture_command_topic_arg,
            use_posture_skill_arg,
            posture_skill_action_arg,
            posture_skill_speed_arg,
            posture_skill_dispatch_wait_sec_arg,
            posture_skill_server_enabled_arg,
            posture_skill_server_default_speed_arg,
            posture_skill_server_fallback_to_topic_arg,
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
            naoqi_driver,
            dialogue_manager,
            asr_vosk,
            posture_skill_server,
            say_skill_server,
            chat_skill_server,
            mission,
            ollama,
            posture_bridge,
            rqt_chat,
        ]
    )
