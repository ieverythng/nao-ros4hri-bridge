from nao_chatbot.stack_launch import generate_profile_launch_description


_SIM_ASR_PROFILE_DEFAULTS = {
    "chatbot_model": "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ",
    "planner_llm_provider": "openai_compatible",
    "planner_llm_model": "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ",
    "chatbot_server_url": "http://10.7.138.215:8004/v1/chat/completions",
    "planner_llm_base_url": "http://10.7.138.215:8004",
    "start_naoqi_driver": "false",
    "start_nao_robot": "false",
    "start_nao_robot_hri_visualization": "false",
    "start_rviz": "false",
    "hri_visualization_image_topic": "/camera/image_raw",
    "object_detection_input_image_topic": "/camera/image_raw",
    "start_interaction_sim": "true",
    "start_interaction_sim_perception": "true",
    "start_interaction_sim_tools": "true",
    "start_interaction_sim_expressive_face": "false",
    "start_interaction_sim_ui": "false",
    "start_rqt_console": "true",
    "enable_orchestrator_planner_gate": "true",
    "chatbot_planner_request_topic": "/nao_orchestrator/planner_request",
    "head_motion_allow_open_loop_without_joint_state": "true",
    "head_motion_assume_success_on_convergence_timeout": "true",
    "chatbot_think": "false",
    "planner_llm_think": "false",
    "chatbot_preflight_required": "true",
    "chatbot_preflight_keepalive_interval_sec": "180.0",
    "planner_llm_preflight_required": "true",
    "chat_input_tracked_topic": "/humans/voices/tracked",
    "chat_input_speech_topic": "/humans/voices/anonymous_speaker/speech",
    "chat_input_is_speaking_topic": "/humans/voices/anonymous_speaker/is_speaking",
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_SIM_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
