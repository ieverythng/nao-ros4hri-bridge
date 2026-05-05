from nao_chatbot.stack_launch import generate_profile_launch_description


_ROBOT_ASR_PROFILE_DEFAULTS = {
    "nao_ip": "172.26.112.62",
    "ollama_model": "gemma4:31b-cloud",
    "planner_llm_model": "gemma4:31b-cloud",
    "chatbot_server_url": "http://127.0.0.1:11434/api/chat",
    "planner_llm_base_url": "http://127.0.0.1:11435",
    "start_demo_log_window": "true",
    "start_naoqi_driver": "false",
    "start_nao_robot": "true",
    "start_planner_llm": "true",
    "chatbot_planner_mode_enabled": "true",
    "enable_orchestrator_planner_gate": "true",
    "chatbot_planner_request_topic": "/nao_orchestrator/planner_request",
    "start_nao_robot_hri_visualization": "true",
    "start_rviz": "true",
    "hri_visualization_image_topic": "/camera/front/image_raw",
    "object_detection_input_image_topic": "/camera/front/image_raw",
    "start_interaction_sim": "false",
    "start_interaction_sim_perception": "false",
    "start_interaction_sim_tools": "true",
    "start_interaction_sim_ui": "false",
    "start_rqt_console": "false",
    "chatbot_think": "false",
    "planner_llm_think": "false",
    "chatbot_preflight_required": "true",
    "chatbot_preflight_keepalive_interval_sec": "180.0",
    "planner_llm_preflight_required": "true",
    "chatbot_request_timeout_sec": "60.0",
    "chatbot_first_request_timeout_sec": "75.0",
    "chatbot_preflight_timeout_sec": "60.0",
    "chatbot_preflight_attempts": "3",
    "chatbot_preflight_realistic_enabled": "true",
    "planner_llm_timeout_sec": "60.0",
    "planner_llm_preflight_timeout_sec": "60.0",
    "planner_llm_preflight_attempts": "3",
    "planner_llm_preflight_realistic_enabled": "true",
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_ROBOT_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
