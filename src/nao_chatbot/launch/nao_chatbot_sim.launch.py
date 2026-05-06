from nao_chatbot.stack_launch import generate_profile_launch_description


_SIM_PROFILE_DEFAULTS = {
    "ollama_model": "gemma4:31b-cloud",
    "planner_llm_model": "gemma4:31b-cloud",
    "chatbot_server_url": "http://127.0.0.1:11434/api/chat",
    "planner_llm_base_url": "http://127.0.0.1:11435",
    "start_managed_ollama": "false",
    "start_demo_log_window": "true",
    "start_naoqi_driver": "false",
    "start_nao_robot": "false",
    "start_nao_robot_hri_visualization": "false",
    "start_rviz": "false",
    "hri_visualization_image_topic": "/camera/image_raw",
    "object_detection_input_image_topic": "/camera/image_raw",
    "start_interaction_sim": "true",
    "start_interaction_sim_perception": "false",
    "start_interaction_sim_tools": "true",
    "start_interaction_sim_expressive_face": "false",
    "start_interaction_sim_ui": "false",
    "interaction_sim_hri_log_profile": "quiet",
    "object_detection_threshold": "0.40",
    "scene_grounding_knowledge_lifespan_sec": "3.0",
    "scene_grounding_knowledge_refresh_interval_sec": "0.75",
    "scene_grounding_fallback_match_distance_px": "40.0",
    "scene_grounding_fallback_match_max_age_sec": "1.2",
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
        profile_defaults=_SIM_PROFILE_DEFAULTS,
        include_asr=False,
    )
