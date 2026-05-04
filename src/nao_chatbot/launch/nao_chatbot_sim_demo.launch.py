from nao_chatbot.stack_launch import generate_profile_launch_description


_SIM_DEMO_PROFILE_DEFAULTS = {
    "ollama_model": "gemma4:31b-cloud",
    "planner_llm_model": "gemma4:31b-cloud",
    "nao_ip": "172.26.112.25",
    "network_interface": "wlp1s0",
    "start_naoqi_driver": "true",
    "start_object_detection": "true",
    "start_scene_grounding": "true",
    "object_detection_backend": "emorobcare_cv",
    "start_planner_llm": "true",
    "chatbot_planner_mode_enabled": "true",
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
    "interaction_sim_hri_log_profile": "quiet",
    "object_detection_threshold": "0.40",
    "scene_grounding_knowledge_lifespan_sec": "3.0",
    "scene_grounding_knowledge_refresh_interval_sec": "0.75",
    "scene_grounding_fallback_match_distance_px": "40.0",
    "scene_grounding_fallback_match_max_age_sec": "1.2",
    "start_rqt_console": "true",
    "head_motion_allow_open_loop_without_joint_state": "true",
    "head_motion_assume_success_on_convergence_timeout": "true",
    "chatbot_think": "false",
    "planner_llm_think": "false",
    "chatbot_preflight_required": "true",
    "chatbot_preflight_keepalive_interval_sec": "180.0",
    "planner_llm_preflight_required": "true",
    "enable_demo_scan_skill": "true",
    "demo_scan_result_mode": "success",
    "demo_scan_summary": "I looked around and can report a simple demo scene summary.",
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_SIM_DEMO_PROFILE_DEFAULTS,
        include_asr=False,
    )
