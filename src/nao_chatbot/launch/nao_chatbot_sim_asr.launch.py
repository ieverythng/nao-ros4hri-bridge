from nao_chatbot.stack_launch import generate_profile_launch_description


_SIM_ASR_PROFILE_DEFAULTS = {
    "ollama_model": "gemma4:31b-cloud",
    "planner_llm_model": "gemma4:31b-cloud",
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
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_SIM_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
