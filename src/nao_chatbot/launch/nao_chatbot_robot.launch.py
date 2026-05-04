from nao_chatbot.stack_launch import generate_profile_launch_description


_ROBOT_PROFILE_DEFAULTS = {
    "nao_ip": "172.26.112.25",
    "ollama_model": "gemma4:31b-cloud",
    "planner_llm_model": "gemma4:31b-cloud",
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
    "posture_bridge_connect_on_startup": "true",
    "posture_bridge_disable_autonomous_life_on_connect": "false",
    "posture_bridge_wake_up_on_connect": "true",
    "head_motion_allow_open_loop_without_joint_state": "true",
    "head_motion_assume_success_on_convergence_timeout": "true",
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
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_ROBOT_PROFILE_DEFAULTS,
        include_asr=False,
    )
