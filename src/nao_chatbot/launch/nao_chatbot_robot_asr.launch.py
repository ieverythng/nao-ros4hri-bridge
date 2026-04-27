from nao_chatbot.stack_launch import generate_profile_launch_description


_ROBOT_ASR_PROFILE_DEFAULTS = {
    "nao_ip": "172.26.112.62",
    "ollama_model": "qwen3.5:397b-cloud",
    "start_naoqi_driver": "false",
    "start_nao_robot": "true",
    "start_planner_llm": "true",
    "chatbot_planner_mode_enabled": "true",
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
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_ROBOT_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
