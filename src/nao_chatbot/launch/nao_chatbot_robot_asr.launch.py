from nao_chatbot.stack_launch import generate_profile_launch_description


_ROBOT_ASR_PROFILE_DEFAULTS = {
    "nao_ip": "127.0.0.1",
    "ollama_model": "gpt-oss:120b-cloud",
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
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_ROBOT_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
