from nao_chatbot.stack_launch import generate_profile_launch_description


_SIM_ASR_PROFILE_DEFAULTS = {
    "ollama_model": "gpt-oss:120b-cloud",
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
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_SIM_ASR_PROFILE_DEFAULTS,
        include_asr=True,
    )
