from nao_chatbot.stack_launch import generate_profile_launch_description


_PLANNER_LOCAL_PROFILE_DEFAULTS = {
    "start_chatbot_llm": "false",
    "start_dialogue_manager": "false",
    "start_knowledge_core": "false",
    "start_nao_orchestrator": "true",
    "start_nao_say_skill": "false",
    "start_nao_replay_motion": "false",
    "start_nao_look_at": "false",
    "start_interaction_sim": "false",
    "start_interaction_sim_perception": "false",
    "start_interaction_sim_tools": "false",
    "start_object_detection": "false",
    "start_scene_grounding": "false",
    "start_planner_llm": "true",
    "start_rqt_console": "false",
    "start_robot_speech_debug": "false",
    "planner_llm_provider": "ollama",
    "planner_llm_model": "gpt-oss:120b-cloud",
    "planner_llm_base_url": "http://127.0.0.1:11434",
}


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=_PLANNER_LOCAL_PROFILE_DEFAULTS,
        include_asr=False,
    )
