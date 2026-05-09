from nao_chatbot.stack_launch import generate_profile_launch_description
from nao_chatbot.stack_launch import sim_profile_defaults


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=sim_profile_defaults(),
    )
