from nao_chatbot.stack_launch import demo_profile_defaults
from nao_chatbot.stack_launch import generate_profile_launch_description


def generate_launch_description():
    return generate_profile_launch_description(
        profile_defaults=demo_profile_defaults(),
    )
