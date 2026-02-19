import re


def detect_intent(text: str) -> str:
    lowered = text.lower()
    if _contains_any_phrase(
        lowered,
        (
            "stand up",
            "get up",
            "please stand",
            "can you stand",
            "stand",
        ),
    ):
        return "posture_stand"
    if _contains_any_phrase(
        lowered,
        (
            "sit down",
            "take a seat",
            "please sit",
            "can you sit",
            "sit",
        ),
    ):
        return "posture_sit"
    if _contains_any_phrase(
        lowered,
        (
            "kneel down",
            "kneel",
            "crouch",
            "on your knees",
            "seiza",
        ),
    ):
        return "posture_kneel"
    if _contains_any_phrase(lowered, ("hello", "hi", "hey")):
        return "greet"
    if _contains_any_phrase(lowered, ("how are you",)):
        return "wellbeing"
    if _contains_any_phrase(lowered, ("your name", "who are you")):
        return "identity"
    if _contains_any_phrase(lowered, ("help",)):
        return "help"
    return "fallback"


def build_rule_response(intent: str) -> str:
    if intent == "posture_stand":
        return "Sure. I am switching to a standing posture."
    if intent == "posture_sit":
        return "Sure. I am switching to a sitting posture."
    if intent == "posture_kneel":
        return "Sure. I am switching to a kneeling posture."
    if intent == "greet":
        return "Hello! Nice to meet you."
    if intent == "wellbeing":
        return "I am doing well. Thank you for asking."
    if intent == "identity":
        return "I am your Nao mission controller."
    if intent == "help":
        return (
            "You can greet me, ask my name, ask how I am, or ask for posture "
            "changes like stand, kneel, or sit."
        )
    return "I heard you. We are testing the chat to speech pipeline."


def posture_command_for_intent(intent: str) -> str:
    posture_map = {
        "posture_stand": "stand",
        "posture_sit": "sit",
        "posture_kneel": "kneel",
    }
    return posture_map.get(intent, "")


def _contains_any_phrase(text: str, phrases: tuple[str, ...]) -> bool:
    for phrase in phrases:
        words = [re.escape(part) for part in phrase.split()]
        pattern = r"\b" + r"\s+".join(words) + r"\b"
        if re.search(pattern, text):
            return True
    return False
