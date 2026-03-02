import re


SUPPORTED_INTENTS = (
    "posture_stand",
    "posture_sit",
    "posture_kneel",
    "greet",
    "wellbeing",
    "identity",
    "help",
    "fallback",
)

_INTENT_ALIASES = {
    "__intent_greet__": "greet",
    "__intent_hello__": "greet",
    "__intent_identity__": "identity",
    "__intent_help__": "help",
    "__intent_wellbeing__": "wellbeing",
    "__intent_stand__": "posture_stand",
    "__intent_sit__": "posture_sit",
    "__intent_kneel__": "posture_kneel",
    "__intent_say__": "fallback",
    "__intent_start_activity__": "fallback",
}


def normalize_intent(
    intent: str,
    default: str = "fallback",
    hint_text: str = "",
) -> str:
    """Normalize incoming intent labels to one of SUPPORTED_INTENTS."""
    raw = str(intent).strip().lower()
    hints = str(hint_text).strip().lower()
    if not raw:
        if hints:
            hinted = detect_intent(hints)
            if hinted != "fallback":
                return hinted
        return default
    if raw in SUPPORTED_INTENTS:
        return raw
    if raw in _INTENT_ALIASES:
        alias = _INTENT_ALIASES[raw]
        if alias != "fallback":
            return alias

    search_space = f"{raw} {hints}".strip()
    if "stand" in search_space or search_space.endswith("_up"):
        return "posture_stand"
    if "sit" in search_space or "seat" in search_space:
        return "posture_sit"
    if (
        "kneel" in search_space
        or "crouch" in search_space
        or "seiza" in search_space
    ):
        return "posture_kneel"
    if "greet" in search_space or "hello" in search_space:
        return "greet"
    if (
        "who_are_you" in search_space
        or "name" in search_space
        or "identity" in search_space
    ):
        return "identity"
    if "wellbeing" in search_space or "how_are_you" in search_space:
        return "wellbeing"
    if "help" in search_space:
        return "help"
    return default


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
