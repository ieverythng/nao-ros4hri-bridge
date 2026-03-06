import re


SUPPORTED_INTENTS = (
    "posture_stand",
    "posture_sit",
    "posture_kneel",
    "head_center",
    "head_look_left",
    "head_look_right",
    "head_look_up",
    "head_look_down",
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
    "__intent_head_center__": "head_center",
    "__intent_look_left__": "head_look_left",
    "__intent_look_right__": "head_look_right",
    "__intent_look_up__": "head_look_up",
    "__intent_look_down__": "head_look_down",
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
    search_space = search_space.replace("_", " ").replace("-", " ")
    if "stand" in search_space or search_space.endswith(" up"):
        return "posture_stand"
    if "sit" in search_space or "seat" in search_space:
        return "posture_sit"
    if (
        "kneel" in search_space
        or "crouch" in search_space
        or "seiza" in search_space
    ):
        return "posture_kneel"
    if "look left" in search_space or "turn left" in search_space:
        return "head_look_left"
    if "look right" in search_space or "turn right" in search_space:
        return "head_look_right"
    if "look up" in search_space:
        return "head_look_up"
    if "look down" in search_space:
        return "head_look_down"
    if "center head" in search_space or "look center" in search_space:
        return "head_center"
    if "greet" in search_space or "hello" in search_space:
        return "greet"
    if (
        "who_are_you" in search_space
        or "who are you" in search_space
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
            "look left",
            "turn your head left",
            "head left",
        ),
    ):
        return "head_look_left"
    if _contains_any_phrase(
        lowered,
        (
            "look right",
            "turn your head right",
            "head right",
        ),
    ):
        return "head_look_right"
    if _contains_any_phrase(
        lowered,
        (
            "look up",
            "head up",
            "tilt your head up",
        ),
    ):
        return "head_look_up"
    if _contains_any_phrase(
        lowered,
        (
            "look down",
            "head down",
            "tilt your head down",
        ),
    ):
        return "head_look_down"
    if _contains_any_phrase(
        lowered,
        (
            "look straight",
            "look center",
            "center your head",
            "head center",
            "head straight",
            "face forward",
        ),
    ):
        return "head_center"
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
    if intent == "head_look_left":
        return "Sure. I am turning my head to the left."
    if intent == "head_look_right":
        return "Sure. I am turning my head to the right."
    if intent == "head_look_up":
        return "Sure. I am tilting my head up."
    if intent == "head_look_down":
        return "Sure. I am tilting my head down."
    if intent == "head_center":
        return "Sure. I am centering my head."
    if intent == "greet":
        return "Hello! Nice to meet you."
    if intent == "wellbeing":
        return "I am doing well. Thank you for asking."
    if intent == "identity":
        return "I am your Nao mission controller."
    if intent == "help":
        return (
            "You can greet me, ask my name, ask how I am, or ask for posture "
            "changes like stand, kneel, or sit, and head movements like look "
            "left, right, up, down, or center."
        )
    return "I heard you. We are testing the chat to speech pipeline."


def posture_command_for_intent(intent: str) -> str:
    posture_map = {
        "posture_stand": "stand",
        "posture_sit": "sit",
        "posture_kneel": "kneel",
    }
    return posture_map.get(intent, "")


def head_motion_goal_for_intent(intent: str) -> dict:
    """Map intent to canned head-motion goal parameters."""
    goal_map = {
        "head_center": {"yaw": 0.0, "pitch": 0.0, "relative": False},
        "head_look_left": {"yaw": 0.45, "pitch": 0.0, "relative": False},
        "head_look_right": {"yaw": -0.45, "pitch": 0.0, "relative": False},
        "head_look_up": {"yaw": 0.0, "pitch": -0.20, "relative": False},
        "head_look_down": {"yaw": 0.0, "pitch": 0.20, "relative": False},
    }
    return dict(goal_map.get(intent, {}))


def _contains_any_phrase(text: str, phrases: tuple[str, ...]) -> bool:
    for phrase in phrases:
        words = [re.escape(part) for part in phrase.split()]
        pattern = r"\b" + r"\s+".join(words) + r"\b"
        if re.search(pattern, text):
            return True
    return False
