#!/usr/bin/env python3
"""Intent routing helpers for `nao_orchestrator`."""

from __future__ import annotations

import json

from hri_actions_msgs.msg import Intent


def parse_intent_data(raw_data: str) -> dict:
    if not raw_data:
        return {}
    try:
        parsed = json.loads(raw_data)
    except json.JSONDecodeError:
        return {"raw": raw_data}
    return parsed if isinstance(parsed, dict) else {"raw": raw_data}


def normalize_legacy_intent(intent_name: str) -> tuple[str, dict]:
    clean = str(intent_name).strip().lower()
    mapping = {
        "greet": (Intent.GREET, {}),
        "__intent_greet__": (Intent.GREET, {}),
        "identity": (Intent.SAY, {"object": "I am your Nao orchestrator."}),
        "wellbeing": (Intent.SAY, {"object": "I am doing well. Thank you for asking."}),
        "help": (
            Intent.SAY,
            {
                "object": "You can greet me, ask me to stand, sit, kneel, or move my head.",
            },
        ),
        "posture_stand": (Intent.PERFORM_MOTION, {"object": "stand"}),
        "posture_sit": (Intent.PERFORM_MOTION, {"object": "sit"}),
        "posture_kneel": (Intent.PERFORM_MOTION, {"object": "kneel"}),
        "head_center": (Intent.PERFORM_MOTION, {"object": "head_center"}),
        "head_look_left": (Intent.PERFORM_MOTION, {"object": "head_look_left"}),
        "head_look_right": (Intent.PERFORM_MOTION, {"object": "head_look_right"}),
        "head_look_up": (Intent.PERFORM_MOTION, {"object": "head_look_up"}),
        "head_look_down": (Intent.PERFORM_MOTION, {"object": "head_look_down"}),
    }
    return mapping.get(clean, (Intent.RAW_USER_INPUT, {"input": clean}))


def classify_motion_target(intent_name: str, data: dict) -> tuple[str, dict]:
    if intent_name == Intent.PERFORM_MOTION:
        obj = str(data.get("object", "")).strip().lower()
    else:
        obj = str(intent_name).strip().lower()

    replay_motion_map = {
        "stand": "stand",
        "standinit": "standinit",
        "sit": "sit",
        "kneel": "kneel",
        "crouch": "crouch",
    }
    head_motion_map = {
        "head_center": {"yaw": 0.0, "pitch": 0.0, "relative": False},
        "head_look_left": {"yaw": 0.45, "pitch": 0.0, "relative": False},
        "head_look_right": {"yaw": -0.45, "pitch": 0.0, "relative": False},
        "head_look_up": {"yaw": 0.0, "pitch": -0.20, "relative": False},
        "head_look_down": {"yaw": 0.0, "pitch": 0.20, "relative": False},
    }

    if obj in replay_motion_map:
        return ("replay_motion", {"motion_name": replay_motion_map[obj]})
    if obj in head_motion_map:
        return ("head_motion", head_motion_map[obj])
    return ("unsupported", {"object": obj})
