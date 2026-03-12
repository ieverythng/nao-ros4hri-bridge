from hri_actions_msgs.msg import Intent

from nao_orchestrator.intent_rules import classify_motion_target, normalize_legacy_intent


def test_normalize_legacy_intent_maps_posture_to_perform_motion():
    intent_name, data = normalize_legacy_intent("posture_stand")
    assert intent_name == Intent.PERFORM_MOTION
    assert data["object"] == "stand"


def test_classify_motion_target_maps_head_motion():
    route, payload = classify_motion_target(Intent.PERFORM_MOTION, {"object": "head_look_left"})
    assert route == "head_motion"
    assert payload["yaw"] == 0.45
