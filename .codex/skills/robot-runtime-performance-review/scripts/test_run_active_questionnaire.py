import importlib.util
import sys
from pathlib import Path


def _load_questionnaire_module():
    path = Path(__file__).with_name("run_active_questionnaire.py")
    spec = importlib.util.spec_from_file_location("run_active_questionnaire", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_kb_query_rows_from_ros2_service_response():
    module = _load_questionnaire_module()
    output = (
        "response:\n"
        "kb_msgs.srv.Query_Response(success=True, error_msg='', "
        "json='[{\"subject\":\"codex_missing_cup\"}]')\n"
    )

    assert module.parse_kb_query_rows(output) == [{"subject": "codex_missing_cup"}]


def test_parse_kb_query_rows_handles_empty_result():
    module = _load_questionnaire_module()
    output = (
        "response:\n"
        "kb_msgs.srv.Query_Response(success=True, error_msg='', json='[]')\n"
    )

    assert module.parse_kb_query_rows(output) == []


def test_absence_guard_marks_stale_rows_as_contaminated(monkeypatch):
    module = _load_questionnaire_module()

    def fake_query(_container, *, patterns, query_vars, timeout_sec):
        return {
            "raw_output": "fake",
            "rows": [{"subject": "codex_lab_blake"}],
        }

    monkeypatch.setattr(module, "query_kb_rows", fake_query)
    result = module.run_absence_guards(
        "nao_ros2",
        (
            module.KbAbsenceGuard(
                "blake_named_person_absent",
                ("?subject dbp:name BLAKE",),
                ("?subject",),
            ),
        ),
    )

    assert result["contaminated"] is True
    assert result["guard_results"][0]["clean"] is False
    assert result["guard_results"][0]["row_count"] == 1


def test_absence_guard_allows_clean_fixture(monkeypatch):
    module = _load_questionnaire_module()

    def fake_query(_container, *, patterns, query_vars, timeout_sec):
        return {"raw_output": "fake", "rows": []}

    monkeypatch.setattr(module, "query_kb_rows", fake_query)
    result = module.run_absence_guards(
        "nao_ros2",
        (
            module.KbAbsenceGuard(
                "missing_cup_subject_absent",
                ("codex_missing_cup ?predicate ?object",),
            ),
        ),
    )

    assert result["contaminated"] is False
    assert result["guard_results"][0]["clean"] is True


def test_questionnaire_metadata_records_grounded_context_digest(monkeypatch):
    module = _load_questionnaire_module()

    def fake_get_ros_param(_container, _node_name, param_name):
        values = {
            "turn_pipeline_mode": "response_first",
            "grounded_context_digest_enabled": "Boolean value is: False",
        }
        return values[param_name]

    monkeypatch.setattr(module, "get_ros_param", fake_get_ros_param)

    metadata = module.collect_questionnaire_metadata(
        "nao_ros2",
        expected_turn_pipeline_mode="response_first",
    )

    assert metadata["chatbot_turn_pipeline_mode"] == "response_first"
    assert metadata["grounded_context_digest_enabled"] == "Boolean value is: False"
    assert metadata["turn_pipeline_mode_matches_expected"] is True


def test_voice_speech_topic_uses_raw_per_voice_hri_namespace():
    module = _load_questionnaire_module()

    assert (
        module._voice_speech_topic("anonymous_speaker")
        == "/humans/voices/anonymous_speaker/speech"
    )
    assert (
        module._voice_speech_topic("fake_deep_lab_sections_1")
        == "/humans/voices/fake_deep_lab_sections_1/speech"
    )


def test_complete_context_execution_fails_on_clarification():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "grouped_delivery",
        "fake_deep",
        "Bring every object from the work table to ALEX.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    )
    observations = {
        "turn_injected": True,
        "planner_request_observed": False,
        "execution_feedback_observed": False,
        "terminal_observed": True,
        "speech_observed": True,
        "clarification_observed": True,
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(case, observations=observations, stale_world_guard=None)

    assert result["status"] == "fail"
    assert "clarification" in " ".join(result["reasons"])


def test_missing_recipient_passes_only_with_clarification_without_execution():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "missing_recipient",
        "fake_deep",
        "Bring every object to BLAKE.",
        expected_outcome="clarification_expected",
    )
    observations = {
        "turn_injected": True,
        "planner_request_observed": False,
        "execution_feedback_observed": False,
        "terminal_observed": True,
        "speech_observed": True,
        "clarification_observed": True,
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(case, observations=observations, stale_world_guard=None)

    assert result["status"] == "pass"


def test_missing_person_wording_counts_as_clarification():
    module = _load_questionnaire_module()

    assert module.clarification_observed(
        "I cannot confirm that person in the current grounded context. "
        "Which person should I use for the task?"
    )


def test_fallback_markers_track_report_and_kb_failures():
    module = _load_questionnaire_module()

    markers = module.fallback_markers(
        "execution report chatbot returned error; KnowledgeCore query timeout"
    )

    assert markers["report_result_fallback"] == 1
    assert markers["kb_service_timeout"] == 1
    assert markers["total"] >= 2


def test_fake_deep_iiia_floor_has_post_effect_location_followup_without_refixture():
    module = _load_questionnaire_module()
    cases = {case.name: case for case in module.FAKE_DEEP_CASES}

    delivery = cases["fake_deep_iiia_kitchen_delivery"]
    followup = cases["fake_deep_iiia_floor_location_followup"]

    assert followup.category == "fake_deep_post_effect_query"
    assert followup.conversation_group == delivery.conversation_group
    assert followup.environment_ids == ()
