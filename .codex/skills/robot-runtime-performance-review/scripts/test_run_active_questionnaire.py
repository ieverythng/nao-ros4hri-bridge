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
