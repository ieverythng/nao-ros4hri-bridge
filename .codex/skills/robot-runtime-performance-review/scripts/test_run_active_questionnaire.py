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


def test_fixture_type_readiness_requires_declared_rdf_types():
    module = _load_questionnaire_module()
    injection = module.KbInjection(
        object_id="lab",
        statements=(
            "codex_phone rdf:type CellularTelephone",
            "codex_phone dbp:name VEGA",
            "myself sees codex_phone",
        ),
        query_patterns=("codex_phone ?predicate ?object",),
        query_vars=("?predicate", "?object"),
    )

    assert module._fixture_type_bindings(injection) == {
        ("codex_phone", "CellularTelephone")
    }
    assert module._present_fixture_type_bindings(
        [{"entity": "codex_phone", "predicate": "rdf:type", "object": "owl:Thing"}]
    ) == {("codex_phone", "owl:Thing")}
    assert not module._fixture_type_bindings(injection).issubset(
        module._present_fixture_type_bindings(
            [{"entity": "codex_phone", "predicate": "rdf:type", "object": "owl:Thing"}]
        )
    )


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


def test_voice_speech_topic_uses_integrated_remap_for_shared_speaker():
    module = _load_questionnaire_module()

    assert (
        module._voice_speech_topic("anonymous_speaker")
        == "/nao_chatbot/humans/voices/anonymous_speaker/speech"
    )
    assert (
        module._voice_speech_topic("fake_deep_lab_sections_1")
        == "/humans/voices/fake_deep_lab_sections_1/speech"
    )


def test_publish_voice_turn_keeps_tracked_voice_alive_for_group_continuity(monkeypatch):
    module = _load_questionnaire_module()
    commands = []

    def fake_run(command, **_kwargs):
        commands.append(command)
        return "published"

    monkeypatch.setattr(module, "run", fake_run)

    assert module.publish_voice_turn(
        "nao_ros2",
        "Bring those objects to the person.",
        voice_id="robust_group",
        mirror_rqt_display=False,
    ) == "published"

    script = commands[0][-1]
    assert "nao_questionnaire_tracked_robust_group.pid" in script
    assert "nohup ros2 topic pub -r 2" in script
    assert 'kill "$tracked_pub_pid"' not in script
    assert '[ "$prior_pid_file" = "/tmp/nao_questionnaire_tracked_robust_group.pid" ]' in script
    assert 'kill "$prior_pid"' in script


def test_cleanup_tracked_voice_publishers_removes_persistent_publishers(monkeypatch):
    module = _load_questionnaire_module()
    commands = []
    monkeypatch.setattr(
        module,
        "run",
        lambda command, **_kwargs: commands.append(command) or "cleaned",
    )

    result = module.cleanup_tracked_voice_publishers("nao_ros2")

    assert result == "cleaned"
    assert "nao_questionnaire_tracked_*.pid" in commands[0][-1]


def test_posture_ablation_covers_body_postures_without_head_motion():
    module = _load_questionnaire_module()

    names = {case.name for case in module.POSTURE_ABLATION_CASES}
    combined_text = " ".join(case.text.lower() for case in module.POSTURE_ABLATION_CASES)

    assert names == {
        "posture_stand_report",
        "posture_sit_report",
        "posture_kneel_report",
        "posture_sequence_final_state",
    }
    assert all(
        case.expected_outcome == "execute_no_clarification"
        for case in module.POSTURE_ABLATION_CASES
    )
    assert "head" not in combined_text


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


def test_execution_help_after_evidenced_failure_is_not_context_clarification():
    module = _load_questionnaire_module()

    assert not module.clarification_observed(
        'act=ask_for_help reason=I could not navigate to codex_lab_book. '
        'Could you help me?'
    )


def test_fallback_markers_track_report_and_kb_failures():
    module = _load_questionnaire_module()

    markers = module.fallback_markers(
        "execution report chatbot returned error; KnowledgeCore query timeout"
    )

    assert markers["report_result_fallback"] == 1
    assert markers["kb_service_timeout"] == 1
    assert markers["total"] >= 2


def test_recovery_requires_speech_after_terminal_event():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "blocked_delivery",
        "fake_deep",
        "Bring the cup.",
        expected_outcome="recover_or_truthful_failure",
    )
    observations = {
        "turn_injected": True,
        "terminal_observed": True,
        "speech_observed": True,
        "post_terminal_speech_observed": False,
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(case, observations=observations, stale_world_guard=None)

    assert result["status"] == "degraded"
    assert "after terminal" in " ".join(result["reasons"])


def test_failure_profile_is_not_scored_when_configured_failure_did_not_fire():
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
        "planner_request_observed": True,
        "execution_feedback_observed": True,
        "terminal_observed": True,
        "speech_observed": True,
        "post_terminal_speech_observed": True,
        "clarification_observed": False,
        "failure_observed": False,
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
        fake_policy_profile="fail_once_pick",
    )

    assert result["status"] == "not_scored"
    assert "not exercised" in " ".join(result["reasons"])


def test_recoverable_failure_case_requires_replan_evidence_when_declared():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "ordered_walk",
        "fake_deep",
        "Walk to every object on the table.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_replan=True,
    )
    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "planner_request_observed": True,
            "execution_feedback_observed": True,
            "terminal_observed": True,
            "speech_observed": True,
            "post_terminal_speech_observed": True,
            "failure_observed": True,
            "replan_observed": False,
            "clarification_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
        fake_policy_profile="fail_once_navigation",
    )

    assert result["status"] == "fail"
    assert "did not produce a replan" in " ".join(result["reasons"])


def test_phase_observations_expose_failure_and_replan_evidence():
    module = _load_questionnaire_module()

    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt="step_failed status=failed\nplanner mode=replan plan_version=2",
        topic_samples={},
    )

    assert observations["failure_observed"] is True
    assert observations["replan_observed"] is True


def test_case_evidence_excludes_unrelated_goal_lines():
    module = _load_questionnaire_module()
    logs = "\n".join(
        [
            "[100.0] source=voice_case goal_id=goal_current PLANNER_REQUEST",
            "[101.0] goal_id=goal_current event_type=plan_completed",
            "[102.0] goal_id=goal_old DEBUG_SPEECH_PUBLISHED",
        ]
    )

    correlated = module.correlate_case_evidence(logs, voice_id="voice_case")
    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt=logs,
        topic_samples={},
        voice_id="voice_case",
    )

    assert "goal_current" in correlated
    assert "goal_old" not in correlated
    assert observations["terminal_observed"] is True
    assert observations["speech_observed"] is False


def test_case_evidence_includes_speech_inside_goal_scoped_report_step():
    module = _load_questionnaire_module()
    logs = "\n".join(
        [
            "[100.0] source=voice_case goal_id=goal_current PLANNER_REQUEST",
            "[101.0] goal_id=goal_current event_type=step_started,step=report_result",
            '[101.2] [ROBOT OUTPUT] "I arrived at the cup."',
            "[101.3] goal_id=goal_current event_type=step_succeeded,step=report_result",
            '[102.0] [ROBOT OUTPUT] "Unrelated speech."',
        ]
    )

    correlated = module.correlate_case_evidence(logs, voice_id="voice_case")

    assert "I arrived at the cup" in correlated
    assert "Unrelated speech" not in correlated


def test_dialogue_only_case_rejects_planner_handoff():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "non_action",
        "route_safety",
        "Remember ALEX, but do not act yet.",
        expected_outcome="dialogue_only",
    )

    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "route_observed": True,
            "speech_observed": True,
            "planner_request_observed": True,
            "execution_feedback_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert "leaked" in " ".join(result["reasons"])


def test_complete_grouped_case_requires_target_selection_evidence():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "grouped",
        "fake_deep",
        "Bring every object from the table to ALEX.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
    )

    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "planner_request_observed": True,
            "execution_feedback_observed": True,
            "terminal_observed": True,
            "speech_observed": True,
            "clarification_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert "target_selection" in " ".join(result["reasons"])


def test_complete_context_help_request_is_a_clarification_failure():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "grouped_delivery",
        "fake_deep",
        "Bring every object from the work table to ALEX.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    )
    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "planner_request_observed": True,
            "execution_feedback_observed": True,
            "terminal_observed": True,
            "speech_observed": True,
            "clarification_observed": module.clarification_observed(
                "act=ask_for_help I need help identifying the person named ALEX"
            ),
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert result["reasons"] == [
        "asked for clarification despite complete fixture context"
    ]


def test_fake_deep_iiia_floor_has_post_effect_location_followup_without_refixture():
    module = _load_questionnaire_module()
    cases = {case.name: case for case in module.FAKE_DEEP_CASES}

    delivery = cases["fake_deep_iiia_kitchen_delivery"]
    followup = cases["fake_deep_iiia_floor_location_followup"]

    assert followup.category == "fake_deep_post_effect_query"
    assert followup.conversation_group == delivery.conversation_group
    assert followup.environment_ids == ()


def test_synthetic_operator_delivery_requires_grounded_recipient_clarification():
    module = _load_questionnaire_module()
    cases = {case.name: case for case in module.MAIN_QUESTIONNAIRE_CASES}

    case = cases["maximal_kitchen_cup_to_operator"]

    assert case.expected_outcome == "clarification_expected"
    assert case.all_required_context is False


def test_phase_observations_extract_complete_target_selection():
    module = _load_questionnaire_module()
    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt=(
            "planner request target_selection={'selection_kind': 'explicit_members', "
            "'operation': 'visit', 'member_ids': ['cup_1', 'book_1'], "
            "'recipient_id': '', 'ordering': 'sequential', "
            "'report_policy': 'per_target'} source=voice_case"
        ),
        topic_samples={},
    )

    assert observations["target_selection_observed"] is True
    assert observations["target_selections"][0]["member_ids"] == ["book_1", "cup_1"]
    assert observations["target_selections"][0]["report_policy"] == "per_target"


def test_semantic_oracle_rejects_wrong_selected_member_set():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "ordered_objects",
        "composite",
        "Walk to every object.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("apple_1", "book_1", "phone_1"),
        expected_report_policy="per_target",
    )
    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "planner_request_observed": True,
            "execution_feedback_observed": True,
            "target_selection_observed": True,
            "target_selections": [
                {
                    "member_ids": ["table_1", "person_1"],
                    "recipient_id": "",
                    "report_policy": "per_target",
                }
            ],
            "terminal_observed": True,
            "speech_observed": True,
            "clarification_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert "selected members differed" in " ".join(result["reasons"])


def test_semantic_oracle_allows_extra_live_objects_for_visible_scope():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "all_visible_objects",
        "composite",
        "Walk to every object.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("apple_1", "book_1", "phone_1"),
        expected_report_policy="per_target",
    )
    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "planner_request_observed": True,
            "execution_feedback_observed": True,
            "target_selection_observed": True,
            "target_selections": [
                {
                    "selection_kind": "visible_objects",
                    "member_ids": [
                        "apple_1",
                        "book_1",
                        "phone_1",
                        "live_cup",
                    ],
                    "recipient_id": "",
                    "report_policy": "per_target",
                }
            ],
            "terminal_observed": True,
            "speech_observed": True,
            "clarification_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "pass"


def test_empty_target_selection_is_not_semantic_evidence():
    module = _load_questionnaire_module()
    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt="planner request target_selection={} source=voice_case",
        topic_samples={},
    )

    assert observations["target_selection_observed"] is False
    assert observations["target_selections"] == []


def test_fallback_markers_deduplicate_mirrored_rosout_lines():
    module = _load_questionnaire_module()
    evidence = "\n".join(
        (
            "1783999158.933 [node] [INFO] [1783999158.933366] "
            "ROUTE_RESOLVED source=llm_response_route_repair turn=turn_1",
            "[INFO] [1783999158.933366] ROUTE_RESOLVED "
            "source=llm_response_route_repair turn=turn_1",
            "[INFO] [1783999160.100000] ROUTE_RESOLVED "
            "source=llm_response_route_repair turn=turn_2",
        )
    )

    markers = module.fallback_markers(evidence)

    assert markers["route_repair"] == 2
    assert markers["total"] == 2


def test_fixture_cleanup_uses_two_world_snapshots_and_filters_locally(monkeypatch):
    module = _load_questionnaire_module()
    query_calls = []
    query_results = [
        {
            "raw_output": "before",
            "rows": [
                {
                    "subject": "codex_cup",
                    "predicate": "oro:isAt",
                    "object": "codex_person",
                },
                {
                    "subject": "unrelated",
                    "predicate": "oro:isAt",
                    "object": "elsewhere",
                },
            ],
        },
        {"raw_output": "after", "rows": []},
    ]
    revised = []

    def fake_query(_container, *, patterns, query_vars, timeout_sec):
        query_calls.append((patterns, query_vars, timeout_sec))
        return query_results.pop(0)

    def fake_revise(_container, _service, _service_type, request_yaml, *, timeout_sec):
        revised.append(request_yaml)
        return "success"

    monkeypatch.setattr(module, "query_kb_rows", fake_query)
    monkeypatch.setattr(module, "call_ros_service", fake_revise)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)

    result = module.retract_environment_fixtures(
        "nao_ros2",
        ("fixture",),
        environment_fixtures={
            "fixture": {
                "statements": [
                    "codex_cup rdf:type Cup",
                    "codex_person rdf:type Human",
                ]
            }
        },
    )

    assert len(query_calls) == 2
    assert all(call[0] == ("?subject ?predicate ?object",) for call in query_calls)
    assert "codex_cup oro:isAt codex_person" in revised[0]
    assert "unrelated oro:isAt elsewhere" not in revised[0]
    assert result["contaminated"] is False


def test_case_injection_cleanup_retracts_post_effects_touching_fixture_subjects(monkeypatch):
    module = _load_questionnaire_module()
    query_results = [
        {
            "raw_output": "before",
            "rows": [
                {
                    "subject": "codex_cup",
                    "predicate": "oro:isAt",
                    "object": "codex_person",
                },
                {
                    "subject": "unrelated",
                    "predicate": "oro:isAt",
                    "object": "elsewhere",
                },
            ],
        },
        {"raw_output": "after", "rows": []},
    ]
    revised = []

    monkeypatch.setattr(
        module,
        "query_kb_rows",
        lambda *_args, **_kwargs: query_results.pop(0),
    )
    monkeypatch.setattr(
        module,
        "call_ros_service",
        lambda _container, _service, _service_type, request_yaml, **_kwargs: (
            revised.append(request_yaml) or "success"
        ),
    )
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    injection = module.KbInjection(
        object_id="case_fixture",
        statements=(
            "codex_cup rdf:type Cup",
            "codex_person rdf:type Human",
            "codex_cup dbp:poseX 0.00",
            "myself sees codex_cup",
        ),
        query_patterns=("codex_cup ?predicate ?object",),
        query_vars=("?predicate", "?object"),
    )

    result = module.retract_kb_injections("nao_ros2", (injection,))

    assert result["subjects"] == ["codex_cup", "codex_person"]
    assert "codex_cup dbp:poseX 0.00" in revised[0]
    assert "codex_cup oro:isAt codex_person" in revised[0]
    assert "unrelated oro:isAt elsewhere" not in revised[0]
    assert result["contaminated"] is False


def test_recovery_wait_requires_speech_after_terminal_event():
    module = _load_questionnaire_module()
    observations = {
        "terminal_observed": True,
        "speech_observed": True,
        "post_terminal_speech_observed": False,
        "clarification_observed": False,
    }

    assert module.case_wait_complete(observations, "all_success") is True
    assert module.case_wait_complete(observations, "fail_once_navigation") is False

    observations["post_terminal_speech_observed"] = True
    assert module.case_wait_complete(observations, "fail_once_navigation") is True


def test_recovery_wait_accepts_report_speech_after_failure_before_plan_completed():
    module = _load_questionnaire_module()
    observations = {
        "terminal_observed": True,
        "speech_observed": True,
        "post_terminal_speech_observed": False,
        "post_failure_speech_observed": True,
        "clarification_observed": False,
    }

    assert module.case_wait_complete(observations, "fail_once_navigation") is True


def test_post_failure_speech_excludes_initial_acknowledgement():
    module = _load_questionnaire_module()
    logs = "\n".join(
        (
            '[100.0] [ROBOT OUTPUT] "I will try that now."',
            '[101.0] event_type=step_failed step=navigate_to',
            '[102.0] [ROBOT OUTPUT] "I recovered and reached the cup."',
            '[103.0] event_type=plan_completed',
        )
    )

    assert module.post_failure_speech_observed(logs) is True


def test_thesis_manifests_have_frozen_case_counts():
    module = _load_questionnaire_module()

    assert len(module.MAIN_QUESTIONNAIRE_CASES) == 21
    assert len(module.ENVIRONMENT_CASES) == 11
    assert len(module.FAKE_DEEP_CASES) == 9
    assert len(module.ROBUSTNESS_CASES) == 5
