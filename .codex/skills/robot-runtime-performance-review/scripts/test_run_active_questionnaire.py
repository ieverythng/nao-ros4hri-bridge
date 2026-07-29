import importlib.util
import json
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


def test_structured_trace_preflight_rejects_stale_files_without_writer(monkeypatch):
    module = _load_questionnaire_module()
    monkeypatch.setattr(
        module,
        "run",
        lambda *_args, **_kwargs: "TRACE_NODE_COUNT=0\nTRACE_FILE_COUNT=2\n",
    )

    result = module.collect_structured_trace_events("nao_ros2", 0.0)

    assert result["available"] is False
    assert result["file_count"] == 2
    assert result["writer_node_count"] == 0


def test_structured_trace_preflight_accepts_live_writer_and_trace_file(monkeypatch):
    module = _load_questionnaire_module()
    monkeypatch.setattr(
        module,
        "run",
        lambda *_args, **_kwargs: "TRACE_NODE_COUNT=1\nTRACE_FILE_COUNT=1\n",
    )

    result = module.collect_structured_trace_events("nao_ros2", 0.0)

    assert result["available"] is True
    assert result["writer_node_count"] == 1


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


def test_questionnaire_metadata_records_generation_and_timeout_configuration(monkeypatch):
    module = _load_questionnaire_module()
    calls = []

    def fake_get_ros_params(_container, node_name, param_names):
        calls.append((node_name, tuple(param_names)))
        values = {
            ("/chatbot_llm", "turn_pipeline_mode"): "response_first",
            ("/chatbot_llm", "grounded_context_digest_enabled"): False,
            ("/chatbot_llm", "model"): "QuantTrio/Qwen3.6-35B-A3B-AWQ",
            ("/chatbot_llm", "intent_model"): "QuantTrio/Qwen3.6-35B-A3B-AWQ",
            ("/chatbot_llm", "temperature"): 0.7,
            ("/chatbot_llm", "top_p"): 0.8,
            ("/chatbot_llm", "top_k"): 20,
            ("/chatbot_llm", "min_p"): 0.0,
            ("/chatbot_llm", "presence_penalty"): 1.5,
            ("/chatbot_llm", "repetition_penalty"): 1.0,
            ("/chatbot_llm", "response_max_tokens"): 256,
            ("/chatbot_llm", "intent_max_tokens"): 256,
            ("/chatbot_llm", "request_timeout_sec"): 30.0,
            ("/chatbot_llm", "first_request_timeout_sec"): 60.0,
            ("/chatbot_llm", "intent_request_timeout_sec"): 20.0,
            ("/chatbot_llm", "think"): False,
            ("/planner_llm", "provider"): "openai_compatible",
            ("/planner_llm", "model"): "QuantTrio/Qwen3.6-35B-A3B-AWQ",
            ("/planner_llm", "temperature"): 0.7,
            ("/planner_llm", "top_p"): 0.8,
            ("/planner_llm", "top_k"): 20,
            ("/planner_llm", "min_p"): 0.0,
            ("/planner_llm", "presence_penalty"): 1.5,
            ("/planner_llm", "repetition_penalty"): 1.0,
            ("/planner_llm", "max_tokens"): 1024,
            ("/planner_llm", "timeout_sec"): 45.0,
            ("/planner_llm", "think"): False,
        }
        return {name: values[(node_name, name)] for name in param_names}

    monkeypatch.setattr(module, "get_ros_params", fake_get_ros_params)

    metadata = module.collect_questionnaire_metadata(
        "nao_ros2",
        expected_turn_pipeline_mode="response_first",
    )

    assert metadata["chatbot_turn_pipeline_mode"] == "response_first"
    assert metadata["grounded_context_digest_enabled"] is False
    assert metadata["turn_pipeline_mode_matches_expected"] is True
    assert metadata["chatbot_generation"] == {
        "model": "QuantTrio/Qwen3.6-35B-A3B-AWQ",
        "intent_model": "QuantTrio/Qwen3.6-35B-A3B-AWQ",
        "temperature": 0.7,
        "top_p": 0.8,
        "top_k": 20,
        "min_p": 0.0,
        "presence_penalty": 1.5,
        "repetition_penalty": 1.0,
        "response_max_tokens": 256,
        "intent_max_tokens": 256,
        "request_timeout_sec": 30.0,
        "first_request_timeout_sec": 60.0,
        "intent_request_timeout_sec": 20.0,
        "think": False,
    }
    assert metadata["planner_generation"] == {
        "provider": "openai_compatible",
        "model": "QuantTrio/Qwen3.6-35B-A3B-AWQ",
        "temperature": 0.7,
        "top_p": 0.8,
        "top_k": 20,
        "min_p": 0.0,
        "presence_penalty": 1.5,
        "repetition_penalty": 1.0,
        "max_tokens": 1024,
        "timeout_sec": 45.0,
        "think": False,
    }
    assert [node_name for node_name, _ in calls] == ["/chatbot_llm", "/planner_llm"]


def test_parse_ros_param_dump_extracts_typed_top_level_parameters():
    module = _load_questionnaire_module()
    output = """/chatbot_llm:
  ros__parameters:
    knowledge_patterns:
    - myself sees ?entity
      && ?entity rdf:type ?type
    model: QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ
    temperature: 0.2
    top_k: 20
    think: false
    api_key: ''
    nested_values:
    - ignored
"""

    assert module.parse_ros_param_dump(output, "/chatbot_llm") == {
        "model": "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ",
        "temperature": 0.2,
        "top_k": 20,
        "think": False,
        "api_key": "",
        "nested_values": "",
        "knowledge_patterns": "",
    }


def test_get_ros_param_parses_typed_ros_cli_values(monkeypatch):
    module = _load_questionnaire_module()
    outputs = iter(
        (
            "String value is: qwen-model\n",
            "Double value is: 0.7\n",
            "Integer value is: 20\n",
            "Boolean value is: False\n",
        )
    )
    monkeypatch.setattr(module, "run", lambda *_args, **_kwargs: next(outputs))

    assert module.get_ros_param("nao_ros2", "/chatbot_llm", "model") == "qwen-model"
    assert module.get_ros_param("nao_ros2", "/chatbot_llm", "temperature") == 0.7
    assert module.get_ros_param("nao_ros2", "/chatbot_llm", "top_k") == 20
    assert module.get_ros_param("nao_ros2", "/chatbot_llm", "think") is False


def test_voice_speech_topic_uses_integrated_remap_for_shared_speaker():
    module = _load_questionnaire_module()

    assert (
        module._voice_speech_topic("anonymous_speaker")
        == "/nao_chatbot/humans/voices/anonymous_speaker/speech"
    )


def test_smoke_reflective_followup_shares_the_composite_turn_conversation():
    module = _load_questionnaire_module()
    cases = {case.name: case for case in module.SMOKE_CASES}

    assert cases["composite_head_wave"].conversation_group == "head_wave_reflection"
    assert cases["reflective_followup"].conversation_group == "head_wave_reflection"
    assert cases["reflective_followup"].wait_sec >= 20.0
    assert cases["reflective_followup"].expected_outcome == "dialogue_only"
    assert cases["reflective_followup"].expected_speech_terms == ("four",)


def test_smoke_dialogue_windows_prevent_late_speech_cross_case_overlap():
    module = _load_questionnaire_module()
    cases = {case.name: case for case in module.SMOKE_CASES}

    for case_name in (
        "simple_dialogue_hey",
        "kb_visible_now",
        "kb_injected_object_name",
        "reflective_followup",
    ):
        assert cases[case_name].wait_sec >= 30.0


def test_speech_dialogue_categories_receive_global_observation_floor():
    module = _load_questionnaire_module()
    short_dialogue = module.ProbeCase(
        "short",
        "kb_query_dialogue",
        "What can you see?",
        8.0,
    )
    long_execution = module.ProbeCase(
        "long",
        "simple_skill_execution",
        "Wave.",
        90.0,
    )

    assert module.effective_case_wait_sec(short_dialogue, "speech") == 30.0
    assert module.effective_case_wait_sec(short_dialogue, "chatbot_service") == 8.0
    assert module.effective_case_wait_sec(long_execution, "speech") == 90.0
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


def test_kb_stress_manifest_covers_relation_revision_execution_and_postconditions():
    module = _load_questionnaire_module()

    cases = module.KB_STRESS_CASES
    names = {case.name for case in cases}

    assert names == {
        "kb_stress_seed_inventory",
        "kb_stress_revise_support",
        "kb_stress_revised_relation_query",
        "kb_stress_grounded_delivery",
        "kb_stress_delivery_postcondition",
        "kb_stress_move_remaining_object",
        "kb_stress_mixed_final_query",
    }
    assert len({case.conversation_group for case in cases}) == 1
    assert any(case.setup and case.setup.retract_statements for case in cases)
    assert any(case.expected_member_ids for case in cases)
    assert any(case.postcondition is not None for case in cases)


def test_grounding_people_holdout_repeats_current_scene_query_in_one_dialogue():
    module = _load_questionnaire_module()

    cases = {case.name: case for case in module.GROUNDING_PEOPLE_CASES}
    assert set(cases) == {
        "grounding_people_stale_kb_rows",
        "grounding_people_current_scene",
        "grounding_people_repeated_scene",
    }
    assert all(case.expected_outcome == "dialogue_only" for case in cases.values())
    assert {case.conversation_group for case in cases.values()} == {
        "grounding_people_repeat",
        "grounding_people_stale_kb",
    }


def test_capability_extreme_manifest_is_seeded_and_compositional():
    module = _load_questionnaire_module()

    cases = module.CAPABILITY_EXTREME_CASES
    names = {case.name for case in cases}

    assert module.CAPABILITY_EXTREME_SEED == 20260716
    assert len(cases) == 7
    assert names == {
        "extreme_walk_pick_sit_report",
        "extreme_kneel_under_table_pick_report",
        "extreme_dialogue_inventory",
        "extreme_dialogue_sit_stand_grab_return",
        "extreme_all_objects_visit_look_wave_sit",
        "extreme_pick_place_kneel_report",
        "extreme_unreachable_object_recovery",
    }
    combined = " ".join(case.text.lower() for case in cases)
    for action in ("walk", "pick", "sit", "stand", "kneel", "look"):
        assert action in combined
    assert any(term in combined for term in ("report", "summarize", "tell me"))
    assert any(case.requires_target_selection for case in cases)
    assert any(case.absence_guards for case in cases)
    assert len({case.conversation_group for case in cases if case.conversation_group}) < len(cases)


def test_capability_extreme_generation_is_reproducible():
    module = _load_questionnaire_module()

    first = module._build_capability_extreme_cases(module.CAPABILITY_EXTREME_SEED)
    second = module._build_capability_extreme_cases(module.CAPABILITY_EXTREME_SEED)
    different = module._build_capability_extreme_cases(module.CAPABILITY_EXTREME_SEED + 1)

    assert [case.text for case in first] == [case.text for case in second]
    assert [case.text for case in first] != [case.text for case in different]


def test_formal_main_suite_has_a_full_run_timeout_budget():
    module = _load_questionnaire_module()

    assert module.DEFAULT_GLOBAL_TIMEOUT_SEC >= 1200


def test_route_ack_pair_covers_capability_polite_explicit_and_followup_turns():
    module = _load_questionnaire_module()

    cases = {case.name: case for case in module.ROUTE_ACK_CASES}
    assert set(cases) == {
        "route_ack_capability_question",
        "route_ack_polite_execution",
        "route_ack_polite_followup",
        "route_ack_explicit_execution",
    }
    assert cases["route_ack_capability_question"].expected_outcome == "dialogue_only"
    assert cases["route_ack_polite_execution"].expected_outcome == "execute_no_clarification"
    assert cases["route_ack_explicit_execution"].expected_outcome == "execute_no_clarification"
    assert (
        cases["route_ack_polite_execution"].conversation_group
        == cases["route_ack_polite_followup"].conversation_group
    )


def test_phase_observations_reports_route_intent_gap_separately_from_fallbacks():
    module = _load_questionnaire_module()
    observations = module.phase_observations(
        mode="speech",
        turn_result="",
        log_excerpt=(
            "[1784164683.181] ROUTE_INTENT_HANDOFF route=execution "
            "normalized_intents=[]\n"
            "[1784164683.182] ROUTE_INTENT_GAP execution admitted without "
            "normalized_intents"
        ),
        topic_samples={},
        voice_id="",
    )

    assert observations["route_intent_handoff_observed"] is True
    assert observations["route_intent_gap_count"] == 1
    assert observations["fallback_markers"]["total"] == 0


def test_composite_walk_every_object_case_requires_semantic_selection():
    module = _load_questionnaire_module()
    case = next(
        case
        for case in module.COMPOSITE_CASES
        if case.name == "composite_walk_every_object_reports"
    )

    assert case.requires_target_selection is True
    assert case.expected_member_ids == (
        "codex_probe_apple",
        "codex_probe_book",
        "codex_probe_phone",
    )
    assert case.expected_report_policy == "per_target"


def test_kb_probe_retracts_replaced_relations_before_updating(monkeypatch):
    module = _load_questionnaire_module()
    requests = []

    monkeypatch.setattr(module, "run", lambda *_args, **_kwargs: "services ready")
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        module,
        "call_ros_service",
        lambda _container, _service, _type, request, **_kwargs: requests.append(request)
        or "success",
    )
    monkeypatch.setattr(
        module,
        "_wait_for_fixture_type_rows",
        lambda *_args, **_kwargs: {
            "raw_output": "ready",
            "readiness": {"ready": True},
        },
    )

    result = module.inject_kb_probe(
        "nao_ros2",
        module.KbInjection(
            object_id="support_revision",
            statements=("cup oro:isOn shelf",),
            retract_statements=("cup oro:isOn table",),
            query_patterns=("cup oro:isOn ?support",),
            query_vars=("?support",),
        ),
        lifespan_sec=300,
    )

    assert requests[0].lstrip().startswith("method: retract")
    assert "cup oro:isOn table" in requests[0]
    assert requests[1].lstrip().startswith("method: update")
    assert "cup oro:isOn shelf" in requests[1]
    assert result["retract_output"] == "success"


def test_kb_postcondition_requires_rows_and_declared_values(monkeypatch):
    module = _load_questionnaire_module()

    monkeypatch.setattr(
        module,
        "query_kb_rows",
        lambda *_args, **_kwargs: {
            "raw_output": "query complete",
            "rows": [{"recipient": "codex_stress_alex"}],
        },
    )
    check = module.KbPostcondition(
        "cup_delivered",
        ("codex_stress_cup oro:isAt ?recipient",),
        ("?recipient",),
        min_rows=1,
        expected_values=("codex_stress_alex",),
    )

    passed = module.evaluate_kb_postcondition("nao_ros2", check)

    assert passed["passed"] is True
    assert passed["row_count"] == 1
    assert passed["missing_values"] == []

    monkeypatch.setattr(
        module,
        "query_kb_rows",
        lambda *_args, **_kwargs: {"raw_output": "empty", "rows": []},
    )
    failed = module.evaluate_kb_postcondition("nao_ros2", check)

    assert failed["passed"] is False
    assert failed["missing_values"] == ["codex_stress_alex"]


def test_execution_case_fails_when_declared_kb_postcondition_is_missing():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "deliver_cup",
        "kb_stress_execution",
        "Bring the cup to ALEX.",
        expected_outcome="execute_no_clarification",
        postcondition=module.KbPostcondition(
            "cup_delivered",
            ("cup oro:isAt alex",),
        ),
    )
    observations = {
        "turn_injected": True,
        "planner_request_observed": True,
        "execution_feedback_observed": True,
        "terminal_observed": True,
        "speech_observed": True,
        "clarification_observed": False,
        "kb_postcondition_passed": False,
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert "KB postcondition" in " ".join(result["reasons"])


def test_grounded_dialogue_requires_declared_terms_in_robot_speech():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "grounded_query",
        "kb_stress_dialogue",
        "Where is the cup?",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "storage shelf"),
    )
    observations = {
        "turn_injected": True,
        "route_observed": True,
        "speech_observed": True,
        "spoken_texts": ["TITAS is on the storage shelf."],
        "planner_request_observed": False,
        "execution_feedback_observed": False,
        "fallback_markers": {"total": 0},
    }

    passed = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
    )
    assert passed["status"] == "pass"

    observations["spoken_texts"] = ["I can see a cup."]
    failed = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
    )
    assert failed["status"] == "fail"
    assert "TITAS" in " ".join(failed["reasons"])


def test_extract_robot_speech_texts_ignores_user_turn_mirrors():
    module = _load_questionnaire_module()
    logs = "\n".join(
        (
            'runtime_review_rqt_input: Where is TITAS?',
            '\x1b[0m[INFO] [robot_speech_debug]: [ROBOT OUTPUT] '
            '(closed_caption) "TITAS is on the storage shelf."\x1b[0m',
        )
    )

    assert module.extract_robot_speech_texts(logs) == [
        "TITAS is on the storage shelf."
    ]


def test_incremental_payload_is_not_marked_finished(tmp_path):
    module = _load_questionnaire_module()
    out_path = tmp_path / "questionnaire.json"

    module.write_payload(
        str(out_path),
        "nao_ros2",
        "fake_deep",
        100.0,
        [],
        runtime_metadata={"profile": "all_success"},
    )

    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["run_status"] == "running"
    assert "updated_at_unix_sec" in payload
    assert "finished_at_unix_sec" not in payload


def test_speech_term_comparison_normalizes_symbolic_separators():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "location_query",
        "kb_stress",
        "Where are TITAS and MIDAS?",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "storage shelf", "MIDAS", "work table"),
    )

    result = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "route_observed": True,
            "planner_request_observed": False,
            "execution_feedback_observed": False,
            "speech_observed": True,
            "spoken_texts": [
                "TITAS is on storage_shelf and MIDAS is on work_table."
            ],
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
    )

    assert result["status"] == "pass"


def test_phase_observations_keeps_robot_speech_without_voice_id():
    module = _load_questionnaire_module()

    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt=(
            '[ROBOT OUTPUT] (closed_caption) '
            '"TITAS is on the storage shelf."'
        ),
        topic_samples={},
        voice_id="kb_stress_chain",
    )

    assert observations["spoken_texts"] == [
        "TITAS is on the storage shelf."
    ]


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
        "clarification_speech_observed": True,
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


def test_planner_clarification_requires_correlated_clarification_speech():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "missing_recipient",
        "robustness",
        "Bring every object to MORGAN.",
        expected_outcome="clarification_expected",
    )
    observations = {
        "turn_injected": True,
        "planner_request_observed": True,
        "execution_feedback_observed": False,
        "terminal_observed": True,
        "speech_observed": True,
        "clarification_observed": True,
        "clarification_speech_observed": False,
        "planner_dialogue_acts": ["ask_clarification"],
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(case, observations=observations, stale_world_guard=None)

    assert result["status"] == "fail"
    assert "correlated speech" in " ".join(result["reasons"])


def test_planner_clarification_waits_for_post_act_clarification_speech():
    module = _load_questionnaire_module()
    observations = {
        "terminal_observed": True,
        "speech_observed": True,
        "planner_dialogue_acts": ["ask_clarification"],
        "post_terminal_speech_observed": False,
        "clarification_speech_observed": False,
    }

    assert not module.case_wait_complete(observations, "none")

    observations["post_terminal_speech_observed"] = True
    observations["clarification_speech_observed"] = True
    assert module.case_wait_complete(observations, "none")


def test_missing_person_wording_counts_as_clarification():
    module = _load_questionnaire_module()

    assert module.clarification_observed(
        "I cannot confirm that person in the current grounded context. "
        "Which person should I use for the task?"
    )


def test_capitalized_clarification_question_counts_as_clarification():
    module = _load_questionnaire_module()

    assert module.clarification_observed("Which person should I use for the task?")


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


def test_fallback_markers_track_target_selection_recovery():
    module = _load_questionnaire_module()

    markers = module.fallback_markers(
        "planner decision mode=validated_target_selection_recovery steps=3"
    )

    assert markers["planner_target_selection_recovery"] == 1
    assert markers["total"] == 1


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


def test_missing_object_case_uses_local_not_found_fake_policy():
    module = _load_questionnaire_module()

    case = next(
        case
        for case in module.FAKE_DEEP_CASES
        if case.name == "fake_deep_missing_object_recovery"
    )

    assert case.fake_mode_overrides == (("find_object", "always_fail"),)


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
        "executed_skills": ["pick_object"],
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


def test_failure_profile_is_not_scored_when_different_skill_failed():
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
            "post_terminal_speech_observed": True,
            "failure_observed": True,
            "executed_skills": ["bring_object"],
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
        fake_policy_profile="fail_once_pick",
    )

    assert result["status"] == "not_scored"
    assert "pick_object" in " ".join(result["reasons"])


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
            "executed_skills": ["navigate_to"],
            "replan_observed": False,
            "clarification_observed": False,
            "fallback_markers": {"total": 0},
        },
        stale_world_guard=None,
        fake_policy_profile="fail_once_navigation",
    )

    assert result["status"] == "fail"
    assert "did not produce a replan" in " ".join(result["reasons"])


def test_case_assessment_requires_ordered_skills_and_rejects_forbidden_delivery():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "robot_return",
        "capability_extreme",
        "Pick up MIDAS and return to ALEX.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        expected_skill_sequence=("pick_object", "navigate_to", "report_result"),
        forbidden_skills=("bring_object",),
    )
    observations = {
        "turn_injected": True,
        "planner_request_observed": True,
        "execution_feedback_observed": True,
        "terminal_observed": True,
        "speech_observed": True,
        "clarification_observed": False,
        "failure_observed": False,
        "executed_skills": ["pick_object", "bring_object", "report_result"],
        "executed_skill_sequence": ["pick_object", "bring_object", "report_result"],
        "fallback_markers": {"total": 0},
    }

    result = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
    )

    assert result["status"] == "fail"
    assert "missing ordered skills" in " ".join(result["reasons"])
    assert "forbidden skills" in " ".join(result["reasons"])


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


def test_structured_trace_correlates_injected_failure_and_replan():
    module = _load_questionnaire_module()
    events = [
        {
            "timestamp": 100.0,
            "event_type": "chatbot_turn_trace",
            "channel": "/chatbot_llm/turn_trace",
            "payload": {
                "event_type": "chatbot_turn_result",
                "turn_id": "turn_case",
                "route": "execution",
                "intent": "navigate_to",
                "verbal_ack": "I will visit each object.",
                "planner_handoff_allowed": True,
                "planner_handoff_published": True,
            },
        },
        {
            "timestamp": 100.1,
            "event_type": "planner_request",
            "channel": "/planner/request",
            "payload": {
                "data": {
                    "dialogue_turn_id": "turn_case",
                    "goal_id": "goal_case",
                    "target_selection": {
                        "operation": "visit",
                        "member_ids": ["apple"],
                    },
                }
            },
        },
        {
            "timestamp": 101.0,
            "event_type": "execution_feedback",
            "channel": "/planner/execution_feedback",
            "payload": {
                "event_type": "step_failed",
                "goal_id": "goal_case",
                "plan_id": "plan_1",
                "plan_version": 1,
                "step": {"name": "navigate_to"},
                "result_payload": {
                    "skill": "navigate_to",
                    "status": "failed",
                    "metadata": {
                        "result_mode": "fail_once",
                        "mode_source": "skill_override",
                    },
                },
            },
        },
        {
            "timestamp": 102.0,
            "event_type": "execution_feedback",
            "channel": "/planner/execution_feedback",
            "payload": {
                "event_type": "plan_accepted",
                "goal_id": "goal_case",
                "plan_id": "plan_2",
                "plan_version": 2,
            },
        },
        {
            "timestamp": 103.0,
            "event_type": "execution_feedback",
            "channel": "/planner/execution_feedback",
            "payload": {
                "event_type": "plan_completed",
                "goal_id": "goal_case",
                "plan_id": "plan_2",
                "plan_version": 2,
            },
        },
    ]

    observations = module.structured_phase_observations(
        mode="speech",
        turn_result="published",
        events=events,
        turn_started_at=99.5,
    )

    assert observations["correlation_status"] == "complete"
    assert observations["failure_observed"] is True
    assert observations["replan_observed"] is True
    assert observations["terminal_observed"] is True
    assert observations["executed_skills"] == ["navigate_to"]
    assert observations["failure_injection"] == {
        "observed": True,
        "result_modes": ["fail_once"],
        "mode_sources": ["skill_override"],
    }
    assert observations["lineage"]["goal_id"] == "goal_case"
    assert observations["lineage"]["plan_ids"] == ["plan_1", "plan_2"]


def test_structured_trace_excludes_unrelated_goal_failure():
    module = _load_questionnaire_module()
    events = [
        {
            "timestamp": 100.0,
            "event_type": "chatbot_turn_trace",
            "payload": {
                "event_type": "chatbot_turn_result",
                "turn_id": "turn_case",
                "route": "execution",
                "planner_handoff_allowed": True,
                "planner_handoff_published": True,
            },
        },
        {
            "timestamp": 100.1,
            "event_type": "planner_request",
            "payload": {
                "data": {
                    "dialogue_turn_id": "turn_case",
                    "goal_id": "goal_case",
                }
            },
        },
        {
            "timestamp": 100.2,
            "event_type": "execution_feedback",
            "payload": {
                "event_type": "step_failed",
                "goal_id": "goal_other",
                "plan_id": "plan_other",
                "step": {"name": "pick_object"},
            },
        },
        {
            "timestamp": 100.3,
            "event_type": "execution_feedback",
            "payload": {
                "event_type": "plan_completed",
                "goal_id": "goal_case",
                "plan_id": "plan_case",
            },
        },
    ]

    observations = module.structured_phase_observations(
        mode="speech",
        turn_result="published",
        events=events,
        turn_started_at=99.5,
    )

    assert observations["failure_observed"] is False
    assert observations["executed_skills"] == []
    assert observations["lineage"]["plan_ids"] == ["plan_case"]


def test_structured_trace_refuses_execution_score_without_planner_lineage():
    module = _load_questionnaire_module()
    observations = module.structured_phase_observations(
        mode="speech",
        turn_result="published",
        events=[
            {
                "timestamp": 100.0,
                "event_type": "chatbot_turn_trace",
                "payload": {
                    "event_type": "chatbot_turn_result",
                    "turn_id": "turn_case",
                    "route": "execution",
                    "planner_handoff_allowed": True,
                    "planner_handoff_published": True,
                },
            }
        ],
        turn_started_at=99.5,
    )

    assert observations["correlation_status"] == "incomplete"
    assert observations["evidence_consistent"] is False
    assert "planner request" in " ".join(observations["evidence_inconsistencies"])


def test_structured_trace_scores_correlated_planner_failure_as_semantic_evidence():
    module = _load_questionnaire_module()
    observations = module.structured_phase_observations(
        mode="speech",
        turn_result="published",
        events=[
            {
                "timestamp": 100.0,
                "event_type": "chatbot_turn_trace",
                "payload": {
                    "event_type": "chatbot_turn_result",
                    "turn_id": "turn_case",
                    "route": "execution",
                    "planner_handoff_allowed": True,
                    "planner_handoff_published": True,
                },
            },
            {
                "timestamp": 100.1,
                "event_type": "planner_request",
                "payload": {
                    "data": {
                        "dialogue_turn_id": "turn_case",
                        "goal_id": "goal_case",
                    }
                },
            },
            {
                "timestamp": 100.2,
                "event_type": "planner_dialogue_act",
                "payload": {
                    "act": "explain_failure",
                    "goal_id": "goal_case",
                    "plan_id": "plan_case",
                    "plan_version": 1,
                },
            },
        ],
        turn_started_at=99.5,
    )

    assert observations["correlation_status"] == "complete"
    assert observations["execution_feedback_observed"] is False
    assert observations["failure_observed"] is True
    assert observations["terminal_observed"] is True
    assert observations["planner_dialogue_acts"] == ["explain_failure"]


def test_assessment_does_not_score_missing_required_grounded_fixture():
    module = _load_questionnaire_module()
    case = module.ProbeCase(
        "missing_fixture",
        "grounding",
        "return to ALEX",
        1.0,
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        expected_member_ids=("person_alex",),
    )

    assessment = module.assess_case(
        case,
        observations={
            "turn_injected": True,
            "evidence_consistent": True,
            "grounded_entity_ids": [],
        },
        fake_policy_profile="all_success",
        stale_world_guard=None,
    )

    assert assessment["status"] == "not_scored"
    assert "person_alex" in assessment["reasons"][0]


def test_parse_structured_trace_jsonl_deduplicates_mirrored_files():
    module = _load_questionnaire_module()
    event = {
        "timestamp": 100.0,
        "trace_id": "goal_1",
        "channel": "/planner/execution_feedback",
        "event_type": "execution_feedback",
        "payload": {"event_type": "step_failed", "goal_id": "goal_1"},
    }
    chunks = [
        json.dumps(event) + "\ninvalid",
        json.dumps(event)
        + "\n"
        + json.dumps({**event, "timestamp": 90.0, "trace_id": "old"}),
    ]

    parsed = module.parse_structured_trace_jsonl(chunks, since_unix_sec=99.0)

    assert parsed == [event]


def test_execution_feedback_without_upstream_markers_is_not_scored():
    module = _load_questionnaire_module()
    observations = module.phase_observations(
        mode="speech",
        turn_result="published",
        log_excerpt="event_type=step_succeeded\nevent_type=plan_completed",
        topic_samples={},
    )
    case = module.ProbeCase(
        "grouped_delivery",
        "fake_deep",
        "Bring every object from the table to ALEX.",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    )

    result = module.assess_case(
        case,
        observations=observations,
        stale_world_guard=None,
    )

    assert observations["evidence_consistent"] is False
    assert result["status"] == "not_scored"
    assert "correlation" in " ".join(result["reasons"])


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
