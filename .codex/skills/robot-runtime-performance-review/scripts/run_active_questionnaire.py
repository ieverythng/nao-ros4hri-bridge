#!/usr/bin/env python3
"""Inject ROS4HRI speech turns and collect runtime evidence per case."""

from __future__ import annotations

import argparse
import ast
import json
import random
import re
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONTAINER = "nao_ros2"
VOICE_ID = "anonymous_speaker"
VOICE_TRACKED_TOPIC = "/nao_chatbot/humans/voices/tracked"
# dialogue_manager receives the tracked voice through the integrated remap, then
# subscribes to the per-voice HRI speech topic it constructs internally. The
# launch profile remaps the shared rqt speaker explicitly; synthetic ids remain
# on the raw dynamic namespace.
VOICE_SPEECH_PREFIX = "/humans/voices"
VOICE_SPEECH_TOPIC = f"/nao_chatbot{VOICE_SPEECH_PREFIX}/{VOICE_ID}/speech"
VOICE_TRACKED_QOS = "--qos-reliability reliable --qos-durability transient_local"
VOICE_SPEECH_QOS = "--qos-reliability reliable --qos-durability volatile"
RQT_DISPLAY_ROSOUT_TOPIC = "/rosout"
RQT_DISPLAY_CAPTIONS_TOPIC = "/dialogue_manager/closed_captions"
RQT_DISPLAY_ROSOUT_QOS = "--qos-reliability reliable --qos-durability transient_local"
TOPIC_SAMPLE_TIMEOUT_SEC = 1.5
TOPIC_SAMPLE_KILL_AFTER_SEC = 1.0
DEFAULT_GLOBAL_TIMEOUT_SEC = 1200
DEFAULT_KB_LIFESPAN_SEC = 300
KB_FIXTURE_READY_TIMEOUT_SEC = 12.0
KB_FIXTURE_READY_POLL_SEC = 0.5
DEFAULT_ENVIRONMENT_FIXTURE_PATH = (
    Path(__file__).resolve().parents[4]
    / "src"
    / "nao_chatbot"
    / "config"
    / "preloaded_environments.json"
)
FALLBACK_ENVIRONMENT_FIXTURE_PATH = (
    Path(__file__).resolve().parent.parent / "references" / "preloaded_environments.json"
)
KB_PROBE_OBJECT_ID = "codex_probe_cup"
KB_MAXIMAL_CUP_ID = "codex_kitchen_cup"
KB_MAXIMAL_LOCATION_ID = "codex_kitchen"
KB_MAXIMAL_ORIGIN_ID = "codex_robot_station"
KB_MAXIMAL_PERSON_ID = "codex_recipient_person"
ROS_CLI_PREAMBLE = """
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
""".strip()


@dataclass(frozen=True)
class KbInjection:
    object_id: str
    statements: tuple[str, ...]
    query_patterns: tuple[str, ...]
    query_vars: tuple[str, ...]
    retract_statements: tuple[str, ...] = ()


@dataclass(frozen=True)
class KbPostcondition:
    """KnowledgeCore query that must hold after the user turn."""

    name: str
    query_patterns: tuple[str, ...]
    query_vars: tuple[str, ...] = ("?subject",)
    min_rows: int = 1
    expected_values: tuple[str, ...] = ()


@dataclass(frozen=True)
class KbAbsenceGuard:
    """Preflight query that must return no rows for a case to be isolated."""

    name: str
    query_patterns: tuple[str, ...]
    query_vars: tuple[str, ...] = ("?predicate", "?object")


@dataclass(frozen=True)
class ProbeCase:
    name: str
    category: str
    text: str
    wait_sec: float = 10.0
    mode: str = "speech"
    setup: KbInjection | None = None
    conversation_group: str | None = None
    environment_ids: tuple[str, ...] = ()
    absence_guards: tuple[KbAbsenceGuard, ...] = ()
    expected_outcome: str = "observe"
    all_required_context: bool = False
    requires_target_selection: bool = False
    requires_replan: bool = False
    expected_member_ids: tuple[str, ...] = ()
    expected_recipient_id: str = ""
    expected_report_policy: str = ""
    postcondition: KbPostcondition | None = None
    expected_speech_terms: tuple[str, ...] = ()
    fake_mode_overrides: tuple[tuple[str, str], ...] = ()
    expected_skill_sequence: tuple[str, ...] = ()
    forbidden_skills: tuple[str, ...] = ()


SMOKE_CASES = (
    ProbeCase("simple_dialogue_hey", "simple_dialogue", "Hey, how are you?", 8.0),
    ProbeCase("kb_visible_now", "kb_query_dialogue", "What can you see?", 10.0),
    ProbeCase(
        "kb_injected_object_name",
        "kb_query_dialogue",
        "What is the name and color of the probe cup?",
        12.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn table_1",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase("simple_wave", "simple_skill_execution", "Wave at me.", 90.0),
    ProbeCase(
        "composite_head_wave",
        "composite_skill_execution",
        "Move your head in all directions and then wave at me.",
        95.0,
    ),
    ProbeCase(
        "reflective_followup",
        "simple_dialogue",
        "How many directions did you move your head?",
        10.0,
        conversation_group="head_wave_reflection",
    ),
)

MAIN_QUESTIONNAIRE_CASES = (
    ProbeCase("dialogue_greeting", "simple_dialogue", "Hey, how are you?", 8.0, conversation_group="dialogue_basic", expected_outcome="dialogue_only"),
    ProbeCase(
        "dialogue_favorite_movie",
        "simple_dialogue",
        "What is your favourite movie?",
        8.0,
        conversation_group="dialogue_basic",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "dialogue_movie_followup",
        "simple_dialogue",
        "My favourite movie is Back to the Future!",
        8.0,
        conversation_group="dialogue_basic",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "dialogue_weekend_plans",
        "simple_dialogue",
        "Any ideas for plans this weekend?",
        8.0,
        conversation_group="dialogue_basic",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "dialogue_speed_of_light",
        "simple_dialogue",
        "What is the speed of light?",
        8.0,
        conversation_group="dialogue_basic",
        expected_outcome="dialogue_only",
    ),
    ProbeCase("kb_visible_now_baseline", "kb_query_dialogue", "What can you see now?", 10.0, expected_outcome="dialogue_only"),
    ProbeCase(
        "kb_probe_scene_update",
        "kb_query_dialogue",
        "What else can you see now?",
        12.0,
        setup=KbInjection(
            object_id="codex_main_scene",
            statements=(
                "myself sees codex_probe_cup",
                "codex_probe_cup rdf:type Cup",
                "codex_probe_cup dbp:name TITAS",
                "codex_probe_cup dbp:color gold",
                "codex_probe_cup oro:isOn codex_probe_table",
                "codex_probe_table rdf:type Table",
                "codex_probe_table dbp:name probe_table",
                "myself sees codex_probe_table",
            ),
            query_patterns=(
                "codex_probe_cup ?predicate ?object",
                "codex_probe_table ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "kb_object_on_table",
        "kb_query_dialogue",
        "What object is on the table?",
        12.0,
        setup=KbInjection(
            object_id="codex_table_scene",
            statements=(
                "myself sees codex_probe_cup",
                "codex_probe_cup rdf:type Cup",
                "codex_probe_cup dbp:name TITAS",
                "codex_probe_cup dbp:color gold",
                "codex_probe_cup oro:isOn codex_probe_table",
                "codex_probe_table rdf:type Table",
                "codex_probe_table dbp:name table",
                "myself sees codex_probe_table",
            ),
            query_patterns=(
                "codex_probe_cup ?predicate ?object",
                "codex_probe_table ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "kb_cup_name",
        "kb_query_dialogue",
        "Can you tell me the name of the cup?",
        12.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn codex_probe_table",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "kb_multi_object_count",
        "kb_query_dialogue",
        "How many objects can you see?",
        12.0,
        setup=KbInjection(
            object_id="codex_count_scene",
            statements=(
                "myself sees codex_count_cup",
                "codex_count_cup rdf:type Cup",
                "codex_count_cup dbp:name TITAS",
                "codex_count_cup dbp:color gold",
                "myself sees codex_count_book",
                "codex_count_book rdf:type Book",
                "codex_count_book dbp:name MIDAS",
                "codex_count_book dbp:color blue",
                "myself sees codex_count_phone",
                "codex_count_phone rdf:type CellularTelephone",
                "codex_count_phone dbp:name VEGA",
                "codex_count_phone dbp:color silver",
            ),
            query_patterns=(
                "codex_count_cup ?predicate ?object",
                "codex_count_book ?predicate ?object",
                "codex_count_phone ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="dialogue_only",
    ),
    ProbeCase("skill_head_up", "simple_skill_execution", "Move your head up.", 80.0, expected_outcome="execute_no_clarification"),
    ProbeCase(
        "skill_pick_phone_generic",
        "simple_fake_skill_execution",
        "Pick up the phone.",
        120.0,
        setup=KbInjection(
            object_id="codex_skill_phone",
            statements=(
                "myself sees codex_skill_phone",
                "codex_skill_phone rdf:type CellularTelephone",
                "codex_skill_phone dbp:name VEGA",
                "codex_skill_phone dbp:color silver",
                "codex_skill_phone oro:isOn codex_probe_table",
                "myself canReach codex_skill_phone",
            ),
            query_patterns=("codex_skill_phone ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "skill_pick_object_on_table",
        "simple_fake_skill_execution",
        "Pick up the object on the table.",
        120.0,
        setup=KbInjection(
            object_id="codex_pick_scene",
            statements=(
                "myself sees codex_table_object",
                "codex_table_object rdf:type Cup",
                "codex_table_object dbp:name TABLE_PROBE",
                "codex_table_object dbp:color red",
                "codex_table_object oro:isOn codex_probe_table",
                "codex_probe_table rdf:type Table",
                "codex_probe_table dbp:name table",
                "myself canReach codex_table_object",
            ),
            query_patterns=(
                "codex_table_object ?predicate ?object",
                "codex_probe_table ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "skill_look_at_person",
        "simple_skill_execution",
        "Look at the person named ALEX.",
        90.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_PERSON_ID,
            statements=(
                f"myself sees {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} rdf:type Human",
                f"{KB_MAXIMAL_PERSON_ID} dbp:name ALEX",
                f"{KB_MAXIMAL_PERSON_ID} dbp:frameId {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseSource semantic_fixture",
            ),
            query_patterns=(f"{KB_MAXIMAL_PERSON_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "kb_mutation_add_red_cup",
        "kb_mutation_dialogue",
        "Add a red cup to your KB.",
        20.0,
        expected_outcome="execute_no_clarification",
    ),
    ProbeCase(
        "composite_head_all_directions",
        "composite_skill_execution",
        "Can you move your head in all directions?",
        95.0,
        expected_outcome="execute_no_clarification",
    ),
    ProbeCase(
        "composite_bring_every_object_to_person",
        "composite_fake_skill_execution",
        "Can you bring every object in view to the person named ALEX?",
        180.0,
        setup=KbInjection(
            object_id="codex_bring_all_scene",
            statements=(
                "myself sees codex_bring_cup",
                "codex_bring_cup rdf:type Cup",
                "codex_bring_cup dbp:name TITAS",
                "codex_bring_cup dbp:color gold",
                "myself sees codex_bring_book",
                "codex_bring_book rdf:type Book",
                "codex_bring_book dbp:name MIDAS",
                "codex_bring_book dbp:color blue",
                f"myself sees {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} rdf:type Human",
                f"{KB_MAXIMAL_PERSON_ID} dbp:name ALEX",
                f"{KB_MAXIMAL_PERSON_ID} dbp:frameId {KB_MAXIMAL_PERSON_ID}",
                "myself canReach codex_bring_cup",
                "myself canReach codex_bring_book",
            ),
            query_patterns=(
                "codex_bring_cup ?predicate ?object",
                "codex_bring_book ?predicate ?object",
                f"{KB_MAXIMAL_PERSON_ID} ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "maximal_kitchen_cup_to_operator",
        "maximal_semantic_execution",
        "Can you go to the kitchen and bring me the cup?",
        160.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_LOCATION_ID} rdf:type Room",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:name kitchen",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:frameId map",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseSource semantic_fixture",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseX 1.20",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseY 0.40",
                f"{KB_MAXIMAL_CUP_ID} rdf:type Cup",
                f"{KB_MAXIMAL_CUP_ID} dbp:name KITCHEN_PROBE_CUP",
                f"{KB_MAXIMAL_CUP_ID} dbp:color white",
                f"{KB_MAXIMAL_CUP_ID} oro:isIn {KB_MAXIMAL_LOCATION_ID}",
                f"myself sees {KB_MAXIMAL_CUP_ID}",
                f"myself canReceiveAt {KB_MAXIMAL_ORIGIN_ID}",
            ),
            query_patterns=(
                f"{KB_MAXIMAL_LOCATION_ID} ?predicate ?object",
                f"{KB_MAXIMAL_CUP_ID} ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="clarification_expected",
    ),
    ProbeCase(
        "composite_go_to_person_wave_report",
        "composite_skill_execution",
        "Go to the person named ALEX and wave at them. Let me know when you've done that.",
        150.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_PERSON_ID,
            statements=(
                f"myself sees {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} rdf:type Human",
                f"{KB_MAXIMAL_PERSON_ID} dbp:name ALEX",
                f"{KB_MAXIMAL_PERSON_ID} dbp:frameId {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseSource semantic_fixture",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseX 0.30",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseY -0.20",
            ),
            query_patterns=(f"{KB_MAXIMAL_PERSON_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "composite_walk_every_object_reports_main",
        "composite_skill_execution",
        "Walk to every object and let me know when you get to each one.",
        160.0,
        setup=KbInjection(
            object_id="codex_walk_all_scene",
            statements=(
                "myself sees codex_probe_apple",
                "codex_probe_apple rdf:type Apple",
                "codex_probe_apple dbp:name ATLAS",
                "codex_probe_apple dbp:color red",
                "codex_probe_apple oro:isOn table_1",
                "myself sees codex_probe_book",
                "codex_probe_book rdf:type Book",
                "codex_probe_book dbp:name MIDAS",
                "codex_probe_book dbp:color blue",
                "codex_probe_book oro:isOn table_1",
                "myself sees codex_probe_phone",
                "codex_probe_phone rdf:type CellularTelephone",
                "codex_probe_phone dbp:name VEGA",
                "codex_probe_phone dbp:color silver",
                "codex_probe_phone oro:isOn table_1",
            ),
            query_patterns=(
                "codex_probe_apple ?predicate ?object",
                "codex_probe_book ?predicate ?object",
                "codex_probe_phone ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=(
            "codex_probe_apple",
            "codex_probe_book",
            "codex_probe_phone",
        ),
        expected_report_policy="per_target",
    ),
    ProbeCase(
        "dialogue_non_action_recipient_memory",
        "route_safety",
        "Remember that ALEX is the recipient, but do not act yet.",
        12.0,
        expected_outcome="dialogue_only",
    ),
)


KB_STRESS_CASES = (
    ProbeCase(
        "kb_stress_seed_inventory",
        "kb_stress_dialogue",
        "What are the names, colors, and current locations of the two objects in the test area?",
        18.0,
        setup=KbInjection(
            object_id="codex_stress_scene",
            statements=(
                "codex_stress_table rdf:type Table",
                "codex_stress_table dbp:name work_table",
                "codex_stress_shelf rdf:type Shelf",
                "codex_stress_shelf dbp:name storage_shelf",
                "codex_stress_cup rdf:type Cup",
                "codex_stress_cup dbp:name TITAS",
                "codex_stress_cup dbp:color gold",
                "codex_stress_cup oro:isOn codex_stress_table",
                "codex_stress_book rdf:type Book",
                "codex_stress_book dbp:name MIDAS",
                "codex_stress_book dbp:color blue",
                "codex_stress_book oro:isOn codex_stress_table",
                "myself sees codex_stress_table",
                "myself sees codex_stress_shelf",
                "myself sees codex_stress_cup",
                "myself sees codex_stress_book",
                "myself canReach codex_stress_cup",
                "myself canReach codex_stress_book",
            ),
            query_patterns=(
                "codex_stress_cup ?predicate ?object",
                "codex_stress_book ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        conversation_group="kb_stress_chain",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "MIDAS", "gold", "blue", "work table"),
    ),
    ProbeCase(
        "kb_stress_revise_support",
        "kb_stress_relation_revision",
        "The cup has been moved. Which surface is TITAS on now, and where is MIDAS?",
        18.0,
        setup=KbInjection(
            object_id="codex_stress_support_revision",
            statements=("codex_stress_cup oro:isOn codex_stress_shelf",),
            retract_statements=("codex_stress_cup oro:isOn codex_stress_table",),
            query_patterns=("codex_stress_cup oro:isOn ?support",),
            query_vars=("?support",),
        ),
        conversation_group="kb_stress_chain",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "storage shelf", "MIDAS", "work table"),
    ),
    ProbeCase(
        "kb_stress_revised_relation_query",
        "kb_stress_dialogue",
        "Which object is on the storage shelf, and which object remains on the work table?",
        18.0,
        conversation_group="kb_stress_chain",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "storage shelf", "MIDAS", "work table"),
    ),
    ProbeCase(
        "kb_stress_grounded_delivery",
        "kb_stress_execution",
        "Bring TITAS from the storage shelf to the person named ALEX and report what happened.",
        150.0,
        setup=KbInjection(
            object_id="codex_stress_recipient",
            statements=(
                "codex_stress_alex rdf:type Human",
                "codex_stress_alex dbp:name ALEX",
                "codex_stress_alex dbp:frameId codex_stress_alex",
                "codex_stress_alex dbp:poseSource semantic_fixture",
                "myself sees codex_stress_alex",
            ),
            query_patterns=("codex_stress_alex ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
        conversation_group="kb_stress_chain",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_stress_cup",),
        expected_recipient_id="codex_stress_alex",
        expected_report_policy="final",
        expected_speech_terms=("TITAS", "ALEX"),
        postcondition=KbPostcondition(
            "titas_delivered_to_alex",
            ("codex_stress_cup oro:isAt ?recipient",),
            ("?recipient",),
            expected_values=("codex_stress_alex",),
        ),
    ),
    ProbeCase(
        "kb_stress_delivery_postcondition",
        "kb_stress_dialogue",
        "Where are TITAS and MIDAS now?",
        18.0,
        conversation_group="kb_stress_chain",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "ALEX", "MIDAS", "work table"),
    ),
    ProbeCase(
        "kb_stress_move_remaining_object",
        "kb_stress_execution",
        "MIDAS is now on the storage shelf. Bring every object on that shelf to ALEX and summarize the overall result.",
        150.0,
        setup=KbInjection(
            object_id="codex_stress_book_revision",
            statements=("codex_stress_book oro:isOn codex_stress_shelf",),
            retract_statements=("codex_stress_book oro:isOn codex_stress_table",),
            query_patterns=("codex_stress_book oro:isOn ?support",),
            query_vars=("?support",),
        ),
        conversation_group="kb_stress_chain",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_stress_book",),
        expected_recipient_id="codex_stress_alex",
        expected_report_policy="final",
        expected_speech_terms=("MIDAS", "ALEX"),
        postcondition=KbPostcondition(
            "midas_delivered_to_alex",
            ("codex_stress_book oro:isAt ?recipient",),
            ("?recipient",),
            expected_values=("codex_stress_alex",),
        ),
    ),
    ProbeCase(
        "kb_stress_mixed_final_query",
        "kb_stress_dialogue",
        "Summarize the names, colors, and current locations of TITAS and MIDAS after those changes.",
        20.0,
        conversation_group="kb_stress_chain",
        expected_outcome="dialogue_only",
        expected_speech_terms=("TITAS", "MIDAS", "gold", "blue", "ALEX"),
    ),
)

CAPABILITY_EXTREME_SEED = 20260716


def _build_capability_extreme_cases(seed: int) -> tuple[ProbeCase, ...]:
    """Build reproducible long-horizon cases without fixed lexical phrasing."""
    rng = random.Random(seed)
    report_phrase = rng.choice(
        ("tell me what happened", "summarize the result", "report the outcome")
    )
    return_phrase = rng.choice(("return to", "go back to", "come back to"))
    all_objects_phrase = rng.choice(
        ("each visible object", "all objects in view", "every object you can see")
    )
    scene = KbInjection(
        object_id="codex_extreme_scene",
        statements=(
            "codex_extreme_table rdf:type Table",
            "codex_extreme_table dbp:name work_table",
            "codex_extreme_shelf rdf:type Shelf",
            "codex_extreme_shelf dbp:name storage_shelf",
            "codex_extreme_cup rdf:type Cup",
            "codex_extreme_cup dbp:name TITAS",
            "codex_extreme_cup dbp:color gold",
            "codex_extreme_cup oro:isOn codex_extreme_table",
            "codex_extreme_book rdf:type Book",
            "codex_extreme_book dbp:name MIDAS",
            "codex_extreme_book dbp:color blue",
            "codex_extreme_book oro:isOn codex_extreme_table",
            "codex_extreme_book dbp:locationHint under_table",
            "codex_extreme_apple rdf:type Apple",
            "codex_extreme_apple dbp:name ATLAS",
            "codex_extreme_apple dbp:color red",
            "codex_extreme_apple oro:isOn codex_extreme_table",
            "codex_extreme_alex rdf:type Human",
            "codex_extreme_alex dbp:name ALEX",
            "codex_extreme_alex dbp:frameId codex_extreme_alex",
            "codex_extreme_alex dbp:poseSource semantic_fixture",
            "myself sees codex_extreme_table",
            "myself sees codex_extreme_shelf",
            "myself sees codex_extreme_cup",
            "myself sees codex_extreme_book",
            "myself sees codex_extreme_apple",
            "myself sees codex_extreme_alex",
            "myself canReach codex_extreme_cup",
            "myself canReach codex_extreme_book",
            "myself canReach codex_extreme_apple",
        ),
        query_patterns=(
            "codex_extreme_cup ?predicate ?object",
            "codex_extreme_book ?predicate ?object",
            "codex_extreme_apple ?predicate ?object",
            "codex_extreme_alex ?predicate ?object",
        ),
        query_vars=("?predicate", "?object"),
    )
    return (
        ProbeCase(
            "extreme_walk_pick_sit_report",
            "capability_extreme_execution",
            f"Walk to TITAS, pick it up, sit down, and {report_phrase}.",
            180.0,
            setup=scene,
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            expected_member_ids=("codex_extreme_cup",),
            expected_report_policy="final",
            expected_skill_sequence=(
                "navigate_to",
                "pick_object",
                "perform_motion",
                "report_result",
            ),
        ),
        ProbeCase(
            "extreme_kneel_under_table_pick_report",
            "capability_extreme_execution",
            f"Kneel down, pick up MIDAS from under the work table, stand, and {report_phrase}.",
            180.0,
            setup=scene,
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            expected_member_ids=("codex_extreme_book",),
            expected_report_policy="final",
            expected_skill_sequence=(
                "perform_motion",
                "pick_object",
                "perform_motion",
                "report_result",
            ),
        ),
        ProbeCase(
            "extreme_dialogue_inventory",
            "capability_extreme_dialogue",
            "Before we do anything, tell me which named objects and people are in the test area.",
            20.0,
            setup=scene,
            conversation_group="extreme_dialogue_chain",
            expected_outcome="dialogue_only",
            expected_speech_terms=("TITAS", "MIDAS", "ATLAS", "ALEX"),
        ),
        ProbeCase(
            "extreme_dialogue_sit_stand_grab_return",
            "capability_extreme_execution",
            f"Now sit, stand again, pick up MIDAS, {return_phrase} ALEX, and {report_phrase}.",
            200.0,
            conversation_group="extreme_dialogue_chain",
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            expected_member_ids=("codex_extreme_alex",),
            expected_report_policy="final",
            expected_skill_sequence=(
                "perform_motion",
                "perform_motion",
                "pick_object",
                "navigate_to",
                "report_result",
            ),
            forbidden_skills=("bring_object", "deliver_object", "place_object"),
        ),
        ProbeCase(
            "extreme_all_objects_visit_look_wave_sit",
            "capability_extreme_execution",
            f"Stand up, walk to {all_objects_phrase} on the work table, look at each one, wave to ALEX, sit down, and give one final summary.",
            240.0,
            setup=scene,
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            requires_target_selection=True,
            expected_member_ids=(
                "codex_extreme_cup",
                "codex_extreme_book",
                "codex_extreme_apple",
            ),
            expected_report_policy="final",
            expected_skill_sequence=(
                "perform_motion",
                "navigate_to",
                "look_at",
                "wave_greet",
                "perform_motion",
                "report_result",
            ),
        ),
        ProbeCase(
            "extreme_pick_place_kneel_report",
            "capability_extreme_execution",
            f"Pick up ATLAS, place it on the storage shelf, kneel, and {report_phrase}.",
            200.0,
            setup=scene,
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            expected_member_ids=("codex_extreme_apple",),
            expected_report_policy="final",
            expected_skill_sequence=(
                "pick_object",
                "place_object",
                "perform_motion",
                "report_result",
            ),
            postcondition=KbPostcondition(
                "atlas_on_storage_shelf",
                ("codex_extreme_apple oro:isOn ?support",),
                ("?support",),
                expected_values=("codex_extreme_shelf",),
            ),
        ),
        ProbeCase(
            "extreme_unreachable_object_recovery",
            "capability_extreme_recovery",
            f"Kneel and try to pick up the heavy box under the table. If you cannot, recover safely and {report_phrase}.",
            200.0,
            setup=KbInjection(
                object_id="codex_extreme_heavy_box",
                statements=(
                    "codex_extreme_heavy_box rdf:type Box",
                    "codex_extreme_heavy_box dbp:name heavy_box",
                    "codex_extreme_heavy_box dbp:locationHint under_table",
                    "codex_extreme_heavy_box oro:isOn codex_extreme_table",
                    "myself sees codex_extreme_heavy_box",
                ),
                query_patterns=("codex_extreme_heavy_box ?predicate ?object",),
                query_vars=("?predicate", "?object"),
            ),
            absence_guards=(
                KbAbsenceGuard(
                    "no_reachability_claim",
                    ("myself canReach codex_extreme_heavy_box",),
                    ("?subject",),
                ),
            ),
            expected_outcome="execute_no_clarification",
            all_required_context=True,
            requires_replan=True,
            expected_report_policy="final",
            expected_skill_sequence=("perform_motion", "pick_object", "report_result"),
        ),
    )


CAPABILITY_EXTREME_CASES = _build_capability_extreme_cases(CAPABILITY_EXTREME_SEED)

GROUNDING_PEOPLE_CASES = (
    ProbeCase(
        "grounding_people_stale_kb_rows",
        "grounding_people_repeat",
        "What people can you see right now?",
        15.0,
        setup=KbInjection(
            object_id="codex_stale_people",
            statements=(
                "anonymous_person_stale rdf:type Human",
                "myself sees anonymous_person_stale",
                "sim_person_stale rdf:type Human",
                "myself sees sim_person_stale",
            ),
            query_patterns=("?subject rdf:type Human",),
            query_vars=("?subject",),
        ),
        conversation_group="grounding_people_stale_kb",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "grounding_people_current_scene",
        "grounding_people_repeat",
        "What people can you see right now?",
        15.0,
        conversation_group="grounding_people_repeat",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "grounding_people_repeated_scene",
        "grounding_people_repeat",
        "Please check again. What people can you see right now?",
        15.0,
        conversation_group="grounding_people_repeat",
        expected_outcome="dialogue_only",
    ),
)

COMPOSITE_CASES = (
    ProbeCase("kb_visible_baseline", "kb_query_dialogue", "What can you see?", 10.0),
    ProbeCase(
        "kb_injected_probe_baseline",
        "kb_query_dialogue",
        "What is the name and color of the probe cup?",
        12.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn table_1",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "composite_head_wave",
        "composite_skill_execution",
        "Move your head in all directions and then wave at me.",
        95.0,
        conversation_group="head_wave_reflection",
    ),
    ProbeCase(
        "composite_walk_every_object_reports",
        "composite_skill_execution",
        "Now walk to every object, let me know when you are there and then walk to the next!",
        120.0,
        setup=KbInjection(
            object_id="codex_multi_object_scene",
            statements=(
                "myself sees codex_probe_apple",
                "codex_probe_apple rdf:type Apple",
                "codex_probe_apple dbp:name ATLAS",
                "codex_probe_apple dbp:color red",
                "codex_probe_apple oro:isOn table_1",
                "myself sees codex_probe_book",
                "codex_probe_book rdf:type Book",
                "codex_probe_book dbp:name MIDAS",
                "codex_probe_book dbp:color blue",
                "codex_probe_book oro:isOn table_1",
                "myself sees codex_probe_phone",
                "codex_probe_phone rdf:type CellularTelephone",
                "codex_probe_phone dbp:name VEGA",
                "codex_probe_phone dbp:color silver",
                "codex_probe_phone oro:isOn table_1",
            ),
            query_patterns=(
                "codex_probe_apple ?predicate ?object",
                "codex_probe_book ?predicate ?object",
                "codex_probe_phone ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=(
            "codex_probe_apple",
            "codex_probe_book",
            "codex_probe_phone",
        ),
        expected_report_policy="per_target",
    ),
    ProbeCase(
        "composite_look_at_probe_report",
        "composite_skill_execution",
        "Look at the probe cup and then tell me what you did.",
        80.0,
    ),
    ProbeCase(
        "favorite_movie_dialogue_holdout",
        "route_prompt_ambiguity",
        "What is your favorite movie?",
        12.0,
    ),
    ProbeCase(
        "composite_navigate_probe_report",
        "composite_skill_execution",
        "Navigate to the probe cup and then tell me what else you see.",
        90.0,
    ),
    ProbeCase(
        "maximal_kitchen_cup_bring",
        "maximal_semantic_execution",
        "I am at the robot station. There is a cup in the kitchen. Go to the kitchen, pick up the cup, bring it back to me at the robot station, and report what happened.",
        160.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_ORIGIN_ID} rdf:type Place",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:name robot_station",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:frameId map",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:poseSource semantic_fixture",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:poseX 0.00",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:poseY 0.00",
                f"{KB_MAXIMAL_LOCATION_ID} rdf:type Room",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:name kitchen",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:frameId map",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseSource semantic_fixture",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseX 1.20",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:poseY 0.40",
                f"{KB_MAXIMAL_CUP_ID} rdf:type Cup",
                f"{KB_MAXIMAL_CUP_ID} dbp:name KITCHEN_PROBE_CUP",
                f"{KB_MAXIMAL_CUP_ID} dbp:color white",
                f"{KB_MAXIMAL_CUP_ID} oro:isIn {KB_MAXIMAL_LOCATION_ID}",
                f"myself sees {KB_MAXIMAL_CUP_ID}",
                f"myself oro:isAt {KB_MAXIMAL_ORIGIN_ID}",
                f"nao_robot oro:isAt {KB_MAXIMAL_ORIGIN_ID}",
                f"myself canReach {KB_MAXIMAL_LOCATION_ID}",
                f"myself canReceiveAt {KB_MAXIMAL_ORIGIN_ID}",
            ),
            query_patterns=(
                f"{KB_MAXIMAL_ORIGIN_ID} ?predicate ?object",
                f"{KB_MAXIMAL_LOCATION_ID} ?predicate ?object",
                f"{KB_MAXIMAL_CUP_ID} ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "manipulation_pick_place_report",
        "composite_fake_skill_execution",
        "Pick up the apple, place it on the table, and report what happened.",
        150.0,
        setup=KbInjection(
            object_id="apple_ajrte",
            statements=(
                "apple_ajrte rdf:type Apple",
                "apple_ajrte dbp:name apple",
                "apple_ajrte dbp:color red",
                "apple_ajrte oro:isOn table_zmrkd",
                "table_zmrkd rdf:type Table",
                "table_zmrkd dbp:name table",
                "myself sees apple_ajrte",
                "myself sees table_zmrkd",
                "myself canReach apple_ajrte",
                "myself canReach table_zmrkd",
            ),
            query_patterns=(
                "apple_ajrte ?predicate ?object",
                "table_zmrkd ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        expected_outcome="execute_no_clarification",
        all_required_context=True,
    ),
    ProbeCase(
        "manipulation_multi_bring_report",
        "composite_fake_skill_execution",
        "Bring the apple and the book to Alex, then report what you did.",
        180.0,
        setup=KbInjection(
            object_id="apple_ajrte",
            statements=(
                "apple_ajrte rdf:type Apple",
                "apple_ajrte dbp:name apple",
                "apple_ajrte dbp:color red",
                "apple_ajrte oro:isOn table_zmrkd",
                "book_vpqlm rdf:type Book",
                "book_vpqlm dbp:name book",
                "book_vpqlm dbp:color blue",
                "book_vpqlm oro:isOn table_zmrkd",
                "table_zmrkd rdf:type Table",
                "table_zmrkd dbp:name table",
                "person_axlre rdf:type Human",
                "person_axlre dbp:name Alex",
                "myself sees apple_ajrte",
                "myself sees book_vpqlm",
                "myself sees table_zmrkd",
                "myself sees person_axlre",
                "myself canReach apple_ajrte",
                "myself canReach book_vpqlm",
            ),
            query_patterns=(
                "apple_ajrte ?predicate ?object",
                "book_vpqlm ?predicate ?object",
                "person_axlre ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "multi_turn_gold_apple_scene",
        "multi_turn_kb_manipulation",
        "What can you see now?",
        12.0,
        setup=KbInjection(
            object_id="codex_gold_apple_scene",
            statements=(
                "myself sees codex_gold_apple",
                "codex_gold_apple rdf:type Apple",
                "codex_gold_apple dbp:name KAREN",
                "codex_gold_apple dbp:color gold",
                "codex_gold_apple oro:isOn codex_gold_table",
                "codex_gold_table rdf:type Table",
                "codex_gold_table dbp:name table",
                "myself sees codex_gold_recipient",
                "codex_gold_recipient rdf:type Human",
                "codex_gold_recipient dbp:name ALEX",
                "codex_gold_recipient dbp:frameId codex_gold_recipient",
                "myself canReach codex_gold_apple",
            ),
            query_patterns=(
                "codex_gold_apple ?predicate ?object",
                "codex_gold_recipient ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
        conversation_group="multi_turn_gold_apple",
    ),
    ProbeCase(
        "multi_turn_gold_apple_bring",
        "multi_turn_kb_manipulation",
        "Can you bring that apple to the person named ALEX?",
        160.0,
        conversation_group="multi_turn_gold_apple",
    ),
    ProbeCase(
        "multi_turn_gold_apple_location_followup",
        "multi_turn_kb_manipulation",
        "Where is the apple now?",
        12.0,
        conversation_group="multi_turn_gold_apple",
    ),
    ProbeCase(
        "replan_absent_then_scan_report",
        "replan_recovery",
        "Find the codex missing cup. If you cannot find it, scan the scene and report what you can confirm.",
        150.0,
    ),
    ProbeCase(
        "replan_kitchen_cup_fallback_report",
        "replan_recovery",
        "Bring me the kitchen cup at the robot station. If you cannot bring it, look at the kitchen cup and report the reason.",
        150.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_ORIGIN_ID} rdf:type Place",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:name robot_station",
                f"{KB_MAXIMAL_LOCATION_ID} rdf:type Room",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:name kitchen",
                f"{KB_MAXIMAL_CUP_ID} rdf:type Cup",
                f"{KB_MAXIMAL_CUP_ID} dbp:name KITCHEN_PROBE_CUP",
                f"{KB_MAXIMAL_CUP_ID} dbp:color white",
                f"{KB_MAXIMAL_CUP_ID} oro:isIn {KB_MAXIMAL_LOCATION_ID}",
                f"myself sees {KB_MAXIMAL_CUP_ID}",
                f"myself oro:isAt {KB_MAXIMAL_ORIGIN_ID}",
                f"myself canReceiveAt {KB_MAXIMAL_ORIGIN_ID}",
            ),
            query_patterns=(
                f"{KB_MAXIMAL_ORIGIN_ID} ?predicate ?object",
                f"{KB_MAXIMAL_CUP_ID} ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "maximal_kitchen_cup_to_person",
        "maximal_semantic_execution",
        "There is a cup in the kitchen. Bring the kitchen cup to the person named ALEX and report what happened.",
        160.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_LOCATION_ID} rdf:type Room",
                f"{KB_MAXIMAL_LOCATION_ID} dbp:name kitchen",
                f"{KB_MAXIMAL_CUP_ID} rdf:type Cup",
                f"{KB_MAXIMAL_CUP_ID} dbp:name KITCHEN_PROBE_CUP",
                f"{KB_MAXIMAL_CUP_ID} dbp:color white",
                f"{KB_MAXIMAL_CUP_ID} oro:isIn {KB_MAXIMAL_LOCATION_ID}",
                f"myself sees {KB_MAXIMAL_CUP_ID}",
                f"{KB_MAXIMAL_PERSON_ID} rdf:type Human",
                f"{KB_MAXIMAL_PERSON_ID} dbp:name ALEX",
                f"{KB_MAXIMAL_PERSON_ID} dbp:frameId {KB_MAXIMAL_PERSON_ID}",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseSource semantic_fixture",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseX 0.30",
                f"{KB_MAXIMAL_PERSON_ID} dbp:poseY -0.20",
                f"myself sees {KB_MAXIMAL_PERSON_ID}",
            ),
            query_patterns=(
                f"{KB_MAXIMAL_LOCATION_ID} ?predicate ?object",
                f"{KB_MAXIMAL_CUP_ID} ?predicate ?object",
                f"{KB_MAXIMAL_PERSON_ID} ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "future_action_admission_holdout",
        "route_prompt_ambiguity",
        "Could we navigate to the probe cup later?",
        12.0,
    ),
    ProbeCase(
        "reflective_followup",
        "simple_dialogue",
        "How many directions did you move your head?",
        10.0,
        conversation_group="head_wave_reflection",
    ),
)

INTENT_ABLATION_CASES = (
    ProbeCase(
        "intent_ablation_dialogue_greeting",
        "intent_route_ablation_dialogue",
        "Hey, how are you?",
        30.0,
        conversation_group="intent_ablation_dialogue",
    ),
    ProbeCase(
        "intent_ablation_favorite_movie",
        "intent_route_ablation_dialogue",
        "What is your favourite movie?",
        45.0,
        conversation_group="intent_ablation_dialogue",
    ),
    ProbeCase(
        "intent_ablation_wave_particle",
        "intent_route_ablation_dialogue",
        "What is wave-particle duality?",
        45.0,
    ),
    ProbeCase(
        "intent_ablation_future_navigation",
        "intent_route_ablation_holdout",
        "Could we navigate to the probe cup later?",
        45.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn codex_probe_table",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "intent_ablation_kb_visible",
        "intent_route_ablation_kb_query",
        "What can you see now?",
        45.0,
        setup=KbInjection(
            object_id="codex_ablation_scene",
            statements=(
                "myself sees codex_ablation_cup",
                "codex_ablation_cup rdf:type Cup",
                "codex_ablation_cup dbp:name TITAS",
                "codex_ablation_cup dbp:color gold",
                "myself sees codex_ablation_person",
                "codex_ablation_person rdf:type Human",
                "codex_ablation_person dbp:name ALEX",
            ),
            query_patterns=(
                "codex_ablation_cup ?predicate ?object",
                "codex_ablation_person ?predicate ?object",
            ),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "intent_ablation_look_report",
        "intent_route_ablation_execution",
        "Look at the probe cup and tell me what you did.",
        100.0,
        setup=KbInjection(
            object_id=KB_PROBE_OBJECT_ID,
            statements=(
                f"myself sees {KB_PROBE_OBJECT_ID}",
                f"{KB_PROBE_OBJECT_ID} rdf:type Cup",
                f"{KB_PROBE_OBJECT_ID} dbp:name TITAS",
                f"{KB_PROBE_OBJECT_ID} dbp:color gold",
                f"{KB_PROBE_OBJECT_ID} oro:isOn codex_probe_table",
            ),
            query_patterns=(f"{KB_PROBE_OBJECT_ID} ?predicate ?object",),
            query_vars=("?predicate", "?object"),
        ),
    ),
    ProbeCase(
        "intent_ablation_head_wave",
        "intent_route_ablation_execution",
        "Move your head up and then wave at me.",
        95.0,
    ),
)

POSTURE_ABLATION_CASES = (
    ProbeCase(
        "posture_stand_report",
        "posture_ablation",
        "Stand up and tell me when you are standing.",
        90.0,
        conversation_group="posture_stand",
        expected_outcome="execute_no_clarification",
        expected_report_policy="final",
    ),
    ProbeCase(
        "posture_sit_report",
        "posture_ablation",
        "Sit down and tell me when you are sitting.",
        90.0,
        conversation_group="posture_sit",
        expected_outcome="execute_no_clarification",
        expected_report_policy="final",
    ),
    ProbeCase(
        "posture_kneel_report",
        "posture_ablation",
        "Kneel down and tell me when you are kneeling.",
        90.0,
        conversation_group="posture_kneel",
        expected_outcome="execute_no_clarification",
        expected_report_policy="final",
    ),
    ProbeCase(
        "posture_sequence_final_state",
        "posture_ablation",
        "Sit down, then stand up again, and tell me your final posture.",
        120.0,
        conversation_group="posture_sequence",
        expected_outcome="execute_no_clarification",
        expected_report_policy="final",
    ),
)

ENVIRONMENT_CASES = (
    ProbeCase(
        "environment_baseline_table_inventory",
        "preloaded_environment_kb_query",
        "What objects are on the table?",
        12.0,
        environment_ids=("baseline_table",),
        conversation_group="preloaded_baseline_table",
    ),
    ProbeCase(
        "environment_lab_table_inventory",
        "preloaded_environment_kb_query",
        "What objects are on the table?",
        12.0,
        environment_ids=("lab_table",),
        conversation_group="preloaded_lab_table",
    ),
    ProbeCase(
        "environment_lab_sections_inventory",
        "preloaded_environment_kb_query",
        "What can you see in the lab?",
        14.0,
        environment_ids=("lab_sections",),
        conversation_group="preloaded_lab_sections",
    ),
    ProbeCase(
        "environment_lab_sections_delivery",
        "preloaded_environment_composite_execution",
        "Bring every object from the work table to ALEX and report what happened.",
        180.0,
        environment_ids=("lab_sections",),
        conversation_group="preloaded_lab_sections",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_lab_cup", "codex_lab_manual", "codex_lab_phone"),
        expected_recipient_id="codex_lab_alex",
        expected_report_policy="final",
    ),
    ProbeCase(
        "environment_kitchen_inventory",
        "preloaded_environment_kb_query",
        "What is in the kitchen?",
        12.0,
        environment_ids=("kitchen_delivery",),
        conversation_group="preloaded_kitchen_delivery",
    ),
    ProbeCase(
        "environment_grouped_location_delivery",
        "preloaded_environment_composite_execution",
        "Bring every object from the kitchen to ALEX and report what happened.",
        180.0,
        environment_ids=("kitchen_delivery",),
        conversation_group="preloaded_kitchen_delivery",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_kitchen_cup", "codex_kitchen_book"),
        expected_recipient_id="codex_recipient_person",
        expected_report_policy="final",
    ),
    ProbeCase(
        "environment_grouped_location_followup",
        "preloaded_environment_kb_query",
        "Where is the kitchen cup now?",
        14.0,
        environment_ids=("kitchen_delivery",),
        conversation_group="preloaded_kitchen_delivery",
    ),
    ProbeCase(
        "environment_iiia_floor_inventory",
        "preloaded_environment_kb_query",
        "What rooms and objects are in the IIIA floor scene?",
        16.0,
        environment_ids=("iiia_floor",),
        conversation_group="preloaded_iiia_floor",
    ),
    ProbeCase(
        "environment_iiia_kitchen_delivery",
        "preloaded_environment_composite_execution",
        "Bring every object from the kitchen to ALEX and report what happened.",
        180.0,
        environment_ids=("iiia_floor",),
        conversation_group="preloaded_iiia_floor",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_iiia_cup",),
        expected_recipient_id="codex_iiia_alex",
        expected_report_policy="final",
    ),
    ProbeCase(
        "environment_gold_apple_handoff",
        "preloaded_environment_multi_turn",
        "Can you bring that gold apple to the person named ALEX?",
        160.0,
        environment_ids=("gold_apple_handoff",),
        conversation_group="preloaded_gold_apple",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_gold_apple",),
        expected_recipient_id="codex_gold_recipient",
    ),
    ProbeCase(
        "environment_gold_apple_followup",
        "preloaded_environment_kb_query",
        "Where is the gold apple now?",
        14.0,
        environment_ids=("gold_apple_handoff",),
        conversation_group="preloaded_gold_apple",
    ),
)

FAKE_DEEP_CASES = (
    ProbeCase(
        "fake_deep_baseline_inventory",
        "fake_deep_preflight",
        "What objects are on the table?",
        12.0,
        environment_ids=("baseline_table",),
        conversation_group="fake_deep_baseline",
    ),
    ProbeCase(
        "fake_deep_ordered_walk_report",
        "fake_deep_composite_success",
        "Walk to every object on the table and let me know when you get to each one.",
        180.0,
        environment_ids=("lab_table",),
        conversation_group="fake_deep_lab_table",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        requires_replan=True,
        expected_member_ids=(
            "codex_lab_apple",
            "codex_lab_book",
            "codex_lab_phone",
            "codex_probe_cup",
        ),
        expected_report_policy="per_target",
    ),
    ProbeCase(
        "fake_deep_grouped_work_table_delivery",
        "fake_deep_location_delivery",
        "Bring every object from the work table to ALEX and report what happened.",
        180.0,
        environment_ids=("lab_sections",),
        conversation_group="fake_deep_lab_sections",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_lab_cup", "codex_lab_manual", "codex_lab_phone"),
        expected_recipient_id="codex_lab_alex",
        expected_report_policy="final",
    ),
    ProbeCase(
        "fake_deep_missing_object_recovery",
        "fake_deep_absent_target_replan",
        "Find the codex missing cup. If you cannot find it, scan the scene and report what you can confirm.",
        180.0,
        environment_ids=("baseline_table",),
        conversation_group="fake_deep_absent_target",
        expected_outcome="recover_or_truthful_failure",
        absence_guards=(
            KbAbsenceGuard(
                "missing_cup_subject_absent",
                ("codex_missing_cup ?predicate ?object",),
            ),
        ),
        fake_mode_overrides=(("find_object", "always_fail"),),
    ),
    ProbeCase(
        "fake_deep_missing_recipient_clarification",
        "fake_deep_missing_secondary_target",
        "Bring every object from the work table to the person named BLAKE and report what happened.",
        150.0,
        environment_ids=("lab_sections",),
        conversation_group="fake_deep_missing_recipient",
        expected_outcome="clarification_expected",
        absence_guards=(
            KbAbsenceGuard(
                "blake_subject_absent",
                ("codex_lab_blake ?predicate ?object",),
            ),
            KbAbsenceGuard(
                "blake_named_person_absent",
                ("?subject dbp:name BLAKE",),
                ("?subject",),
            ),
        ),
    ),
    ProbeCase(
        "fake_deep_iiia_kitchen_delivery",
        "fake_deep_maximal_location_delivery",
        "Bring every object from the kitchen to ALEX and report what happened.",
        220.0,
        environment_ids=("iiia_floor",),
        conversation_group="fake_deep_iiia_floor",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_iiia_cup",),
        expected_recipient_id="codex_iiia_alex",
        expected_report_policy="final",
    ),
    ProbeCase(
        "fake_deep_iiia_floor_location_followup",
        "fake_deep_post_effect_query",
        "Where are the objects from the kitchen now?",
        20.0,
        conversation_group="fake_deep_iiia_floor",
    ),
    ProbeCase(
        "fake_deep_gold_apple_multiturn",
        "fake_deep_multiturn_pronoun_handoff",
        "Can you bring that gold apple to the person named ALEX?",
        180.0,
        environment_ids=("gold_apple_handoff",),
        conversation_group="fake_deep_gold_apple",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_gold_apple",),
        expected_recipient_id="codex_gold_recipient",
    ),
    ProbeCase(
        "fake_deep_gold_apple_followup",
        "fake_deep_post_effect_query",
        "Where is the gold apple now?",
        16.0,
        environment_ids=("gold_apple_handoff",),
        conversation_group="fake_deep_gold_apple",
    ),
)

ROBUSTNESS_CASES = (
    ProbeCase(
        "robust_paraphrased_group_delivery",
        "robust_group_selection",
        "Take everything resting on the work table over to ALEX. Once every item has reached them, summarize the whole delivery.",
        180.0,
        environment_ids=("lab_sections",),
        conversation_group="robust_group_delivery",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_lab_cup", "codex_lab_manual", "codex_lab_phone"),
        expected_recipient_id="codex_lab_alex",
        expected_report_policy="final",
    ),
    ProbeCase(
        "robust_object_role_exclusion",
        "robust_role_integrity",
        "Visit each movable item on the table in turn. Do not navigate to the table or to any person, and report each arrival.",
        180.0,
        environment_ids=("lab_table",),
        conversation_group="robust_role_exclusion",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=(
            "codex_lab_apple",
            "codex_lab_book",
            "codex_lab_phone",
            "codex_probe_cup",
        ),
        expected_report_policy="per_target",
    ),
    ProbeCase(
        "robust_missing_recipient",
        "robust_clarification",
        "Deliver every work-table object to MORGAN and explain what information is missing if MORGAN is not present.",
        90.0,
        environment_ids=("lab_sections",),
        conversation_group="robust_missing_recipient",
        expected_outcome="clarification_expected",
        absence_guards=(
            KbAbsenceGuard(
                "morgan_named_person_absent",
                ("?subject dbp:name MORGAN",),
                ("?subject",),
            ),
        ),
    ),
    ProbeCase(
        "robust_future_recipient_memory",
        "robust_route_safety",
        "ALEX will receive the work-table items later. For now, only acknowledge this and do not execute anything.",
        12.0,
        environment_ids=("lab_sections",),
        conversation_group="robust_multiturn",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "robust_multiturn_execute_followup",
        "robust_multiturn_grounding",
        "Now deliver all of those objects to them and tell me the overall result.",
        180.0,
        conversation_group="robust_multiturn",
        expected_outcome="execute_no_clarification",
        all_required_context=True,
        requires_target_selection=True,
        expected_member_ids=("codex_lab_cup", "codex_lab_manual", "codex_lab_phone"),
        expected_recipient_id="codex_lab_alex",
        expected_report_policy="final",
    ),
)

ROUTE_ACK_CASES = (
    ProbeCase(
        "route_ack_capability_question",
        "route_ack_pair",
        "What can you do with your head?",
        12.0,
        conversation_group="route_ack_capability",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "route_ack_polite_execution",
        "route_ack_pair",
        "Can you move your head in all directions?",
        95.0,
        conversation_group="route_ack_polite_execution",
        expected_outcome="execute_no_clarification",
    ),
    ProbeCase(
        "route_ack_polite_followup",
        "route_ack_pair",
        "How many directions did you move your head?",
        12.0,
        conversation_group="route_ack_polite_execution",
        expected_outcome="dialogue_only",
    ),
    ProbeCase(
        "route_ack_explicit_execution",
        "route_ack_pair",
        "Please move your head in all directions now.",
        95.0,
        conversation_group="route_ack_explicit_execution",
        expected_outcome="execute_no_clarification",
    ),
)

TOPICS_TO_SAMPLE = (
    "/chatbot_llm/turn_trace",
    "/planner/request",
    "/planner/execution_feedback",
    "/nao_orchestrator/planner_dialogue_act",
    "/debug/nao_say/speech",
    "/speech",
)

FAKE_POLICY_PROFILES = {
    "none": {},
    "all_success": {"global_mode": "always_success", "mode_overrides_json": "{}"},
    "every_other": {"global_mode": "every_other", "mode_overrides_json": "{}"},
    "random_seeded": {
        "global_mode": "random_seeded",
        "random_failure_prob": "0.50",
        "mode_overrides_json": "{}",
    },
    "fail_once_navigation": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"navigate_to":"fail_once"}',
    },
    "fail_once_pick": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"pick_object":"fail_once"}',
    },
    "delivery_blocked": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"bring_object":"delivery_blocked"}',
    },
    "fail_once_place": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"place_object":"fail_once"}',
    },
    "destination_unavailable": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"place_object":"destination_unavailable"}',
    },
    "recipient_missing": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"bring_object":"recipient_unavailable"}',
    },
}

FAKE_POLICY_EXPECTED_SKILLS = {
    "fail_once_navigation": "navigate_to",
    "fail_once_pick": "pick_object",
    "delivery_blocked": "bring_object",
    "fail_once_place": "place_object",
    "destination_unavailable": "place_object",
    "recipient_missing": "bring_object",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default=DEFAULT_CONTAINER)
    parser.add_argument(
        "--case-set",
        default="smoke",
        choices=(
            "smoke",
            "main",
            "composite",
            "intent_ablation",
            "posture_ablation",
            "environment",
            "kb_stress",
            "grounding_people",
            "capability_extreme",
            "fake_deep",
            "robustness",
            "route_ack",
        ),
    )
    parser.add_argument(
        "--case-names",
        default="",
        help="Comma-separated case names to run from the selected case set.",
    )
    parser.add_argument(
        "--categories",
        default="",
        help="Comma-separated categories to run from the selected case set.",
    )
    parser.add_argument("--out", default="/tmp/nao_active_questionnaire.json")
    parser.add_argument("--since-sec", type=int, default=90)
    parser.add_argument("--global-timeout-sec", type=int, default=DEFAULT_GLOBAL_TIMEOUT_SEC)
    parser.add_argument(
        "--max-case-wait-sec",
        type=float,
        default=0.0,
        help=(
            "Optional cap for per-case post-turn waits. Leave at 0 for the "
            "formal questionnaire waits; set during interactive deep-fake "
            "debugging to avoid waiting several minutes after evidence already arrived."
        ),
    )
    parser.add_argument(
        "--kb-lifespan-sec",
        type=int,
        default=DEFAULT_KB_LIFESPAN_SEC,
        help="Lifespan for KB facts injected by questionnaire setup cases.",
    )
    parser.add_argument(
        "--sample-topics",
        action="store_true",
        help="Collect one-shot topic samples after each case. Slower, but useful for E2E speech runs.",
    )
    parser.add_argument(
        "--mode",
        default="speech",
        choices=("speech", "chatbot_service"),
        help="Turn injection seam. Speech is full ROS4HRI E2E; service is chatbot-only.",
    )
    parser.add_argument(
        "--speech-voice-scope",
        default="shared",
        choices=("shared", "group", "case"),
        help=(
            "Voice id policy for speech-mode runs. Use shared for long-context "
            "stress, group/case for isolated scored evidence."
        ),
    )
    parser.add_argument(
        "--no-rqt-display-mirror",
        action="store_true",
        help="Do not mirror injected user turns to rqt-visible debug topics.",
    )
    parser.add_argument(
        "--expected-turn-pipeline-mode",
        default="",
        choices=("", "response_first", "intent_first"),
        help="Optional assertion label for the active /chatbot_llm turn_pipeline_mode.",
    )
    parser.add_argument(
        "--environment-fixtures",
        default=str(DEFAULT_ENVIRONMENT_FIXTURE_PATH),
        help="JSON file containing named KnowledgeCore environment fixtures.",
    )
    parser.add_argument(
        "--preload-environment",
        default="",
        help=(
            "Comma-separated environment fixture ids to inject once before the "
            "case set. Use --case-set environment for the built-in environment "
            "validation ladder."
        ),
    )
    parser.add_argument(
        "--list-environments",
        action="store_true",
        help="List available environment fixture ids and exit.",
    )
    parser.add_argument(
        "--fake-policy-profile",
        default="none",
        choices=tuple(sorted(FAKE_POLICY_PROFILES.keys())),
        help=(
            "Optional fake_skill_server policy profile applied before the run. "
            "Use this for deterministic success/failure/replan validation."
        ),
    )
    args = parser.parse_args()

    environment_fixture_path = resolve_environment_fixture_path(Path(args.environment_fixtures))
    environment_fixtures = load_environment_fixtures(environment_fixture_path)
    if args.list_environments:
        for fixture_id in sorted(environment_fixtures.keys()):
            description = str(environment_fixtures[fixture_id].get("description", "")).strip()
            suffix = " - %s" % description if description else ""
            print("%s%s" % (fixture_id, suffix))
        return 0

    case_sets = {
        "smoke": SMOKE_CASES,
        "main": MAIN_QUESTIONNAIRE_CASES,
        "composite": COMPOSITE_CASES,
        "intent_ablation": INTENT_ABLATION_CASES,
        "posture_ablation": POSTURE_ABLATION_CASES,
        "environment": ENVIRONMENT_CASES,
        "kb_stress": KB_STRESS_CASES,
        "grounding_people": GROUNDING_PEOPLE_CASES,
        "capability_extreme": CAPABILITY_EXTREME_CASES,
        "fake_deep": FAKE_DEEP_CASES,
        "robustness": ROBUSTNESS_CASES,
        "route_ack": ROUTE_ACK_CASES,
    }
    cases = list(case_sets[args.case_set])
    cases = filter_cases(
        cases,
        case_names=parse_csv(args.case_names),
        categories=parse_csv(args.categories),
    )
    results = []
    service_histories: dict[str, list[dict[str, str]]] = {}
    started_at = time.time()
    runtime_metadata = collect_questionnaire_metadata(
        args.container,
        expected_turn_pipeline_mode=args.expected_turn_pipeline_mode,
    )
    trace_preflight = collect_structured_trace_events(args.container, time.time() - 2.0)
    runtime_metadata["structured_trace_preflight"] = {
        key: value for key, value in trace_preflight.items() if key != "events"
    }
    if not trace_preflight.get("available"):
        results.append(
            {
                "name": "structured_trace_preflight",
                "category": "observability",
                "text": "",
                "mode": args.mode,
                "status": "not_scored",
                "turn_result": "interaction trace JSONL is unavailable",
                "started_at_unix_sec": time.time(),
                "wait_sec": 0.0,
                "phase_observations": {
                    "evidence_source": "interaction_trace_jsonl",
                    "correlation_status": "unavailable",
                    "evidence_consistent": False,
                    "evidence_inconsistencies": [
                        "scored runs require interaction trace JSONL"
                    ],
                },
            }
        )
        write_payload(
            args.out,
            args.container,
            args.case_set,
            started_at,
            results,
            runtime_metadata=runtime_metadata,
            run_status="completed",
        )
        return 2
    runtime_metadata["environment_fixture_source"] = str(environment_fixture_path)
    runtime_metadata["available_environment_fixtures"] = sorted(environment_fixtures.keys())
    all_fixture_ids = tuple(sorted(environment_fixtures.keys()))
    all_case_injections = tuple(
        case.setup
        for case_set in case_sets.values()
        for case in case_set
        if case.setup is not None
    )
    runtime_metadata["initial_fixture_cleanup"] = _merge_cleanup_results(
        retract_kb_injections(args.container, all_case_injections),
        retract_environment_fixtures(
            args.container,
            all_fixture_ids,
            environment_fixtures=environment_fixtures,
        ),
    )
    runtime_metadata["fake_policy_profile"] = args.fake_policy_profile
    runtime_metadata["fake_policy_application"] = apply_fake_policy_profile(
        args.container,
        args.fake_policy_profile,
    )
    runtime_metadata["tracked_voice_cleanup_before_run"] = cleanup_tracked_voice_publishers(
        args.container
    )
    preloaded_environment_ids = parse_csv_list(args.preload_environment)
    preloaded_environments = inject_environment_fixtures(
        args.container,
        preloaded_environment_ids,
        environment_fixtures=environment_fixtures,
        lifespan_sec=args.kb_lifespan_sec,
    )
    if preloaded_environments:
        runtime_metadata["preloaded_environments"] = preloaded_environments
    active_fixture_ids: tuple[str, ...] = ()
    active_fixture_group = ""
    active_case_injections: list[KbInjection] = []
    active_case_group = ""
    for index, case in enumerate(cases, start=1):
        if time.time() - started_at > max(30, args.global_timeout_sec):
            runtime_metadata["final_fixture_cleanup"] = _merge_cleanup_results(
                retract_kb_injections(args.container, tuple(active_case_injections)),
                retract_environment_fixtures(
                    args.container,
                    all_fixture_ids,
                    environment_fixtures=environment_fixtures,
                ),
            )
            runtime_metadata["fake_policy_restore"] = restore_fake_policy_profile(
                args.container
            )
            runtime_metadata["tracked_voice_cleanup"] = cleanup_tracked_voice_publishers(
                args.container
            )
            results.append(
                {
                    "name": "global_timeout",
                    "category": "observability",
                    "text": "",
                    "mode": args.mode,
                    "turn_result": "global timeout reached before remaining cases",
                    "started_at_unix_sec": time.time(),
                    "wait_sec": 0.0,
                    "injection_scope": args.mode,
                    "topic_samples": {},
                    "log_excerpt": recent_logs(args.container, args.since_sec),
                }
            )
            write_payload(
                args.out,
                args.container,
                args.case_set,
                started_at,
                results,
                runtime_metadata=runtime_metadata,
                run_status="timed_out",
            )
            return 2

        case_start = time.time()
        setup_result = {}
        case_group = case.conversation_group or case.name
        cleanup_guard = {}
        case_policy_overrides = {}
        if active_case_injections and case_group != active_case_group:
            cleanup_guard = retract_kb_injections(
                args.container,
                tuple(active_case_injections),
            )
            active_case_injections = []
        if active_fixture_ids and case_group != active_fixture_group:
            environment_cleanup = retract_environment_fixtures(
                args.container,
                active_fixture_ids,
                environment_fixtures=environment_fixtures,
            )
            cleanup_guard = _merge_cleanup_results(cleanup_guard, environment_cleanup)
            active_fixture_ids = ()
        requested_fixture_ids = tuple(case.environment_ids)
        if requested_fixture_ids and requested_fixture_ids != active_fixture_ids:
            setup_result["environment"] = inject_environment_fixtures(
                args.container,
                list(requested_fixture_ids),
                environment_fixtures=environment_fixtures,
                lifespan_sec=args.kb_lifespan_sec,
            )
            active_fixture_ids = requested_fixture_ids
            active_fixture_group = case_group
        elif requested_fixture_ids:
            setup_result["environment"] = {
                "retained": list(requested_fixture_ids),
                "reason": "same conversation group preserves post-effect world state",
            }
        if case.setup is not None:
            setup_result["case"] = inject_kb_probe(
                args.container,
                case.setup,
                lifespan_sec=args.kb_lifespan_sec,
            )
            active_case_injections.append(case.setup)
            active_case_group = case_group
        stale_world_guard = run_absence_guards(args.container, case.absence_guards)
        if cleanup_guard.get("contaminated"):
            stale_world_guard = {
                "contaminated": True,
                "fixture_cleanup": cleanup_guard,
            }
        if stale_world_guard.get("contaminated"):
            result_entry = {
                "name": case.name,
                "category": case.category,
                "text": case.text,
                "voice_id": "",
                "mode": args.mode or case.mode,
                "setup_result": setup_result or None,
                "stale_world_guard": stale_world_guard,
                "turn_result": (
                    "skipped: stale KnowledgeCore facts matched an absent-target "
                    "preflight guard; relaunch or clear the fixture namespace"
                ),
                "started_at_unix_sec": case_start,
                "wait_sec": 0.0,
                "configured_wait_sec": case.wait_sec,
                "injection_scope": injection_scope(args.mode or case.mode),
                "topic_samples": {},
                "log_excerpt": "",
                "phase_observations": {
                    "turn_injected": False,
                    "route_observed": False,
                    "planner_request_observed": False,
                    "execution_feedback_observed": False,
                    "speech_observed": False,
                    "clarification_observed": False,
                    "observability_note": (
                        "case skipped before turn injection because fixture "
                        "isolation was contaminated"
                    ),
                },
            }
            result_entry["case_assessment"] = assess_case(
                case,
                observations=result_entry["phase_observations"],
                stale_world_guard=stale_world_guard,
                fake_policy_profile=args.fake_policy_profile,
            )
            result_entry["status"] = result_entry["case_assessment"]["status"]
            results.append(result_entry)
            write_payload(
                args.out,
                args.container,
                args.case_set,
                started_at,
                results,
                runtime_metadata=runtime_metadata,
            )
            continue

        case_policy_overrides = apply_case_fake_mode_overrides(
            args.container,
            args.fake_policy_profile,
            case.fake_mode_overrides,
        )
        if case_policy_overrides:
            setup_result["fake_policy_case"] = {
                "overrides": dict(case.fake_mode_overrides),
                "application": case_policy_overrides,
            }
        case_assessment_policy = _effective_fake_policy_profile(
            args.fake_policy_profile,
            case_policy_overrides,
        )
        mode = args.mode or case.mode
        conversation_group = case.conversation_group or case.name or f"case_{index}"
        voice_id = _voice_id_for_case(
            mode,
            group=conversation_group,
            case_name=case.name,
            index=index,
            speech_voice_scope=args.speech_voice_scope,
        )
        turn_start = time.time()
        if mode == "speech":
            turn_result = publish_voice_turn(
                args.container,
                case.text,
                voice_id=voice_id,
                mirror_rqt_display=not args.no_rqt_display_mirror,
            )
        else:
            history = service_histories.setdefault(conversation_group, [])
            turn_result = call_chatbot_turn(
                args.container,
                case.text,
                len(history) + 1,
                voice_id=voice_id,
                history=history,
            )
            history.append({"speaker": voice_id, "text": case.text})
            response_text = extract_service_response(turn_result)
            if response_text:
                history.append({"speaker": "__assistant__", "text": response_text})
        result_entry = {
            "name": case.name,
            "category": case.category,
            "text": case.text,
            "voice_id": voice_id,
            "mode": mode,
            "setup_result": setup_result or None,
            "turn_result": turn_result,
            "stale_world_guard": stale_world_guard or None,
            "started_at_unix_sec": case_start,
            "turn_started_at_unix_sec": turn_start,
            "wait_sec": 0.0,
            "configured_wait_sec": case.wait_sec,
            "injection_scope": injection_scope(mode),
            "topic_samples": {},
            "log_excerpt": "",
            "phase_observations": phase_observations(
                mode=mode,
                turn_result=turn_result,
                log_excerpt="",
                topic_samples={},
                voice_id=voice_id,
            ),
        }
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=stale_world_guard,
            fake_policy_profile=case_assessment_policy,
        )
        result_entry["status"] = result_entry["case_assessment"]["status"]
        results.append(result_entry)
        write_payload(
            args.out,
            args.container,
            args.case_set,
            started_at,
            results,
            runtime_metadata=runtime_metadata,
        )
        wait_sec = max(0.0, case.wait_sec)
        if args.max_case_wait_sec > 0:
            wait_sec = min(wait_sec, max(0.0, args.max_case_wait_sec))
        _observe_case_during_wait(
            args.out,
            args.container,
            args.case_set,
            started_at,
            results,
            result_entry,
            case=case,
            mode=mode,
            turn_result=turn_result,
            case_start=turn_start,
            wait_sec=wait_sec,
            runtime_metadata=runtime_metadata,
            fake_policy_profile=case_assessment_policy,
        )
        topic_samples = sample_topics(args.container) if args.sample_topics else {}
        log_excerpt = recent_logs_since(args.container, turn_start)
        structured_trace = collect_structured_trace_events(args.container, turn_start)
        result_entry["wait_sec"] = wait_sec
        result_entry["topic_samples"] = topic_samples
        result_entry["log_excerpt"] = log_excerpt
        result_entry["structured_trace"] = {
            key: value for key, value in structured_trace.items() if key != "events"
        }
        result_entry["phase_observations"] = structured_phase_observations(
            mode=mode,
            turn_result=turn_result,
            events=structured_trace.get("events", []),
            turn_started_at=turn_start,
        )
        if case.postcondition is not None:
            postcondition_result = evaluate_kb_postcondition(
                args.container,
                case.postcondition,
            )
            result_entry["postcondition_result"] = postcondition_result
            result_entry["phase_observations"]["kb_postcondition_passed"] = bool(
                postcondition_result.get("passed")
            )
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=stale_world_guard,
            fake_policy_profile=case_assessment_policy,
        )
        result_entry["status"] = result_entry["case_assessment"]["status"]
        write_payload(
            args.out,
            args.container,
            args.case_set,
            started_at,
            results,
            runtime_metadata=runtime_metadata,
        )
        if case_policy_overrides:
            runtime_metadata.setdefault("case_policy_restores", []).append(
                {
                    "case": case.name,
                    "restore": apply_fake_policy_profile(
                        args.container,
                        args.fake_policy_profile,
                    ),
                }
            )

    final_case_cleanup = retract_kb_injections(
        args.container,
        tuple(active_case_injections),
    )
    final_environment_cleanup = retract_environment_fixtures(
        args.container,
        all_fixture_ids,
        environment_fixtures=environment_fixtures,
    )
    runtime_metadata["final_fixture_cleanup"] = _merge_cleanup_results(
        final_case_cleanup,
        final_environment_cleanup,
    )
    runtime_metadata["fake_policy_restore"] = restore_fake_policy_profile(args.container)
    runtime_metadata["tracked_voice_cleanup"] = cleanup_tracked_voice_publishers(
        args.container
    )
    write_payload(
        args.out,
        args.container,
        args.case_set,
        started_at,
        results,
        runtime_metadata=runtime_metadata,
        run_status="completed",
    )
    print(args.out)
    return 0


def parse_csv(value: str) -> set[str]:
    return {item.strip() for item in str(value or "").split(",") if item.strip()}


def parse_csv_list(value: str) -> list[str]:
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def filter_cases(
    cases: list[ProbeCase],
    *,
    case_names: set[str],
    categories: set[str],
) -> list[ProbeCase]:
    if case_names:
        cases = [case for case in cases if case.name in case_names]
    if categories:
        cases = [case for case in cases if case.category in categories]
    return cases


def load_environment_fixtures(path: Path) -> dict[str, dict]:
    fixture_path = Path(path)
    if not fixture_path.exists():
        return {}
    payload = json.loads(fixture_path.read_text(encoding="utf-8"))
    environments = payload.get("environments", {}) if isinstance(payload, dict) else {}
    if not isinstance(environments, dict):
        return {}
    return {
        str(fixture_id).strip(): fixture
        for fixture_id, fixture in environments.items()
        if str(fixture_id).strip() and isinstance(fixture, dict)
    }


def resolve_environment_fixture_path(path: Path) -> Path:
    if path.exists():
        return path
    if FALLBACK_ENVIRONMENT_FIXTURE_PATH.exists():
        return FALLBACK_ENVIRONMENT_FIXTURE_PATH
    return path


def apply_fake_policy_profile(container: str, profile: str) -> dict[str, str]:
    clean_profile = str(profile or "none").strip()
    settings = FAKE_POLICY_PROFILES.get(clean_profile, {})
    if not settings:
        return {"profile": clean_profile, "applied": "false", "reason": "no policy changes requested"}

    outputs = {"profile": clean_profile, "applied": "true"}
    for param_name, value in settings.items():
        outputs[param_name] = set_ros_param(
            container,
            "/fake_skill_server",
            param_name,
            value,
        )
    return outputs


def apply_case_fake_mode_overrides(
    container: str,
    profile: str,
    overrides: tuple[tuple[str, str], ...],
) -> dict[str, str]:
    """Apply a case-local skill override while preserving the run profile."""
    if not overrides:
        return {}
    settings = FAKE_POLICY_PROFILES.get(str(profile or "none").strip(), {})
    raw_base = settings.get("mode_overrides_json", "{}")
    try:
        merged = json.loads(raw_base)
    except (TypeError, json.JSONDecodeError):
        merged = {}
    if not isinstance(merged, dict):
        merged = {}
    for skill, mode in overrides:
        clean_skill = str(skill or "").strip().lower()
        clean_mode = str(mode or "").strip().lower()
        if clean_skill and clean_mode:
            merged[clean_skill] = clean_mode
    if not merged:
        return {}
    return {
        "mode_overrides_json": set_ros_param(
            container,
            "/fake_skill_server",
            "mode_overrides_json",
            json.dumps(merged, separators=(",", ":")),
        )
    }


def _effective_fake_policy_profile(
    profile: str,
    case_policy_overrides: dict[str, str],
) -> str:
    """Make case-local failures use the recovery observation contract."""
    clean_profile = str(profile or "none").strip() or "none"
    if case_policy_overrides and clean_profile in {"none", "all_success"}:
        return "case_local"
    return clean_profile


def restore_fake_policy_profile(container: str) -> dict[str, str]:
    return {
        "global_mode": set_ros_param(
            container,
            "/fake_skill_server",
            "global_mode",
            "scenario",
        ),
        "mode_overrides_json": set_ros_param(
            container,
            "/fake_skill_server",
            "mode_overrides_json",
            "{}",
        ),
    }


def set_ros_param(container: str, node_name: str, param_name: str, value: str) -> str:
    parameter_value = str(value)
    if isinstance(value, str):
        parameter_value = json.dumps(value)
    script = """
%s
timeout 10 ros2 param set %s %s %s 2>&1 || true
""" % (
        ROS_CLI_PREAMBLE,
        shlex.quote(node_name),
        shlex.quote(param_name),
        shlex.quote(parameter_value),
    )
    return run(
        ["docker", "exec", container, "bash", "-lc", script],
        timeout=15,
        check=False,
    )


def inject_environment_fixtures(
    container: str,
    fixture_ids: list[str],
    *,
    environment_fixtures: dict[str, dict],
    lifespan_sec: int,
) -> dict[str, dict]:
    results: dict[str, dict] = {}
    for fixture_id in fixture_ids:
        clean_id = str(fixture_id or "").strip()
        if not clean_id:
            continue
        fixture = environment_fixtures.get(clean_id)
        if not isinstance(fixture, dict):
            results[clean_id] = {
                "error": "unknown_environment_fixture",
                "available": sorted(environment_fixtures.keys()),
            }
            continue
        injection = environment_fixture_to_injection(clean_id, fixture)
        results[clean_id] = {
            "description": str(fixture.get("description", "")).strip(),
            "visual_svg_source": str(fixture.get("visual_svg_source", "")).strip(),
            "injection": inject_kb_probe(container, injection, lifespan_sec=lifespan_sec),
        }
    return results


def retract_environment_fixtures(
    container: str,
    fixture_ids: tuple[str, ...],
    *,
    environment_fixtures: dict[str, dict],
) -> dict[str, object]:
    """Retract current facts touching declared fixture subjects and verify absence."""
    injections = []
    for fixture_id in fixture_ids:
        fixture = environment_fixtures.get(fixture_id, {})
        if isinstance(fixture, dict):
            injections.append(environment_fixture_to_injection(fixture_id, fixture))
    return retract_kb_injections(container, tuple(injections))


def retract_kb_injections(
    container: str,
    injections: tuple[KbInjection, ...],
) -> dict[str, object]:
    """Retract facts touching case-owned subjects and verify their absence."""
    subjects = []
    declared_statements = []
    for injection in injections:
        for statement in injection.statements:
            clean_statement = str(statement).strip()
            if clean_statement and clean_statement not in declared_statements:
                declared_statements.append(clean_statement)
            parts = clean_statement.split()
            if parts and parts[0] not in {"myself", "nao_robot"} and parts[0] not in subjects:
                subjects.append(parts[0])
    fixture_subjects = set(subjects)
    world_before = query_kb_rows(
        container,
        patterns=("?subject ?predicate ?object",),
        query_vars=("?subject", "?predicate", "?object"),
        timeout_sec=20,
    )
    current_statements = list(declared_statements)
    for row in world_before.get("rows", []):
        subject = str(row.get("subject", row.get("?subject", ""))).strip()
        predicate = _canonical_kb_predicate(
            row.get("predicate", row.get("?predicate", ""))
        )
        obj = str(row.get("object", row.get("?object", ""))).strip()
        if (
            subject
            and predicate
            and obj
            and (subject in fixture_subjects or obj in fixture_subjects)
        ):
            current_statements.append(f"{subject} {predicate} {obj}")
    current_statements = list(dict.fromkeys(current_statements))
    retract_output = ""
    if current_statements:
        statements_yaml = "\n".join("  - '%s'" % item for item in current_statements)
        retract_output = call_ros_service(
            container,
            "/kb/revise",
            "kb_msgs/srv/Revise",
            f"""
method: retract
statements:
{statements_yaml}
models:
  - default
lifespan:
  sec: 1
  nanosec: 0
""",
            timeout_sec=20,
        )
        time.sleep(0.5)
    world_after = query_kb_rows(
        container,
        patterns=("?subject ?predicate ?object",),
        query_vars=("?subject", "?predicate", "?object"),
        timeout_sec=20,
    )
    remaining = [
        row
        for row in world_after.get("rows", [])
        if str(row.get("subject", row.get("?subject", ""))).strip()
        in fixture_subjects
        or str(row.get("object", row.get("?object", ""))).strip()
        in fixture_subjects
    ]
    return {
        "contaminated": bool(remaining),
        "subjects": subjects,
        "retracted_statement_count": len(current_statements),
        "remaining": remaining,
        "retract_output": retract_output,
    }


def _merge_cleanup_results(*results: dict[str, object]) -> dict[str, object]:
    clean_results = [item for item in results if item]
    if not clean_results:
        return {}
    return {
        "contaminated": any(bool(item.get("contaminated")) for item in clean_results),
        "subjects": list(
            dict.fromkeys(
                subject
                for item in clean_results
                for subject in item.get("subjects", [])
            )
        ),
        "retracted_statement_count": sum(
            int(item.get("retracted_statement_count", 0)) for item in clean_results
        ),
        "remaining": [
            row
            for item in clean_results
            for row in item.get("remaining", [])
        ],
        "parts": clean_results,
    }


def _canonical_kb_predicate(value) -> str:
    predicate = str(value or "").strip()
    if not predicate or ":" in predicate:
        return predicate
    if predicate == "type":
        return "rdf:type"
    if predicate in {"name", "color", "frameId", "poseSource", "poseX", "poseY"}:
        return f"dbp:{predicate}"
    return f"oro:{predicate}"


def environment_fixture_to_injection(fixture_id: str, fixture: dict) -> KbInjection:
    statements = tuple(
        str(item).strip()
        for item in fixture.get("statements", [])
        if str(item).strip()
    )
    query_patterns = tuple(
        str(item).strip()
        for item in fixture.get("query_patterns", [])
        if str(item).strip()
    )
    if not query_patterns:
        query_patterns = tuple(_query_patterns_for_fixture(statements))
    query_vars = tuple(
        str(item).strip()
        for item in fixture.get("query_vars", ("?predicate", "?object"))
        if str(item).strip()
    )
    return KbInjection(
        object_id=fixture_id,
        statements=statements,
        query_patterns=query_patterns,
        query_vars=query_vars or ("?predicate", "?object"),
    )


def _query_patterns_for_fixture(statements: tuple[str, ...]) -> list[str]:
    subjects: list[str] = []
    for statement in statements:
        subject = statement.split(maxsplit=1)[0] if statement.split() else ""
        if subject and subject not in subjects:
            subjects.append(subject)
    return ["%s ?predicate ?object" % subject for subject in subjects]


def run_absence_guards(container: str, guards: tuple[KbAbsenceGuard, ...]) -> dict[str, object]:
    """Return stale-world evidence for guards that require absent KB facts."""
    if not guards:
        return {}
    results: list[dict[str, object]] = []
    contaminated = False
    for guard in guards:
        query_output = query_kb_rows(
            container,
            patterns=guard.query_patterns,
            query_vars=guard.query_vars,
            timeout_sec=20,
        )
        row_count = len(query_output.get("rows", []))
        guard_result = {
            "name": guard.name,
            "query_patterns": list(guard.query_patterns),
            "query_vars": list(guard.query_vars),
            "row_count": row_count,
            "clean": row_count == 0,
            "query_output": query_output.get("raw_output", ""),
            "rows": query_output.get("rows", []),
        }
        if row_count:
            contaminated = True
        results.append(guard_result)
    return {
        "contaminated": contaminated,
        "guard_results": results,
        "contract": (
            "absent-target and recovery cases must prove the target is absent "
            "before injecting the turn"
        ),
    }


def query_kb_rows(
    container: str,
    *,
    patterns: tuple[str, ...],
    query_vars: tuple[str, ...],
    timeout_sec: int,
) -> dict[str, object]:
    patterns_yaml = "\n".join("  - '%s'" % item for item in patterns)
    vars_yaml = "\n".join("  - '%s'" % item for item in query_vars)
    query_request = f"""
patterns:
{patterns_yaml}
vars:
{vars_yaml}
models:
  - default
"""
    raw_output = call_ros_service(
        container,
        "/kb/query",
        "kb_msgs/srv/Query",
        query_request,
        timeout_sec=timeout_sec,
    )
    return {
        "raw_output": raw_output,
        "rows": parse_kb_query_rows(raw_output),
    }


def parse_kb_query_rows(service_output: str) -> list[dict]:
    """Extract KnowledgeCore JSON bindings from ros2 service-call text."""
    text = str(service_output or "")
    for match in re.finditer(
        r"json=(?P<quote>['\"])(?P<payload>.*?)(?P=quote)(?:[,)\n])",
        text,
        flags=re.DOTALL,
    ):
        rows = _parse_kb_json_payload(match.group("payload"))
        if rows:
            return rows
    for marker in ("json:", "json="):
        index = text.find(marker)
        if index < 0:
            continue
        rows = _parse_kb_json_payload(text[index + len(marker):].strip())
        if rows:
            return rows
    return []


def _parse_kb_json_payload(payload: str) -> list[dict]:
    clean_payload = str(payload or "").strip()
    if not clean_payload:
        return []
    if (clean_payload.startswith("'") and clean_payload.endswith("'")) or (
        clean_payload.startswith('"') and clean_payload.endswith('"')
    ):
        clean_payload = clean_payload[1:-1]
    clean_payload = clean_payload.encode("utf-8").decode("unicode_escape")
    try:
        parsed = json.loads(clean_payload)
    except json.JSONDecodeError:
        return []
    if isinstance(parsed, dict):
        parsed = [parsed]
    if not isinstance(parsed, list):
        return []
    return [row for row in parsed if isinstance(row, dict)]


def write_payload(
    out_path: str,
    container: str,
    case_set: str,
    started_at: float,
    results: list[dict],
    *,
    runtime_metadata: dict[str, object] | None = None,
    run_status: str = "running",
) -> None:
    status = str(run_status or "running").strip().lower()
    if status not in {"running", "completed", "timed_out"}:
        raise ValueError("unsupported questionnaire run status: %s" % status)
    now = time.time()
    payload = {
        "container": container,
        "case_set": case_set,
        "started_at_unix_sec": started_at,
        "updated_at_unix_sec": now,
        "run_status": status,
        "runtime_metadata": runtime_metadata or {},
        "cases": results,
    }
    if status != "running":
        payload["finished_at_unix_sec"] = now
    Path(out_path).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def structured_phase_observations(
    *,
    mode: str,
    turn_result: str,
    events: list[dict],
    turn_started_at: float,
) -> dict[str, object]:
    """Summarize one case from structured interaction-trace events."""
    case_events = [
        event
        for event in events
        if isinstance(event, dict)
        and float(event.get("timestamp", 0.0) or 0.0) >= float(turn_started_at) - 1.0
    ]
    turn_event = next(
        (
            event
            for event in case_events
            if str(event.get("event_type", "")) == "chatbot_turn_trace"
            and _event_payload(event).get("event_type") == "chatbot_turn_result"
            and bool(_event_payload(event).get("planner_handoff_allowed", True))
        ),
        None,
    )
    turn_payload = _event_payload(turn_event)
    turn_id = str(turn_payload.get("turn_id", "")).strip()
    route = str(turn_payload.get("route", "")).strip().lower()

    planner_event = next(
        (
            event
            for event in case_events
            if str(event.get("event_type", "")) == "planner_request"
            and (
                not turn_id
                or str(_planner_request_data(event).get("dialogue_turn_id", "")).strip()
                == turn_id
            )
        ),
        None,
    )
    planner_data = _planner_request_data(planner_event)
    goal_id = str(planner_data.get("goal_id", "")).strip()
    feedback_events = [
        event
        for event in case_events
        if str(event.get("event_type", "")) == "execution_feedback"
        and goal_id
        and str(_event_payload(event).get("goal_id", "")).strip() == goal_id
    ]
    planner_dialogue_events = [
        event
        for event in case_events
        if str(event.get("event_type", "")) == "planner_dialogue_act"
        and goal_id
        and str(_event_payload(event).get("goal_id", "")).strip() == goal_id
    ]

    feedback_payloads = [_event_payload(event) for event in feedback_events]
    feedback_types = {
        str(payload.get("event_type", "")).strip().lower()
        for payload in feedback_payloads
    }
    plan_ids = list(
        dict.fromkeys(
            str(payload.get("plan_id", "")).strip()
            for payload in feedback_payloads
            if str(payload.get("plan_id", "")).strip()
        )
    )
    plan_versions = [
        int(payload.get("plan_version", 0) or 0)
        for payload in feedback_payloads
    ]
    executed_skills = list(
        dict.fromkeys(
            str(payload.get("step", {}).get("name", "")).strip().lower()
            for payload in feedback_payloads
            if isinstance(payload.get("step"), dict)
            and str(payload.get("event_type", "")).strip().lower()
            in {"step_started", "step_succeeded", "step_failed"}
            and str(payload.get("step", {}).get("name", "")).strip()
        )
    )
    executed_skill_sequence = [
        str(payload.get("step", {}).get("name", "")).strip().lower()
        for payload in feedback_payloads
        if isinstance(payload.get("step"), dict)
        and str(payload.get("event_type", "")).strip().lower() == "step_started"
        and str(payload.get("step", {}).get("name", "")).strip()
    ]
    result_metadata = [
        payload.get("result_payload", {}).get("metadata", {})
        for payload in feedback_payloads
        if isinstance(payload.get("result_payload"), dict)
        and isinstance(payload.get("result_payload", {}).get("metadata"), dict)
    ]
    result_modes = list(
        dict.fromkeys(
            str(metadata.get("result_mode", "")).strip().lower()
            for metadata in result_metadata
            if str(metadata.get("result_mode", "")).strip()
        )
    )
    mode_sources = list(
        dict.fromkeys(
            str(metadata.get("mode_source", "")).strip().lower()
            for metadata in result_metadata
            if str(metadata.get("mode_source", "")).strip()
        )
    )

    target_selections = []
    selection = planner_data.get("target_selection", {})
    if isinstance(selection, dict) and selection:
        target_selections.append(selection)
    grounded_context = planner_data.get("grounded_context", {})
    grounded_entities = (
        grounded_context.get("entities", [])
        if isinstance(grounded_context, dict)
        else []
    )
    grounded_entity_ids = sorted(
        {
            str(entity.get("id", "")).strip()
            for entity in grounded_entities
            if isinstance(entity, dict) and str(entity.get("id", "")).strip()
        }
    )

    spoken_texts = []
    ack = str(turn_payload.get("verbal_ack", "")).strip()
    if ack:
        spoken_texts.append(ack)
    for payload in feedback_payloads:
        summary = str(payload.get("result_summary", "")).strip()
        step = payload.get("step", {})
        if summary and isinstance(step, dict) and step.get("name") == "report_result":
            spoken_texts.append(summary)
    spoken_texts = list(dict.fromkeys(spoken_texts))

    inconsistencies = []
    if turn_event is None:
        inconsistencies.append("structured trace is missing the chatbot turn result")
    if route == "execution" and planner_event is None:
        inconsistencies.append("execution route is missing its correlated planner request")
    if planner_event is not None and not goal_id:
        inconsistencies.append("planner request is missing goal lineage")
    if goal_id and not feedback_events and not planner_dialogue_events:
        inconsistencies.append("planner goal is missing execution feedback")

    planner_acts = {
        str(_event_payload(event).get("act", "")).strip().lower()
        for event in planner_dialogue_events
    }
    failure_observed = bool(
        "step_failed" in feedback_types
        or "plan_failed" in feedback_types
        or planner_acts.intersection({"explain_failure", "ask_clarification"})
    )
    terminal_observed_value = bool(
        feedback_types.intersection(
            {"plan_completed", "plan_failed", "plan_cancelled", "plan_rejected"}
        )
        or planner_acts.intersection({"explain_failure", "ask_clarification"})
    )
    injected = bool(str(turn_result or "").strip())
    if mode == "speech":
        injected = injected and "ERROR: dialogue_manager speech subscription" not in str(
            turn_result or ""
        )
    return {
        "turn_injected": injected,
        "route_observed": turn_event is not None,
        "planner_request_observed": planner_event is not None,
        "route_intent_handoff_observed": bool(
            turn_payload.get("planner_handoff_published")
        ),
        "route_intent_gap_count": 0,
        "target_selection_observed": any(
            item.get("member_ids") for item in target_selections
        ),
        "target_selections": target_selections,
        "grounded_entity_ids": grounded_entity_ids,
        "execution_feedback_observed": bool(feedback_events),
        "executed_skills": executed_skills,
        "executed_skill_sequence": executed_skill_sequence,
        "failure_observed": failure_observed,
        "replan_observed": any(version > 1 for version in plan_versions),
        "speech_observed": bool(spoken_texts),
        "spoken_texts": spoken_texts,
        "terminal_observed": terminal_observed_value,
        "post_terminal_speech_observed": False,
        "post_failure_speech_observed": bool(
            failure_observed
            and any(
                str(payload.get("result_summary", "")).strip()
                for payload in feedback_payloads
                if isinstance(payload.get("step"), dict)
                and payload.get("step", {}).get("name") == "report_result"
            )
        ),
        "clarification_observed": bool(
            "ask_clarification" in planner_acts
            or (
                route == "dialogue"
                and _contains_any(
                    ack,
                    ("clarify", "which ", "please specify", "need more information"),
                )
            )
        ),
        "fallback_markers": fallback_markers("\n".join(spoken_texts)),
        "evidence_source": "interaction_trace_jsonl",
        "correlation_status": "complete" if not inconsistencies else "incomplete",
        "evidence_consistent": not inconsistencies,
        "evidence_inconsistencies": inconsistencies,
        "lineage": {
            "dialogue_turn_id": turn_id,
            "goal_id": goal_id,
            "plan_ids": plan_ids,
            "plan_versions": sorted(set(plan_versions)),
        },
        "failure_injection": {
            "observed": failure_observed and bool(result_modes),
            "result_modes": result_modes,
            "mode_sources": mode_sources,
        },
        "planner_dialogue_acts": sorted(planner_acts),
        "structured_event_count": len(case_events),
        "observability_note": "semantic scoring uses structured trace lineage",
    }


def _event_payload(event: dict | None) -> dict:
    if not isinstance(event, dict):
        return {}
    payload = event.get("payload", {})
    return payload if isinstance(payload, dict) else {}


def _planner_request_data(event: dict | None) -> dict:
    payload = _event_payload(event)
    data = payload.get("data", {})
    return data if isinstance(data, dict) else {}


def phase_observations(
    *,
    mode: str,
    turn_result: str,
    log_excerpt: str,
    topic_samples: dict[str, str],
    voice_id: str = "",
) -> dict[str, object]:
    """Summarize runtime-review phase evidence without scoring the case."""
    correlated_log = correlate_case_evidence(log_excerpt, voice_id=voice_id)
    correlated_topics = [
        correlate_case_evidence(value, voice_id=voice_id)
        for value in topic_samples.values()
    ]
    combined = "\n".join(
        [
            str(turn_result or ""),
            correlated_log,
            "\n".join(correlated_topics),
        ]
    )
    injected = bool(str(turn_result or "").strip())
    if mode == "speech":
        injected = injected and "ERROR: dialogue_manager speech subscription" not in combined
    target_selections = extract_target_selections(combined)
    spoken_texts = extract_robot_speech_texts(
        "\n".join(
            [
                str(turn_result or ""),
                str(log_excerpt or ""),
                "\n".join(str(value or "") for value in topic_samples.values()),
            ]
        )
    )
    route_observed = _contains_any(combined, ("ROUTE_RESOLVED", "chatbot_turn_trace"))
    planner_request_observed = _contains_any(
        combined,
        ("PLANNER_REQUEST", "/planner/request", "planner_request"),
    )
    execution_feedback_observed = _contains_any(
        combined,
        (
            "execution_feedback",
            "/planner/execution_feedback",
            "step_succeeded",
            "step_failed",
            "plan_completed",
        ),
    )
    evidence_inconsistencies = []
    if execution_feedback_observed and not route_observed:
        evidence_inconsistencies.append("execution feedback exists without a correlated route marker")
    if execution_feedback_observed and not planner_request_observed:
        evidence_inconsistencies.append(
            "execution feedback exists without a correlated planner request marker"
        )
    executed_skills = _executed_skills(combined)
    return {
        "turn_injected": injected,
        "route_observed": route_observed,
        "planner_request_observed": planner_request_observed,
        "route_intent_handoff_observed": _contains_any(
            combined,
            ("ROUTE_INTENT_HANDOFF",),
        ),
        "route_intent_gap_count": _event_count(combined, "ROUTE_INTENT_GAP"),
        "target_selection_observed": any(
            selection.get("member_ids") for selection in target_selections
        ),
        "target_selections": target_selections,
        "execution_feedback_observed": execution_feedback_observed,
        "executed_skills": executed_skills,
        "failure_observed": _contains_any(
            combined,
            ("step_failed", "plan_failed", "Planned intent step failed"),
        ),
        "replan_observed": _contains_any(
            combined,
            ('"mode": "replan"', " mode=replan ", "plan_version=2", " version=2 "),
        ),
        "speech_observed": _contains_any(
            combined,
            ("ROBOT OUTPUT", "DEBUG_SPEECH", "/debug/nao_say/speech", "Robot saying"),
        ),
        "spoken_texts": spoken_texts,
        "terminal_observed": terminal_observed(combined),
        "post_terminal_speech_observed": post_terminal_speech_observed(combined),
        "post_failure_speech_observed": post_failure_speech_observed(combined),
        "clarification_observed": clarification_observed(combined),
        "fallback_markers": fallback_markers(combined),
        "evidence_consistent": not evidence_inconsistencies,
        "evidence_inconsistencies": evidence_inconsistencies,
        "observability_note": (
            "phase booleans are trace breadcrumbs, not pass/fail scoring"
        ),
    }


def extract_robot_speech_texts(value: str) -> list[str]:
    """Extract deduplicated robot utterances without user-input mirrors."""
    texts: list[str] = []
    for line in str(value or "").splitlines():
        candidates = []
        output_match = re.search(r'\[ROBOT OUTPUT\].*?"([^"]+)"', line)
        if output_match:
            candidates.append(output_match.group(1))
        for encoded in re.findall(r'\{"text":"((?:\\.|[^"\\])*)"\}', line):
            try:
                candidates.append(json.loads('"%s"' % encoded))
            except json.JSONDecodeError:
                continue
        for candidate in candidates:
            clean = str(candidate).strip()
            if clean and clean not in texts:
                texts.append(clean)
    return texts


def extract_target_selections(value: str) -> list[dict[str, object]]:
    """Extract normalized target selections from concise and JSON trace lines."""
    text = str(value or "")
    selections: list[dict[str, object]] = []
    seen: set[str] = set()
    decoder = json.JSONDecoder()

    for line in text.splitlines():
        marker = "target_selection="
        if marker in line:
            candidate = line.split(marker, 1)[1]
            if " source=" in candidate:
                candidate = candidate.split(" source=", 1)[0]
            try:
                parsed = ast.literal_eval(candidate.strip())
            except (SyntaxError, ValueError):
                parsed = None
            _append_target_selection(selections, seen, parsed)

        search_from = 0
        json_marker = '"target_selection"'
        while True:
            marker_index = line.find(json_marker, search_from)
            if marker_index < 0:
                break
            colon_index = line.find(":", marker_index + len(json_marker))
            if colon_index < 0:
                break
            candidate = line[colon_index + 1 :].lstrip()
            try:
                parsed, consumed = decoder.raw_decode(candidate)
            except json.JSONDecodeError:
                search_from = marker_index + len(json_marker)
                continue
            _append_target_selection(selections, seen, parsed)
            search_from = colon_index + 1 + consumed
    return selections


def _append_target_selection(
    selections: list[dict[str, object]],
    seen: set[str],
    value: object,
) -> None:
    if not isinstance(value, dict) or not value:
        return
    normalized = {
        "selection_kind": str(value.get("selection_kind", "")),
        "operation": str(value.get("operation", "")),
        "source_location_id": str(value.get("source_location_id", "")),
        "member_ids": sorted(
            {str(item) for item in value.get("member_ids", []) if str(item).strip()}
        ),
        "recipient_id": str(value.get("recipient_id", "")),
        "ordering": str(value.get("ordering", "")),
        "report_policy": str(value.get("report_policy", "")),
    }
    fingerprint = json.dumps(normalized, sort_keys=True)
    if fingerprint in seen:
        return
    seen.add(fingerprint)
    selections.append(normalized)


def correlate_case_evidence(value: str, *, voice_id: str) -> str:
    """Keep lines connected to one voice, dialogue, turn, or planner goal."""
    text = str(value or "")
    seed = str(voice_id or "").strip()
    if not text or not seed:
        return text
    lines = text.splitlines()
    identifiers = {seed}
    identifier_patterns = (
        re.compile(
            r"\b(?:goal_id|dialogue_id|dialogue|turn_id|turn)="
            r"[\"']?([A-Za-z0-9_.:\-]+)"
        ),
        re.compile(
            r"[\"'](?:goal_id|dialogue_id|dialogue|turn_id|turn)[\"']\s*:\s*"
            r"[\"']([A-Za-z0-9_.:\-]+)[\"']"
        ),
    )
    for _pass in range(3):
        changed = False
        for line in lines:
            if not any(identifier in line for identifier in identifiers):
                continue
            for pattern in identifier_patterns:
                for match in pattern.finditer(line):
                    identifier = match.group(1).strip()
                    if identifier and identifier not in identifiers:
                        identifiers.add(identifier)
                        changed = True
        if not changed:
            break
    scoped_indexes = {
        index
        for index, line in enumerate(lines)
        if any(identifier in line for identifier in identifiers)
    }
    scoped_lines = [lines[index] for index in sorted(scoped_indexes)]
    report_windows = _report_result_windows(scoped_lines)
    for index, line in enumerate(lines):
        if index in scoped_indexes or "robot output" not in line.lower():
            continue
        timestamp = _evidence_timestamp(line)
        if timestamp is not None and any(
            start <= timestamp <= end for start, end in report_windows
        ):
            scoped_indexes.add(index)
    return "\n".join(lines[index] for index in sorted(scoped_indexes))


def _report_result_windows(lines: list[str]) -> list[tuple[float, float]]:
    starts = []
    ends = []
    for line in lines:
        lowered = line.lower()
        if "step=report_result" not in lowered:
            continue
        timestamp = _evidence_timestamp(line)
        if timestamp is None:
            continue
        if "step_started" in lowered:
            starts.append(timestamp)
        elif "step_succeeded" in lowered or "step_failed" in lowered:
            ends.append(timestamp)
    windows = []
    for start in starts:
        end = next((candidate for candidate in ends if candidate >= start), None)
        if end is not None:
            windows.append((start, end))
    return windows


def _evidence_timestamp(line: str) -> float | None:
    match = re.search(r"\[(\d{3,}(?:\.\d+)?)\]", str(line or ""))
    return float(match.group(1)) if match else None


def fallback_markers(value: str) -> dict[str, int]:
    """Return per-case fallback markers for review, not pass/fail scoring."""
    text = str(value or "")
    patterns = {
        "llm_response_failed": r"llm response failed fallback",
        "llm_disabled": r"llm disabled fallback response",
        "rules_response_fallback": r"llm response fallback -> rules",
        "rules_intent_fallback": r"rules_llm_intent_fallback",
        "planner_invalid_json": r"model output did not contain a JSON object",
        "planner_invalid_plan": r"valid executable plan|model output did not contain executable steps",
        "planner_gate_rejected": r"planner_gate_rejected",
        "duplicate_active_goal": r"duplicate active planner goal",
        "planner_target_selection_recovery": r"validated_target_selection_recovery",
        "kb_service_timeout": r"kb service timeout|KnowledgeCore .*timeout|service call timed out",
        "report_result_fallback": r"report_result fallback|execution report chatbot returned error|execution report chatbot request failed",
        "route_repair": r"llm_response_route_repair|route_conflict",
        "complete_context_clarification": r"asked for clarification despite complete fixture context",
        "language_model_unreachable_speech": r"having trouble reaching my language model",
    }
    counts = {}
    lines = text.splitlines()
    for name, pattern in patterns.items():
        matching_lines = [
            line
            for line in lines
            if re.search(pattern, line, flags=re.IGNORECASE)
        ]
        counts[name] = len({_fallback_event_key(line) for line in matching_lines})
    counts["total"] = sum(counts.values())
    return counts


def _event_count(value: str, marker: str) -> int:
    """Count timestamp-correlated events without treating them as fallbacks."""
    keys = set()
    for line in str(value or "").splitlines():
        if marker not in line:
            continue
        goal_match = re.search(r"goal_id[=:][\"']?([A-Za-z0-9_.:-]+)", line)
        turn_match = re.search(
            r"dialogue_turn_id[=:][\"']?([A-Za-z0-9_.:-]+)",
            line,
        )
        if goal_match or turn_match:
            keys.add(
                (
                    marker,
                    goal_match.group(1) if goal_match else "",
                    turn_match.group(1) if turn_match else "",
                )
            )
        else:
            keys.add(_fallback_event_key(line))
    return len(keys)


def _fallback_event_key(line: str) -> str:
    timestamps = re.findall(r"\[(\d{3,}(?:\.\d+)?)\]", str(line or ""))
    if timestamps:
        return timestamps[-1]
    return " ".join(str(line or "").lower().split())


def clarification_observed(value: str) -> bool:
    """Return true when evidence shows a user-facing clarification request."""
    return _contains_any(
        value,
        (
            "act=ask_clarification",
            '"act": "ask_clarification"',
            "ask_clarification",
            "which location",
            "which object",
            "which person",
            "please specify",
            "could you clarify",
            "cannot confirm that person",
            "which person should i use",
            "need you to clarify",
            "need more information",
            "need help identifying",
        ),
    )


def assess_case(
    case: ProbeCase,
    *,
    observations: dict[str, object],
    stale_world_guard: dict | None,
    fake_policy_profile: str = "none",
) -> dict[str, object]:
    """Grade case semantics from trajectory breadcrumbs, not final text alone."""
    reasons: list[str] = []
    status = "pass"
    if stale_world_guard and stale_world_guard.get("contaminated"):
        return {
            "status": "not_scored",
            "expected_outcome": case.expected_outcome,
            "all_required_context": case.all_required_context,
            "reasons": ["stale KnowledgeCore guard contaminated the fixture"],
        }

    if not observations.get("turn_injected"):
        return {
            "status": "fail",
            "expected_outcome": case.expected_outcome,
            "all_required_context": case.all_required_context,
            "reasons": ["turn was not injected through the configured seam"],
        }

    if observations.get("evidence_consistent") is False:
        return {
            "status": "not_scored",
            "expected_outcome": case.expected_outcome,
            "all_required_context": case.all_required_context,
            "reasons": [
                "evidence correlation is internally inconsistent: %s"
                % "; ".join(observations.get("evidence_inconsistencies") or [])
            ],
        }

    required_grounded_ids = set(case.expected_member_ids)
    if case.expected_recipient_id:
        required_grounded_ids.add(case.expected_recipient_id)
    has_grounded_fixture_evidence = "grounded_entity_ids" in observations
    observed_grounded_ids = set(observations.get("grounded_entity_ids") or [])
    missing_grounded_ids = sorted(required_grounded_ids - observed_grounded_ids)
    if case.all_required_context and has_grounded_fixture_evidence and missing_grounded_ids:
        return {
            "status": "not_scored",
            "expected_outcome": case.expected_outcome,
            "all_required_context": case.all_required_context,
            "reasons": [
                "required grounded fixture entities were absent: %s"
                % ", ".join(missing_grounded_ids)
            ],
        }

    fallback = observations.get("fallback_markers") or {}
    severe_fallbacks = {
        key: value
        for key, value in fallback.items()
        if key
        in {
            "duplicate_active_goal",
            "language_model_unreachable_speech",
            "llm_response_failed",
            "kb_service_timeout",
            "planner_invalid_json",
            "planner_invalid_plan",
            "report_result_fallback",
            "rules_response_fallback",
        }
        and value
    }
    if severe_fallbacks:
        status = "fail"
        reasons.append("severe fallback markers: %s" % severe_fallbacks)

    expected = str(case.expected_outcome or "observe")
    clarified = bool(observations.get("clarification_observed"))
    planner_seen = bool(observations.get("planner_request_observed"))
    exec_seen = bool(observations.get("execution_feedback_observed"))
    terminal = bool(observations.get("terminal_observed"))
    speech = bool(observations.get("speech_observed"))
    failure_seen = bool(observations.get("failure_observed"))
    recovery_profile = str(fake_policy_profile or "none") not in {"none", "all_success"}
    expected_failure_skill = FAKE_POLICY_EXPECTED_SKILLS.get(
        str(fake_policy_profile or "none"),
        "",
    )

    if expected == "execute_no_clarification":
        if clarified:
            status = "fail" if case.all_required_context else max_status(status, "degraded")
            reasons.append("asked for clarification despite complete fixture context")
        if not planner_seen or not exec_seen:
            status = max_status(status, "fail")
            reasons.append("expected planner request and execution feedback")
        if case.requires_target_selection and not observations.get(
            "target_selection_observed"
        ):
            status = max_status(status, "fail")
            reasons.append("planner evidence did not carry target_selection")
        selection = _most_complete_target_selection(observations)
        if case.expected_member_ids and selection:
            expected_members = set(case.expected_member_ids)
            observed_members = set(selection.get("member_ids", []))
            selection_kind = str(selection.get("selection_kind", ""))
            members_match = (
                expected_members.issubset(observed_members)
                if selection_kind == "visible_objects"
                else observed_members == expected_members
            )
            if not members_match:
                status = max_status(status, "fail")
                reasons.append(
                    "selected members differed: expected=%s observed=%s"
                    % (sorted(expected_members), sorted(observed_members))
                )
        if case.expected_recipient_id and selection:
            observed_recipient = str(selection.get("recipient_id", ""))
            if observed_recipient != case.expected_recipient_id:
                status = max_status(status, "fail")
                reasons.append(
                    "selected recipient differed: expected=%s observed=%s"
                    % (case.expected_recipient_id, observed_recipient or "<missing>")
                )
        if case.expected_report_policy and selection:
            observed_policy = str(selection.get("report_policy", ""))
            if observed_policy != case.expected_report_policy:
                status = max_status(status, "fail")
                reasons.append(
                    "report policy differed: expected=%s observed=%s"
                    % (case.expected_report_policy, observed_policy or "<missing>")
                )
        if not terminal or not speech:
            status = max_status(status, "degraded")
            reasons.append("missing terminal or speech evidence")
        if (
            recovery_profile
            and expected_failure_skill
            and expected_failure_skill not in set(observations.get("executed_skills") or [])
        ):
            status = max_status(status, "not_scored")
            reasons.append(
                "configured fake failure for %s was not applicable to dispatched skills"
                % expected_failure_skill
            )
        elif recovery_profile and not failure_seen:
            status = max_status(status, "not_scored")
            reasons.append("configured fake failure was not exercised")
        if recovery_profile and failure_seen and not recovery_closure_observed(
            observations
        ):
            status = max_status(status, "degraded")
            reasons.append("recovery closure was not spoken after failure evidence")
        if recovery_profile and case.requires_replan and failure_seen and not observations.get(
            "replan_observed"
        ):
            status = max_status(status, "fail")
            reasons.append("configured recoverable failure did not produce a replan")
    elif expected == "dialogue_only":
        if planner_seen or exec_seen:
            status = max_status(status, "fail")
            reasons.append("dialogue-only turn leaked into planner execution")
        if not observations.get("route_observed") or not speech:
            status = max_status(status, "degraded")
            reasons.append("dialogue route or speech evidence was not observed")
    elif expected == "clarification_expected":
        if not clarified:
            status = max_status(status, "fail")
            reasons.append("expected clarification for deliberately absent/ambiguous target")
        if exec_seen:
            status = max_status(status, "fail")
            reasons.append("execution feedback appeared for a case that should clarify first")
        if not speech:
            status = max_status(status, "degraded")
            reasons.append("clarification/failure was not visible in speech")
    elif expected == "recover_or_truthful_failure":
        if not terminal or not speech:
            status = max_status(status, "degraded")
            reasons.append("recovery/failure case lacked terminal or speech evidence")
        elif not observations.get("post_terminal_speech_observed"):
            status = max_status(status, "degraded")
            reasons.append("recovery/failure speech was not observed after terminal evidence")
        if severe_fallbacks:
            status = max_status(status, "fail")
    else:
        if not speech:
            status = max_status(status, "degraded")
            reasons.append("speech evidence missing")

    observed_sequence = list(observations.get("executed_skill_sequence") or [])
    if case.expected_skill_sequence:
        missing_sequence = _missing_ordered_skills(
            list(case.expected_skill_sequence),
            observed_sequence,
        )
        if missing_sequence:
            status = max_status(status, "fail")
            reasons.append(
                "missing ordered skills: %s; observed=%s"
                % (
                    ",".join(missing_sequence),
                    ",".join(observed_sequence) or "<none>",
                )
            )
    forbidden = sorted(set(case.forbidden_skills).intersection(observed_sequence))
    if forbidden:
        status = max_status(status, "fail")
        reasons.append("forbidden skills executed: %s" % ",".join(forbidden))

    if case.postcondition is not None and not observations.get(
        "kb_postcondition_passed"
    ):
        status = max_status(status, "fail")
        reasons.append("declared KB postcondition was not observed")

    if case.expected_speech_terms:
        spoken_text = _normalize_speech_terms(
            " ".join(observations.get("spoken_texts") or [])
        )
        missing_terms = [
            term
            for term in case.expected_speech_terms
            if _normalize_speech_terms(term) not in spoken_text
        ]
        if missing_terms:
            status = max_status(status, "fail")
            reasons.append(
                "robot speech omitted grounded terms: %s"
                % ", ".join(missing_terms)
            )

    if not reasons:
        reasons.append("matched expected trajectory")
    return {
        "status": status,
        "expected_outcome": expected,
        "all_required_context": case.all_required_context,
        "reasons": reasons,
    }


def _normalize_speech_terms(value: str) -> str:
    """Normalize symbolic separators only for semantic term comparison."""
    return " ".join(re.findall(r"[a-z0-9]+", str(value or "").lower()))


def _missing_ordered_skills(required: list[str], observed: list[str]) -> list[str]:
    cursor = 0
    missing = []
    for skill in required:
        try:
            cursor = observed.index(skill, cursor) + 1
        except ValueError:
            missing.append(skill)
    return missing


def _executed_skills(value: str) -> list[str]:
    """Extract skills that reached execution feedback in correlated evidence."""
    observed = []
    known_skills = set(FAKE_POLICY_EXPECTED_SKILLS.values())
    for line in str(value or "").splitlines():
        lowered = line.lower()
        if not any(
            marker in lowered
            for marker in ("step_started", "step_succeeded", "step_failed")
        ):
            continue
        for skill in sorted(known_skills):
            if re.search(r"\b%s\b" % re.escape(skill), lowered) and skill not in observed:
                observed.append(skill)
    return observed


def _most_complete_target_selection(
    observations: dict[str, object],
) -> dict[str, object]:
    selections = observations.get("target_selections") or []
    if not isinstance(selections, list):
        return {}
    candidates = [selection for selection in selections if isinstance(selection, dict)]
    if not candidates:
        return {}
    return max(
        candidates,
        key=lambda selection: (
            len(selection.get("member_ids", [])),
            bool(selection.get("recipient_id")),
            bool(selection.get("report_policy")),
        ),
    )


def max_status(current: str, candidate: str) -> str:
    order = {"pass": 0, "degraded": 1, "fail": 2, "not_scored": 3}
    return candidate if order.get(candidate, 0) > order.get(current, 0) else current


def post_terminal_speech_observed(value: str) -> bool:
    """Require timestamped user-facing speech after a terminal trace event."""
    terminal_times = []
    speech_times = []
    timestamp_pattern = re.compile(r"\[(\d{9,}(?:\.\d+)?)\]")
    for line in str(value or "").splitlines():
        match = timestamp_pattern.search(line)
        if not match:
            continue
        timestamp = float(match.group(1))
        lowered = line.lower()
        if any(
            marker in lowered
            for marker in (
                "plan_completed",
                "plan_failed",
                "planner_gate_rejected",
                "ask_for_help",
                "ask_clarification",
                "explain_failure",
            )
        ):
            terminal_times.append(timestamp)
        if any(
            marker in lowered
            for marker in ("robot output", "debug_speech", "robot saying", "/debug/nao_say/speech")
        ):
            speech_times.append(timestamp)
    return bool(terminal_times and speech_times and max(speech_times) >= max(terminal_times))


def post_failure_speech_observed(value: str) -> bool:
    """Return true when user-facing speech follows the latest failed step."""
    failure_times = []
    speech_times = []
    timestamp_pattern = re.compile(r"\[(\d{3,}(?:\.\d+)?)\]")
    for line in str(value or "").splitlines():
        match = timestamp_pattern.search(line)
        if not match:
            continue
        timestamp = float(match.group(1))
        lowered = line.lower()
        if "step_failed" in lowered:
            failure_times.append(timestamp)
        if any(
            marker in lowered
            for marker in (
                "robot output",
                "debug_speech",
                "robot saying",
                "/debug/nao_say/speech",
            )
        ):
            speech_times.append(timestamp)
    return bool(
        failure_times
        and speech_times
        and max(speech_times) >= max(failure_times)
    )


def terminal_observed(value: str) -> bool:
    """Return true when logs show a terminal plan or planner-dialogue outcome."""
    return _contains_any(
        value,
        (
            "plan_completed",
            "plan_cancelled",
            "plan_invalid",
            "planner_gate_rejected",
            "act=explain_failure",
            '"act": "explain_failure"',
            "act=ask_clarification",
            '"act": "ask_clarification"',
            "act=ask_for_help",
            '"act": "ask_for_help"',
            "act=notify_completion",
            '"act": "notify_completion"',
            "act=notify_cancellation",
            '"act": "notify_cancellation"',
        ),
    )


def _contains_any(value: str, markers: tuple[str, ...]) -> bool:
    text = str(value or "")
    return any(marker in text for marker in markers)


def _observe_case_during_wait(
    out_path: str,
    container: str,
    case_set: str,
    started_at: float,
    results: list[dict],
    result_entry: dict,
    *,
    case: ProbeCase,
    mode: str,
    turn_result: str,
    case_start: float,
    wait_sec: float,
    runtime_metadata: dict[str, object],
    fake_policy_profile: str,
) -> None:
    """Flush per-case phase breadcrumbs while long fake-deep waits run."""
    deadline = time.time() + max(0.0, float(wait_sec or 0.0))
    interval_sec = 5.0
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        time.sleep(min(interval_sec, remaining))
        elapsed = max(0.0, time.time() - case_start)
        log_excerpt = recent_logs_since(container, case_start)
        structured_trace = collect_structured_trace_events(container, case_start)
        result_entry["wait_sec"] = min(max(0.0, float(wait_sec or 0.0)), elapsed)
        result_entry["log_excerpt"] = log_excerpt
        result_entry["structured_trace"] = {
            key: value for key, value in structured_trace.items() if key != "events"
        }
        result_entry["phase_observations"] = structured_phase_observations(
            mode=mode,
            turn_result=turn_result,
            events=structured_trace.get("events", []),
            turn_started_at=case_start,
        )
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=result_entry.get("stale_world_guard"),
            fake_policy_profile=fake_policy_profile,
        )
        result_entry["status"] = result_entry["case_assessment"]["status"]
        write_payload(
            out_path,
            container,
            case_set,
            started_at,
            results,
            runtime_metadata=runtime_metadata,
        )
        observations = result_entry["phase_observations"]
        if case_wait_complete(observations, fake_policy_profile):
            break
        if observations.get("clarification_observed") and observations.get("speech_observed"):
            break


def case_wait_complete(observations: dict, fake_policy_profile: str) -> bool:
    if not (
        observations.get("terminal_observed")
        and observations.get("speech_observed")
    ):
        return False
    recovery_profile = str(fake_policy_profile or "none") not in {
        "none",
        "all_success",
    }
    return not recovery_profile or recovery_closure_observed(observations)


def recovery_closure_observed(observations: dict) -> bool:
    return bool(
        observations.get("post_terminal_speech_observed")
        or observations.get("post_failure_speech_observed")
    )


def collect_questionnaire_metadata(
    container: str,
    *,
    expected_turn_pipeline_mode: str,
) -> dict[str, object]:
    active_mode = get_ros_param(
        container,
        "/chatbot_llm",
        "turn_pipeline_mode",
    )
    expected = str(expected_turn_pipeline_mode or "").strip()
    grounded_context_digest_enabled = get_ros_param(
        container,
        "/chatbot_llm",
        "grounded_context_digest_enabled",
    )
    chatbot_generation = {
        param_name: get_ros_param(container, "/chatbot_llm", param_name)
        for param_name in (
            "model",
            "intent_model",
            "temperature",
            "top_p",
            "top_k",
            "min_p",
            "presence_penalty",
            "repetition_penalty",
            "response_max_tokens",
            "intent_max_tokens",
            "request_timeout_sec",
            "first_request_timeout_sec",
            "intent_request_timeout_sec",
            "think",
        )
    }
    planner_generation = {
        param_name: get_ros_param(container, "/planner_llm", param_name)
        for param_name in (
            "provider",
            "model",
            "temperature",
            "top_p",
            "top_k",
            "min_p",
            "presence_penalty",
            "repetition_penalty",
            "max_tokens",
            "timeout_sec",
            "think",
        )
    }
    return {
        "chatbot_turn_pipeline_mode": active_mode,
        "grounded_context_digest_enabled": grounded_context_digest_enabled,
        "chatbot_generation": chatbot_generation,
        "planner_generation": planner_generation,
        "expected_turn_pipeline_mode": expected,
        "turn_pipeline_mode_matches_expected": (
            True if not expected else active_mode == expected
        ),
        "ablation_note": (
            "intent_ablation is meaningful only when the active launch sets "
            "chatbot_turn_pipeline_mode:=intent_first or when comparing against "
            "a response_first control run."
        ),
    }


def get_ros_param(container: str, node_name: str, param_name: str) -> object:
    script = f"""
{ROS_CLI_PREAMBLE}
timeout 8 ros2 param get {node_name} {param_name} 2>/dev/null || true
"""
    output = run(
        ["docker", "exec", container, "bash", "-lc", script],
        timeout=12,
        check=False,
    )
    for line in reversed(output.splitlines()):
        text = line.strip()
        value_marker = " value is:"
        if value_marker not in text:
            continue
        value_type, raw_value = text.split(value_marker, 1)
        value = raw_value.strip()
        if value_type == "String":
            return value
        if value_type == "Boolean":
            return value.lower() == "true"
        if value_type == "Integer":
            try:
                return int(value)
            except ValueError:
                return value
        if value_type == "Double":
            try:
                return float(value)
            except ValueError:
                return value
        return value
    return output.strip()


def injection_scope(mode: str) -> str:
    if mode == "speech":
        return "full_ros4hri_dialogue_ingress"
    if mode == "chatbot_service":
        return "chatbot_llm_only_no_dialogue_manager_or_orchestrator"
    return mode


def publish_voice_turn(
    container: str,
    text: str,
    *,
    voice_id: str,
    mirror_rqt_display: bool,
) -> str:
    escaped_text = text.replace("\\", "\\\\").replace('"', '\\"')
    yaml_text = text.replace("'", "''")
    voice_topic = _voice_speech_topic(voice_id)
    safe_voice_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", voice_id)
    tracked_pid_file = f"/tmp/nao_questionnaire_tracked_{safe_voice_id}.pid"
    mirror_script = ""
    if mirror_rqt_display:
        mirror_script = f"""
cat >/tmp/nao_questionnaire_rqt_rosout.yaml <<'EOF'
stamp:
  sec: 0
  nanosec: 0
level: 20
name: runtime_review_rqt_input
msg: '[runtime-review INPUT] {yaml_text}'
file: run_active_questionnaire.py
function: publish_voice_turn
line: 0
EOF
timeout 4 ros2 topic pub --once -w 1 {RQT_DISPLAY_ROSOUT_QOS} {RQT_DISPLAY_ROSOUT_TOPIC} rcl_interfaces/msg/Log "$(cat /tmp/nao_questionnaire_rqt_rosout.yaml)" >/tmp/nao_questionnaire_rqt_rosout.log 2>&1 || true
cat >/tmp/nao_questionnaire_rqt_caption.yaml <<'EOF'
speaker_id: "{voice_id}"
text: '{yaml_text}'
locale: en_US
EOF
timeout 4 ros2 topic pub --once -w 1 {RQT_DISPLAY_CAPTIONS_TOPIC} hri_actions_msgs/msg/ClosedCaption "$(cat /tmp/nao_questionnaire_rqt_caption.yaml)" >/tmp/nao_questionnaire_rqt_caption.log 2>&1 || true
"""
    script = f"""
set -e
{ROS_CLI_PREAMBLE}
cat >/tmp/nao_questionnaire_qos_contract.log <<'EOF'
[runtime-review] Speech ingress seam:
  tracked_topic={VOICE_TRACKED_TOPIC}
  tracked_qos={VOICE_TRACKED_QOS}
  speech_topic={voice_topic}
  speech_qos={VOICE_SPEECH_QOS}
  rqt_display_mirror={str(mirror_rqt_display).lower()}
  rqt_display_topics={RQT_DISPLAY_ROSOUT_TOPIC},{RQT_DISPLAY_CAPTIONS_TOPIC}
  contract=publish tracked voice with TRANSIENT_LOCAL durability and keep the publisher alive until the per-voice LiveSpeech subscription appears.
  reason=dialogue_manager creates the remapped per-voice rqt-chat subscription only after seeing the tracked voice id.
EOF
	{mirror_script}
	dialogue_state="$(ros2 lifecycle get /dialogue_manager 2>/dev/null || true)"
	echo "  dialogue_manager_lifecycle=${{dialogue_state:-unavailable}}" >>/tmp/nao_questionnaire_qos_contract.log
	for prior_pid_file in /tmp/nao_questionnaire_tracked_*.pid; do
	  [ -e "$prior_pid_file" ] || continue
	  [ "$prior_pid_file" = "{tracked_pid_file}" ] && continue
	  prior_pid="$(cat "$prior_pid_file" 2>/dev/null || true)"
	  if [ -n "$prior_pid" ]; then
	    kill "$prior_pid" >/dev/null 2>&1 || true
	  fi
	  rm -f "$prior_pid_file"
	done
	tracked_pub_pid="$(cat {tracked_pid_file} 2>/dev/null || true)"
	if [ -z "$tracked_pub_pid" ] || ! kill -0 "$tracked_pub_pid" 2>/dev/null; then
	  nohup ros2 topic pub -r 2 {VOICE_TRACKED_QOS} {VOICE_TRACKED_TOPIC} hri_msgs/msg/IdsList "{{ids: ['{voice_id}']}}" >/tmp/nao_questionnaire_voice_{safe_voice_id}.log 2>&1 &
	  tracked_pub_pid=$!
	  echo "$tracked_pub_pid" > {tracked_pid_file}
	fi
	subscription_visible=false
	for _ in $(seq 1 30); do
	  if timeout 2 ros2 topic info -v {voice_topic} 2>/dev/null | grep -q 'Node name: dialogue_manager'; then
	    subscription_visible=true
	    break
  fi
  sleep 0.5
done
	timeout 3 ros2 topic info -v {VOICE_TRACKED_TOPIC} >/tmp/nao_questionnaire_tracked_info.log 2>&1 || true
	timeout 3 ros2 topic info -v {voice_topic} >/tmp/nao_questionnaire_speech_info.log 2>&1 || true
	if ! grep -q 'Node name: dialogue_manager' /tmp/nao_questionnaire_speech_info.log 2>/dev/null; then
	  echo "[runtime-review] ERROR: dialogue_manager speech subscription was not visible for {voice_topic}; this is a lifecycle/ingress preflight failure, not a model result." >>/tmp/nao_questionnaire_qos_contract.log
	else
	  echo "[runtime-review] OK: dialogue_manager speech subscription visible before LiveSpeech publish for {voice_topic}." >>/tmp/nao_questionnaire_qos_contract.log
	fi
timeout 8 ros2 topic pub --once -w 1 {VOICE_SPEECH_QOS} {voice_topic} hri_msgs/msg/LiveSpeech "{{final: \\"{escaped_text}\\", confidence: 1.0, locale: \\"en_US\\"}}" >/tmp/nao_questionnaire_speech.log 2>&1 || true
cat /tmp/nao_questionnaire_qos_contract.log /tmp/nao_questionnaire_rqt_rosout.log /tmp/nao_questionnaire_rqt_caption.log /tmp/nao_questionnaire_voice.log /tmp/nao_questionnaire_tracked_info.log /tmp/nao_questionnaire_speech_info.log /tmp/nao_questionnaire_speech.log 2>/dev/null || true
"""
    return run(["docker", "exec", container, "bash", "-lc", script], timeout=45, check=False)


def cleanup_tracked_voice_publishers(container: str) -> str:
    script = """
for pid_file in /tmp/nao_questionnaire_tracked_*.pid; do
  [ -e "$pid_file" ] || continue
  tracked_pid="$(cat "$pid_file" 2>/dev/null || true)"
  if [ -n "$tracked_pid" ]; then
    kill "$tracked_pid" >/dev/null 2>&1 || true
  fi
  rm -f "$pid_file"
done
"""
    return run(
        ["docker", "exec", container, "bash", "-lc", script],
        timeout=12,
        check=False,
    )


def inject_kb_probe(container: str, injection: KbInjection, *, lifespan_sec: int) -> dict[str, object]:
    statements_yaml = "\n".join("  - '%s'" % item for item in injection.statements)
    revise_request = f"""
method: update
statements:
{statements_yaml}
models:
  - default
lifespan:
  sec: {max(1, int(lifespan_sec))}
  nanosec: 0
"""
    retract_output = ""
    if injection.retract_statements:
        retract_yaml = "\n".join(
            "  - '%s'" % item for item in injection.retract_statements
        )
        retract_output = call_ros_service(
            container,
            "/kb/revise",
            "kb_msgs/srv/Revise",
            f"""
method: retract
statements:
{retract_yaml}
models:
  - default
lifespan:
  sec: 1
  nanosec: 0
""",
            timeout_sec=20,
        )
    service_probe = run(
        [
            "docker",
            "exec",
            container,
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /home/ubuntu/ws/install/setup.bash 2>/dev/null || true; "
                "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
                "ros2 service list -t | grep -E '/kb/(revise|query)' || true"
            ),
        ],
        timeout=10,
        check=False,
    )
    revise_output = call_ros_service(
        container,
        "/kb/revise",
        "kb_msgs/srv/Revise",
        revise_request,
        timeout_sec=20,
    )
    time.sleep(1.0)
    query_result = _wait_for_fixture_type_rows(container, injection)
    return {
        "object_id": injection.object_id,
        "service_probe": service_probe,
        "retract_output": retract_output,
        "revise_output": revise_output,
        "query_output": query_result["raw_output"],
        "fixture_readiness": query_result["readiness"],
    }


def evaluate_kb_postcondition(
    container: str,
    postcondition: KbPostcondition,
) -> dict[str, object]:
    """Query one declared symbolic postcondition after a user turn."""
    result = query_kb_rows(
        container,
        patterns=postcondition.query_patterns,
        query_vars=postcondition.query_vars,
        timeout_sec=20,
    )
    rows = result.get("rows", [])
    observed_values = {
        str(value).strip()
        for row in rows
        if isinstance(row, dict)
        for value in row.values()
        if str(value).strip()
    }
    missing_values = sorted(set(postcondition.expected_values) - observed_values)
    passed = len(rows) >= max(0, postcondition.min_rows) and not missing_values
    return {
        "name": postcondition.name,
        "passed": passed,
        "row_count": len(rows),
        "rows": rows,
        "missing_values": missing_values,
        "raw_output": result.get("raw_output", ""),
    }


def _wait_for_fixture_type_rows(
    container: str,
    injection: KbInjection,
) -> dict[str, object]:
    """Wait for declared fixture RDF types before starting a dialogue case."""
    required_types = _fixture_type_bindings(injection)
    deadline = time.monotonic() + KB_FIXTURE_READY_TIMEOUT_SEC
    latest_raw_output = ""
    while True:
        present, latest_raw_output = _query_fixture_type_bindings(
            container,
            required_types,
        )
        missing = sorted(required_types - present)
        if not missing:
            return {
                "raw_output": latest_raw_output,
                "readiness": {
                    "ready": True,
                    "required_type_count": len(required_types),
                    "missing_type_bindings": [],
                    "poll_timeout_sec": KB_FIXTURE_READY_TIMEOUT_SEC,
                },
            }
        if time.monotonic() >= deadline:
            return {
                "raw_output": latest_raw_output,
                "readiness": {
                    "ready": False,
                    "required_type_count": len(required_types),
                    "missing_type_bindings": missing,
                    "poll_timeout_sec": KB_FIXTURE_READY_TIMEOUT_SEC,
                },
            }
        time.sleep(KB_FIXTURE_READY_POLL_SEC)


def _query_fixture_type_bindings(
    container: str,
    required_types: set[tuple[str, str]],
) -> tuple[set[tuple[str, str]], str]:
    """Query each constant fixture subject so subject identity is preserved."""
    present = set()
    latest_raw_output = ""
    subjects = sorted({subject for subject, _ in required_types})
    for subject in subjects:
        result = query_kb_rows(
            container,
            patterns=(f"{subject} rdf:type ?object",),
            query_vars=("?object",),
            timeout_sec=20,
        )
        latest_raw_output = str(result.get("raw_output", ""))
        for row in result.get("rows", []):
            if not isinstance(row, dict):
                continue
            obj = str(row.get("object", "")).strip()
            if obj:
                present.add((subject, obj))
    return present, latest_raw_output


def _fixture_type_bindings(injection: KbInjection) -> set[tuple[str, str]]:
    bindings = set()
    for statement in injection.statements:
        parts = str(statement or "").split()
        if len(parts) < 3 or parts[0] in {"myself", "nao_robot"}:
            continue
        if parts[1] == "rdf:type":
            bindings.add((parts[0], " ".join(parts[2:])))
    return bindings


def _present_fixture_type_bindings(rows) -> set[tuple[str, str]]:
    present = set()
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        predicate = str(row.get("predicate", "")).strip()
        entity = str(row.get("entity", row.get("subject", ""))).strip()
        obj = str(row.get("object", "")).strip()
        if entity and predicate == "rdf:type" and obj:
            present.add((entity, obj))
    return present


def call_ros_service(
    container: str,
    service_name: str,
    service_type: str,
    request_yaml: str,
    *,
    timeout_sec: int,
) -> str:
    script = f"""
set -e
{ROS_CLI_PREAMBLE}
cat >/tmp/nao_questionnaire_service_request.yaml
timeout {timeout_sec} ros2 service call --stdin {service_name} {service_type} < /tmp/nao_questionnaire_service_request.yaml
"""
    return run(
        ["docker", "exec", "-i", container, "bash", "-lc", script],
        timeout=timeout_sec + 5,
        check=False,
        input_text=request_yaml,
    )


def call_chatbot_turn(
    container: str,
    text: str,
    sequence: int,
    *,
    voice_id: str,
    history: list[dict[str, str]],
) -> str:
    uuid_tail = max(1, min(255, sequence))
    history_items = list(history) + [{"speaker": voice_id, "text": text}]
    history_yaml = "\n".join(
        "- speaker: \"%s\"\n  text: '%s'\n  timestamp: 0.0"
        % (
            item["speaker"].replace('"', '\\"'),
            item["text"].replace("'", "''"),
        )
        for item in history_items
    )
    request = f"""
dialogue_id:
  uuid: [{uuid_tail}, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {uuid_tail}]
role:
  name: "__default__"
  configuration: "{{}}"
locale: "en_US"
summary: ""
history:
{history_yaml}
"""
    script = f"""
set -e
{ROS_CLI_PREAMBLE}
cat >/tmp/nao_questionnaire_request.yaml
timeout 90 ros2 service call --stdin /chatbot_llm/dialogue_interaction chatbot_msgs/srv/DialogueInteraction < /tmp/nao_questionnaire_request.yaml
"""
    return run(
        ["docker", "exec", "-i", container, "bash", "-lc", script],
        timeout=100,
        check=False,
        input_text=request,
    )


def extract_service_response(output: str) -> str:
    match = re.search(r"response=(['\"])(.*?)\1, intents=", output, flags=re.DOTALL)
    if not match:
        return ""
    return match.group(2).encode("utf-8").decode("unicode_escape").strip()


def sample_topics(container: str) -> dict[str, str]:
    samples = {}
    for topic in TOPICS_TO_SAMPLE:
        script = f"""
{ROS_CLI_PREAMBLE}
timeout --kill-after={TOPIC_SAMPLE_KILL_AFTER_SEC}s {TOPIC_SAMPLE_TIMEOUT_SEC}s ros2 topic echo {topic} --once 2>/dev/null || true
"""
        samples[topic] = run(
            ["docker", "exec", container, "bash", "-lc", script],
            timeout=TOPIC_SAMPLE_TIMEOUT_SEC + TOPIC_SAMPLE_KILL_AFTER_SEC + 3,
            check=False,
        )
    return samples


def parse_structured_trace_jsonl(
    chunks: list[str],
    *,
    since_unix_sec: float,
) -> list[dict]:
    """Parse, time-bound, and deduplicate interaction-trace JSONL records."""
    events = []
    seen = set()
    since = float(since_unix_sec or 0.0)
    for chunk in chunks:
        for raw_line in str(chunk or "").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("TRACE_FILE_COUNT="):
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(event, dict):
                continue
            try:
                timestamp = float(event.get("timestamp", 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            if timestamp < since:
                continue
            fingerprint = json.dumps(event, sort_keys=True, separators=(",", ":"))
            if fingerprint in seen:
                continue
            seen.add(fingerprint)
            events.append(event)
    return sorted(events, key=lambda event: float(event.get("timestamp", 0.0) or 0.0))


def collect_structured_trace_events(container: str, since_unix_sec: float) -> dict:
    """Read structured trace records emitted since one case began."""
    script = (
        "python3 - <<'PY'\n"
        "import json\n"
        "from pathlib import Path\n"
        f"since={max(0.0, float(since_unix_sec) - 1.0)!r}\n"
        "paths=sorted(Path('/root/.ros/nao_ros4hri_traces').glob('*.jsonl'))\n"
        "print('TRACE_FILE_COUNT=%d' % len(paths))\n"
        "for path in paths:\n"
        "    try:\n"
        "        handle=path.open('r', encoding='utf-8', errors='replace')\n"
        "    except OSError:\n"
        "        continue\n"
        "    with handle:\n"
        "        for raw in handle:\n"
        "            try:\n"
        "                event=json.loads(raw)\n"
        "                timestamp=float(event.get('timestamp', 0.0) or 0.0)\n"
        "            except Exception:\n"
        "                continue\n"
        "            if timestamp >= since:\n"
        "                print(json.dumps(event, separators=(',', ':')))\n"
        "PY"
    )
    output = run(
        ["docker", "exec", container, "bash", "-lc", script],
        timeout=15,
        check=False,
    ) or ""
    count_match = re.search(r"^TRACE_FILE_COUNT=(\d+)$", output, flags=re.MULTILINE)
    file_count = int(count_match.group(1)) if count_match else 0
    events = parse_structured_trace_jsonl(
        [output],
        since_unix_sec=max(0.0, float(since_unix_sec) - 1.0),
    )
    return {
        "available": file_count > 0,
        "file_count": file_count,
        "event_count": len(events),
        "events": events,
    }


def recent_logs(container: str, since_sec: int) -> str:
    output = run(
        ["docker", "logs", "--since", f"{max(1, since_sec)}s", container],
        timeout=15,
        check=False,
    ) or ""
    interesting = []
    markers = (
        "SPEECH INPUT",
        "CHATBOT",
        "ROUTE_RESOLVED",
        "ROUTE_INTENT_HANDOFF",
        "ROUTE_INTENT_GAP",
        "PLANNER_REQUEST",
        "planner_llm",
        "execution_feedback",
        "report_result",
        "DEBUG_SPEECH",
        "ROBOT OUTPUT",
        "GROUNDED_CONTEXT",
        "ERROR",
        "WARN",
    )
    for line in output.splitlines():
        if any(marker in line for marker in markers):
            interesting.append(line)
    return "\n".join(interesting[-260:])


def recent_logs_since(container: str, since_unix_sec: float) -> str:
    since_value = max(0.0, float(since_unix_sec) - 1.0)
    script = (
        "python3 - <<'PY'\n"
        "from pathlib import Path\n"
        f"since={since_value!r}\n"
        "paths=[Path('/root/.ros/log/latest/launch.log')]\n"
        "paths.extend(sorted(Path('/root/.ros/log').glob('python3_*.log')))\n"
        "for path in paths:\n"
        "    if not path.exists() or not path.is_file():\n"
        "        continue\n"
        "    try:\n"
        "        lines=path.read_text(errors='replace').splitlines()\n"
        "    except Exception:\n"
        "        continue\n"
        "    for line in lines:\n"
        "        import re\n"
        "        match = re.search(r'(?<![0-9])1[0-9]{9}\\.[0-9]+(?![0-9])', line)\n"
        "        if match is not None:\n"
        "            try:\n"
        "                if float(match.group(0)) < since:\n"
        "                    continue\n"
        "            except Exception:\n"
        "                pass\n"
        "        print(line)\n"
        "PY"
    )
    output = run(
        ["docker", "exec", container, "bash", "-lc", script],
        timeout=15,
        check=False,
    ) or ""
    interesting = []
    markers = (
        "SPEECH INPUT",
        "CHATBOT",
        "ROUTE_RESOLVED",
        "ROUTE_INTENT_HANDOFF",
        "ROUTE_INTENT_GAP",
        "PLANNER_REQUEST",
        "planner_llm",
        "execution_feedback",
        "report_result",
        "DEBUG_SPEECH",
        "ROBOT OUTPUT",
        "GROUNDED_CONTEXT",
        "ERROR",
        "WARN",
    )
    for line in output.splitlines():
        if any(marker in line for marker in markers):
            interesting.append(line)
    return "\n".join(interesting[-260:])


def run(
    cmd: list[str],
    *,
    timeout: float,
    check: bool = True,
    input_text: str | None = None,
) -> str:
    try:
        completed = subprocess.run(
            cmd,
            input=input_text,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as err:
        partial = err.stdout or ""
        if isinstance(partial, bytes):
            partial = partial.decode(errors="replace")
        return "%s\n[TIMEOUT after %.1fs]" % (str(partial).strip(), timeout)
    if check and completed.returncode != 0:
        raise RuntimeError(
            "command failed (%s):\n%s" % (" ".join(cmd), completed.stdout)
        )
    return completed.stdout.strip()


def _voice_id_for_group(group: str, index: int) -> str:
    text = re.sub(r"[^a-z0-9_]+", "_", str(group or "").strip().lower()).strip("_")
    if not text:
        text = f"case_{index}"
    return f"codex_{text}"


def _voice_id_for_case(
    mode: str,
    *,
    group: str,
    case_name: str,
    index: int,
    speech_voice_scope: str,
) -> str:
    if mode != "speech":
        return _voice_id_for_group(group, index)
    if speech_voice_scope == "case":
        return _voice_id_for_group(case_name, index)
    if speech_voice_scope == "group":
        return _voice_id_for_group(group, index)
    return VOICE_ID


def _voice_speech_topic(voice_id: str) -> str:
    clean_voice = str(voice_id or VOICE_ID).strip() or VOICE_ID
    if clean_voice == VOICE_ID:
        return VOICE_SPEECH_TOPIC
    return f"{VOICE_SPEECH_PREFIX}/{clean_voice}/speech"


if __name__ == "__main__":
    raise SystemExit(main())
