#!/usr/bin/env python3
"""Inject ROS4HRI speech turns and collect runtime evidence per case."""

from __future__ import annotations

import argparse
from datetime import datetime
from datetime import timezone
import json
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
# subscribes to the raw per-voice HRI speech topic it constructs internally.
VOICE_SPEECH_PREFIX = "/humans/voices"
VOICE_SPEECH_TOPIC = f"{VOICE_SPEECH_PREFIX}/{VOICE_ID}/speech"
VOICE_TRACKED_QOS = "--qos-reliability reliable --qos-durability transient_local"
VOICE_SPEECH_QOS = "--qos-reliability reliable --qos-durability volatile"
RQT_DISPLAY_ROSOUT_TOPIC = "/rosout"
RQT_DISPLAY_CAPTIONS_TOPIC = "/dialogue_manager/closed_captions"
RQT_DISPLAY_ROSOUT_QOS = "--qos-reliability reliable --qos-durability transient_local"
TOPIC_SAMPLE_TIMEOUT_SEC = 1.5
TOPIC_SAMPLE_KILL_AFTER_SEC = 1.0
DEFAULT_GLOBAL_TIMEOUT_SEC = 420
DEFAULT_KB_LIFESPAN_SEC = 300
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
    ProbeCase("dialogue_greeting", "simple_dialogue", "Hey, how are you?", 8.0, conversation_group="dialogue_basic"),
    ProbeCase(
        "dialogue_favorite_movie",
        "simple_dialogue",
        "What is your favourite movie?",
        8.0,
        conversation_group="dialogue_basic",
    ),
    ProbeCase(
        "dialogue_movie_followup",
        "simple_dialogue",
        "My favourite movie is Back to the Future!",
        8.0,
        conversation_group="dialogue_basic",
    ),
    ProbeCase(
        "dialogue_weekend_plans",
        "simple_dialogue",
        "Any ideas for plans this weekend?",
        8.0,
        conversation_group="dialogue_basic",
    ),
    ProbeCase(
        "dialogue_speed_of_light",
        "simple_dialogue",
        "What is the speed of light?",
        8.0,
        conversation_group="dialogue_basic",
    ),
    ProbeCase("kb_visible_now_baseline", "kb_query_dialogue", "What can you see now?", 10.0),
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
    ),
    ProbeCase("skill_head_up", "simple_skill_execution", "Move your head up.", 80.0),
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
    ),
    ProbeCase(
        "kb_mutation_add_red_cup",
        "kb_mutation_dialogue",
        "Add a red cup to your KB.",
        20.0,
    ),
    ProbeCase(
        "composite_head_all_directions",
        "composite_skill_execution",
        "Can you move your head in all directions?",
        95.0,
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
    "recipient_missing": {
        "global_mode": "scenario",
        "mode_overrides_json": '{"bring_object":"recipient_unavailable"}',
    },
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
            "environment",
            "fake_deep",
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
        "environment": ENVIRONMENT_CASES,
        "fake_deep": FAKE_DEEP_CASES,
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
    runtime_metadata["environment_fixture_source"] = str(environment_fixture_path)
    runtime_metadata["available_environment_fixtures"] = sorted(environment_fixtures.keys())
    runtime_metadata["fake_policy_profile"] = args.fake_policy_profile
    runtime_metadata["fake_policy_application"] = apply_fake_policy_profile(
        args.container,
        args.fake_policy_profile,
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
    for index, case in enumerate(cases, start=1):
        if time.time() - started_at > max(30, args.global_timeout_sec):
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
            )
            return 2

        case_start = time.time()
        setup_result = {}
        if case.environment_ids:
            setup_result["environment"] = inject_environment_fixtures(
                args.container,
                list(case.environment_ids),
                environment_fixtures=environment_fixtures,
                lifespan_sec=args.kb_lifespan_sec,
            )
        if case.setup is not None:
            setup_result["case"] = inject_kb_probe(
                args.container,
                case.setup,
                lifespan_sec=args.kb_lifespan_sec,
            )
        stale_world_guard = run_absence_guards(args.container, case.absence_guards)
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
            ),
        }
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=stale_world_guard,
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
        )
        topic_samples = sample_topics(args.container) if args.sample_topics else {}
        log_excerpt = recent_logs_since(args.container, turn_start)
        result_entry["wait_sec"] = wait_sec
        result_entry["topic_samples"] = topic_samples
        result_entry["log_excerpt"] = log_excerpt
        result_entry["phase_observations"] = phase_observations(
            mode=mode,
            turn_result=turn_result,
            log_excerpt=log_excerpt,
            topic_samples=topic_samples,
        )
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=stale_world_guard,
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

    write_payload(
        args.out,
        args.container,
        args.case_set,
        started_at,
        results,
        runtime_metadata=runtime_metadata,
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
) -> None:
    payload = {
        "container": container,
        "case_set": case_set,
        "started_at_unix_sec": started_at,
        "finished_at_unix_sec": time.time(),
        "runtime_metadata": runtime_metadata or {},
        "cases": results,
    }
    Path(out_path).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def phase_observations(
    *,
    mode: str,
    turn_result: str,
    log_excerpt: str,
    topic_samples: dict[str, str],
) -> dict[str, object]:
    """Summarize runtime-review phase evidence without scoring the case."""
    combined = "\n".join(
        [
            str(turn_result or ""),
            str(log_excerpt or ""),
            "\n".join(str(value or "") for value in topic_samples.values()),
        ]
    )
    injected = bool(str(turn_result or "").strip())
    if mode == "speech":
        injected = injected and "ERROR: dialogue_manager speech subscription" not in combined
    return {
        "turn_injected": injected,
        "route_observed": _contains_any(combined, ("ROUTE_RESOLVED", "chatbot_turn_trace")),
        "planner_request_observed": _contains_any(
            combined,
            ("PLANNER_REQUEST", "/planner/request", "planner_request"),
        ),
        "execution_feedback_observed": _contains_any(
            combined,
            ("execution_feedback", "/planner/execution_feedback", "step_succeeded", "step_failed", "plan_completed"),
        ),
        "speech_observed": _contains_any(
            combined,
            ("ROBOT OUTPUT", "DEBUG_SPEECH", "/debug/nao_say/speech", "Robot saying"),
        ),
        "terminal_observed": terminal_observed(combined),
        "clarification_observed": clarification_observed(combined),
        "fallback_markers": fallback_markers(combined),
        "observability_note": (
            "phase booleans are trace breadcrumbs, not pass/fail scoring"
        ),
    }


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
        "kb_service_timeout": r"kb service timeout|KnowledgeCore .*timeout|service call timed out",
        "report_result_fallback": r"report_result fallback|execution report chatbot returned error|execution report chatbot request failed",
        "route_repair": r"llm_response_route_repair|route_conflict",
        "complete_context_clarification": r"asked for clarification despite complete fixture context",
        "language_model_unreachable_speech": r"having trouble reaching my language model",
    }
    counts = {
        name: len(re.findall(pattern, text, flags=re.IGNORECASE))
        for name, pattern in patterns.items()
    }
    counts["total"] = sum(counts.values())
    return counts


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
        ),
    )


def assess_case(
    case: ProbeCase,
    *,
    observations: dict[str, object],
    stale_world_guard: dict | None,
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

    if expected == "execute_no_clarification":
        if clarified:
            status = "fail" if case.all_required_context else max_status(status, "degraded")
            reasons.append("asked for clarification despite complete fixture context")
        if not planner_seen or not exec_seen:
            status = max_status(status, "fail")
            reasons.append("expected planner request and execution feedback")
        if not terminal or not speech:
            status = max_status(status, "degraded")
            reasons.append("missing terminal or speech evidence")
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
        if severe_fallbacks:
            status = max_status(status, "fail")
    else:
        if not speech:
            status = max_status(status, "degraded")
            reasons.append("speech evidence missing")

    if not reasons:
        reasons.append("matched expected trajectory")
    return {
        "status": status,
        "expected_outcome": expected,
        "all_required_context": case.all_required_context,
        "reasons": reasons,
    }


def max_status(current: str, candidate: str) -> str:
    order = {"pass": 0, "degraded": 1, "fail": 2, "not_scored": 3}
    return candidate if order.get(candidate, 0) > order.get(current, 0) else current


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
        result_entry["wait_sec"] = min(max(0.0, float(wait_sec or 0.0)), elapsed)
        result_entry["log_excerpt"] = log_excerpt
        result_entry["phase_observations"] = phase_observations(
            mode=mode,
            turn_result=turn_result,
            log_excerpt=log_excerpt,
            topic_samples={},
        )
        result_entry["case_assessment"] = assess_case(
            case,
            observations=result_entry["phase_observations"],
            stale_world_guard=result_entry.get("stale_world_guard"),
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
        if observations.get("terminal_observed") and observations.get("speech_observed"):
            break
        if observations.get("clarification_observed") and observations.get("speech_observed"):
            break


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
    return {
        "chatbot_turn_pipeline_mode": active_mode,
        "grounded_context_digest_enabled": grounded_context_digest_enabled,
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


def get_ros_param(container: str, node_name: str, param_name: str) -> str:
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
        if text.startswith("String value is:"):
            return text.split(":", 1)[1].strip()
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
	timeout 18 ros2 topic pub -r 2 {VOICE_TRACKED_QOS} {VOICE_TRACKED_TOPIC} hri_msgs/msg/IdsList "{{ids: ['{voice_id}']}}" >/tmp/nao_questionnaire_voice.log 2>&1 &
	tracked_pub_pid=$!
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
	kill "$tracked_pub_pid" >/dev/null 2>&1 || true
	wait "$tracked_pub_pid" >/dev/null 2>&1 || true
cat /tmp/nao_questionnaire_qos_contract.log /tmp/nao_questionnaire_rqt_rosout.log /tmp/nao_questionnaire_rqt_caption.log /tmp/nao_questionnaire_voice.log /tmp/nao_questionnaire_tracked_info.log /tmp/nao_questionnaire_speech_info.log /tmp/nao_questionnaire_speech.log 2>/dev/null || true
"""
    return run(["docker", "exec", container, "bash", "-lc", script], timeout=45, check=False)


def inject_kb_probe(container: str, injection: KbInjection, *, lifespan_sec: int) -> dict[str, str]:
    statements_yaml = "\n".join("  - '%s'" % item for item in injection.statements)
    patterns_yaml = "\n".join("  - '%s'" % item for item in injection.query_patterns)
    vars_yaml = "\n".join("  - '%s'" % item for item in injection.query_vars)
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
    query_request = f"""
patterns:
{patterns_yaml}
vars:
{vars_yaml}
models:
  - default
"""
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
    query_output = call_ros_service(
        container,
        "/kb/query",
        "kb_msgs/srv/Query",
        query_request,
        timeout_sec=20,
    )
    return {
        "object_id": injection.object_id,
        "service_probe": service_probe,
        "revise_output": revise_output,
        "query_output": query_output,
    }


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
    return f"{VOICE_SPEECH_PREFIX}/{clean_voice}/speech"


if __name__ == "__main__":
    raise SystemExit(main())
