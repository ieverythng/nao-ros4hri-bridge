#!/usr/bin/env python3
"""Inject ROS4HRI speech turns and collect runtime evidence per case."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONTAINER = "nao_ros2"
VOICE_ID = "anonymous_speaker"
VOICE_TRACKED_TOPIC = "/nao_chatbot/humans/voices/tracked"
VOICE_SPEECH_TOPIC = "/nao_chatbot/humans/voices/anonymous_speaker/speech"
VOICE_TRACKED_QOS = "--qos-reliability reliable --qos-durability transient_local"
VOICE_SPEECH_QOS = "--qos-reliability reliable --qos-durability volatile"
RQT_DISPLAY_ROSOUT_TOPIC = "/rosout"
RQT_DISPLAY_CAPTIONS_TOPIC = "/dialogue_manager/closed_captions"
RQT_DISPLAY_ROSOUT_QOS = "--qos-reliability reliable --qos-durability transient_local"
TOPIC_SAMPLE_TIMEOUT_SEC = 1.5
TOPIC_SAMPLE_KILL_AFTER_SEC = 1.0
DEFAULT_GLOBAL_TIMEOUT_SEC = 420
DEFAULT_KB_LIFESPAN_SEC = 300
KB_PROBE_OBJECT_ID = "codex_probe_cup"
KB_MAXIMAL_CUP_ID = "codex_kitchen_cup"
KB_MAXIMAL_LOCATION_ID = "codex_kitchen"
KB_MAXIMAL_ORIGIN_ID = "codex_operator_station"
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
class ProbeCase:
    name: str
    category: str
    text: str
    wait_sec: float = 10.0
    mode: str = "speech"
    setup: KbInjection | None = None
    conversation_group: str | None = None


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
        "I am at the operator station. There is a cup in the kitchen. Go to the kitchen, pick up the cup, bring it back to me at the operator station, and report what happened.",
        160.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_ORIGIN_ID} rdf:type Place",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:name operator_station",
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
        "replan_absent_then_scan_report",
        "replan_recovery",
        "Find the codex missing cup. If you cannot find it, scan the scene and report what you can confirm.",
        150.0,
    ),
    ProbeCase(
        "replan_kitchen_cup_fallback_report",
        "replan_recovery",
        "Bring me the kitchen cup at the operator station. If you cannot bring it, look at the kitchen cup and report the reason.",
        150.0,
        setup=KbInjection(
            object_id=KB_MAXIMAL_CUP_ID,
            statements=(
                f"{KB_MAXIMAL_ORIGIN_ID} rdf:type Place",
                f"{KB_MAXIMAL_ORIGIN_ID} dbp:name operator_station",
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

TOPICS_TO_SAMPLE = (
    "/chatbot_llm/turn_trace",
    "/planner/request",
    "/planner/execution_feedback",
    "/nao_orchestrator/planner_dialogue_act",
    "/debug/nao_say/speech",
    "/speech",
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default=DEFAULT_CONTAINER)
    parser.add_argument("--case-set", default="smoke", choices=("smoke", "main", "composite"))
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
        "--no-rqt-display-mirror",
        action="store_true",
        help="Do not mirror injected user turns to rqt-visible debug topics.",
    )
    args = parser.parse_args()

    case_sets = {
        "smoke": SMOKE_CASES,
        "main": MAIN_QUESTIONNAIRE_CASES,
        "composite": COMPOSITE_CASES,
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
            write_payload(args.out, args.container, args.case_set, started_at, results)
            return 2

        case_start = time.time()
        setup_result = None
        if case.setup is not None:
            setup_result = inject_kb_probe(args.container, case.setup, lifespan_sec=args.kb_lifespan_sec)

        mode = args.mode or case.mode
        conversation_group = case.conversation_group or case.name or f"case_{index}"
        voice_id = VOICE_ID if mode == "speech" else _voice_id_for_group(conversation_group, index)
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
        time.sleep(max(0.0, case.wait_sec))
        results.append(
            {
                "name": case.name,
                "category": case.category,
                "text": case.text,
                "voice_id": voice_id,
                "mode": mode,
                "setup_result": setup_result,
                "turn_result": turn_result,
                "started_at_unix_sec": case_start,
                "wait_sec": case.wait_sec,
                "injection_scope": injection_scope(mode),
                "topic_samples": sample_topics(args.container) if args.sample_topics else {},
                "log_excerpt": recent_logs(args.container, args.since_sec),
            }
        )
        write_payload(args.out, args.container, args.case_set, started_at, results)

    write_payload(args.out, args.container, args.case_set, started_at, results)
    print(args.out)
    return 0


def parse_csv(value: str) -> set[str]:
    return {item.strip() for item in str(value or "").split(",") if item.strip()}


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


def write_payload(
    out_path: str,
    container: str,
    case_set: str,
    started_at: float,
    results: list[dict],
) -> None:
    payload = {
        "container": container,
        "case_set": case_set,
        "started_at_unix_sec": started_at,
        "finished_at_unix_sec": time.time(),
        "cases": results,
    }
    Path(out_path).write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


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
  contract=publish tracked voice with TRANSIENT_LOCAL durability before LiveSpeech.
  reason=dialogue_manager subscribes to the remapped rqt-chat tracked topic with transient-local QoS.
EOF
{mirror_script}
timeout 8 ros2 topic pub --once -w 1 {VOICE_TRACKED_QOS} {VOICE_TRACKED_TOPIC} hri_msgs/msg/IdsList "{{ids: ['{voice_id}']}}" >/tmp/nao_questionnaire_voice.log 2>&1 || true
for _ in $(seq 1 16); do
  if ros2 topic info -v {voice_topic} 2>/dev/null | grep -q 'Node name: dialogue_manager'; then
    break
  fi
  sleep 0.5
done
ros2 topic info -v {VOICE_TRACKED_TOPIC} >/tmp/nao_questionnaire_tracked_info.log 2>&1 || true
ros2 topic info -v {voice_topic} >/tmp/nao_questionnaire_speech_info.log 2>&1 || true
if ! grep -q 'Node name: dialogue_manager' /tmp/nao_questionnaire_speech_info.log 2>/dev/null; then
  echo "[runtime-review] WARNING: dialogue_manager speech subscription was not visible for {voice_topic}" >>/tmp/nao_questionnaire_qos_contract.log
fi
timeout 8 ros2 topic pub --once -w 1 {VOICE_SPEECH_QOS} {voice_topic} hri_msgs/msg/LiveSpeech "{{final: \\"{escaped_text}\\", confidence: 1.0, locale: \\"en_US\\"}}" >/tmp/nao_questionnaire_speech.log 2>&1 || true
cat /tmp/nao_questionnaire_qos_contract.log /tmp/nao_questionnaire_rqt_rosout.log /tmp/nao_questionnaire_rqt_caption.log /tmp/nao_questionnaire_voice.log /tmp/nao_questionnaire_tracked_info.log /tmp/nao_questionnaire_speech_info.log /tmp/nao_questionnaire_speech.log 2>/dev/null || true
"""
    return run(["docker", "exec", container, "bash", "-lc", script], timeout=20, check=False)


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


def _voice_speech_topic(voice_id: str) -> str:
    if voice_id == VOICE_ID:
        return VOICE_SPEECH_TOPIC
    return f"/nao_chatbot/humans/voices/{voice_id}/speech"


if __name__ == "__main__":
    raise SystemExit(main())
