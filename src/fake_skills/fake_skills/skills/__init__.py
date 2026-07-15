"""Skill executors used by the fake skill engine."""

from fake_skills.skills.bring_object import execute as execute_bring_object
from fake_skills.skills.find_object import execute as execute_find_object
from fake_skills.skills.inspect_area import execute as execute_inspect_area
from fake_skills.skills.look_at import execute as execute_look_at
from fake_skills.skills.navigate_to import execute as execute_navigate_to
from fake_skills.skills.perform_motion import execute as execute_perform_motion
from fake_skills.skills.pick_object import execute as execute_pick_object
from fake_skills.skills.place_object import execute as execute_place_object
from fake_skills.skills.wave_greet import execute as execute_wave_greet
from fake_skills.skills.walk_to import execute as execute_walk_to


SKILL_EXECUTORS = {
    'bring_object': execute_bring_object,
    'find_object': execute_find_object,
    'inspect_area': execute_inspect_area,
    'look_at': execute_look_at,
    'navigate_to': execute_navigate_to,
    'perform_motion': execute_perform_motion,
    'pick_object': execute_pick_object,
    'place_object': execute_place_object,
    'walk_to': execute_walk_to,
    'wave_greet': execute_wave_greet,
}
