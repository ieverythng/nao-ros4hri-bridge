"""Fixture builder — grounded scene context for planner requests.

Builds deterministic entity profiles that simulate what the scene
perception layer would provide to the planner.
"""

from __future__ import annotations


# ── Scene profiles ─────────────────────────────────────────────────────

PROFILES: dict[str, dict] = {
    "empty_scene": {
        "entities": [],
        "scene_summary": "Empty scene — no detectable objects or persons.",
    },
    "cup_visible": {
        "entities": [
            {
                "id": "table_1",
                "label": "table",
                "kind": "object",
                "class": "Table",
                "visible": True,
                "relations": [],
            },
            {
                "id": "cup_1",
                "label": "cup",
                "kind": "object",
                "class": "Cup",
                "visible": True,
                "relations": [{"predicate": "oro:isOn", "object": "table_1"}],
            },
        ],
        "scene_summary": "One cup visible on table.",
    },
    "two_cups_ambiguous": {
        "entities": [
            {
                "id": "table_1",
                "label": "table",
                "kind": "object",
                "class": "Table",
                "visible": True,
                "relations": [],
            },
            {
                "id": "cup_1",
                "label": "cup",
                "kind": "object",
                "class": "Cup",
                "visible": True,
                "relations": [{"predicate": "oro:isOn", "object": "table_1"}],
            },
            {
                "id": "cup_2",
                "label": "cup",
                "kind": "object",
                "class": "Cup",
                "visible": True,
                "relations": [{"predicate": "oro:isOn", "object": "table_1"}],
            },
        ],
        "scene_summary": "Two cups visible on table — ambiguous target.",
    },
    "person_visible": {
        "entities": [
            {
                "id": "person_1",
                "label": "person",
                "kind": "person",
                "class": "Person",
                "visible": True,
                "relations": [],
            },
        ],
        "scene_summary": "One person visible in scene.",
    },
    "person_and_cup_visible": {
        "entities": [
            {
                "id": "table_1",
                "label": "table",
                "kind": "object",
                "class": "Table",
                "visible": True,
                "relations": [],
            },
            {
                "id": "cup_1",
                "label": "cup",
                "kind": "object",
                "class": "Cup",
                "visible": True,
                "relations": [{"predicate": "oro:isOn", "object": "table_1"}],
            },
            {
                "id": "person_1",
                "label": "person",
                "kind": "person",
                "class": "Person",
                "visible": True,
                "relations": [{"predicate": "oro:near", "object": "table_1"}],
            },
        ],
        "scene_summary": "Person near table with cup.",
    },
    "area_table_clear": {
        "entities": [
            {
                "id": "table_1",
                "label": "table",
                "kind": "object",
                "class": "Table",
                "visible": True,
                "relations": [],
            },
        ],
        "scene_summary": "Area inspected — table clear, no obstacles.",
    },
    "area_table_person": {
        "entities": [
            {
                "id": "table_1",
                "label": "table",
                "kind": "object",
                "class": "Table",
                "visible": True,
                "relations": [],
            },
            {
                "id": "person_1",
                "label": "person",
                "kind": "person",
                "class": "Person",
                "visible": True,
                "relations": [{"predicate": "oro:near", "object": "table_1"}],
            },
        ],
        "scene_summary": "Area inspected — person found near table.",
    },
}


def build_grounded_context(profile: str) -> dict:
    """Return a grounded context dict for the given scene profile."""
    template = PROFILES.get(profile)
    if template is None:
        raise ValueError(f"Unknown profile: {profile}. Available: {list_profiles()}")
    return dict(template)


def list_profiles() -> list[str]:
    """Return available profile names."""
    return sorted(PROFILES.keys())
