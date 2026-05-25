"""Action and node health helpers for dashboard status panels."""

from __future__ import annotations


def action_health(*, expected_actions: list[str], discovered_actions: set[str]) -> list[dict]:
    rows: list[dict] = []
    expected_set = {str(item).strip() for item in expected_actions if str(item).strip()}
    for name in sorted(expected_set):
        rows.append(
            {
                'action_name': name,
                'available': name in discovered_actions,
                'status': 'online' if name in discovered_actions else 'missing',
            }
        )
    for extra in sorted(discovered_actions - expected_set):
        rows.append(
            {
                'action_name': extra,
                'available': True,
                'status': 'discovered',
            }
        )
    return rows
