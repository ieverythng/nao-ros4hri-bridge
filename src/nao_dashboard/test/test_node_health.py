import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nao_dashboard.node_health import action_health


def test_action_health_marks_expected_and_discovered_rows() -> None:
    rows = action_health(
        expected_actions=['/skill/scan', '/skill/say'],
        discovered_actions={'/skill/scan', '/skill/fake/navigate_to'},
    )

    assert rows == [
        {'action_name': '/skill/say', 'available': False, 'status': 'missing'},
        {'action_name': '/skill/scan', 'available': True, 'status': 'online'},
        {'action_name': '/skill/fake/navigate_to', 'available': True, 'status': 'discovered'},
    ]
