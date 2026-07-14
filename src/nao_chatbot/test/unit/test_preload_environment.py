import json
from pathlib import Path
import xml.etree.ElementTree as ET

from nao_chatbot.environment_fixtures import validate_environment_fixture


FIXTURE_PATH = Path(__file__).parents[2] / 'config' / 'preloaded_environments.json'
SVG_PATH = Path(__file__).parents[2] / 'config' / 'preloaded_environment_svgs'
INKSCAPE_LABEL = '{http://www.inkscape.org/namespaces/inkscape}label'


def test_packaged_environment_fixtures_have_explicit_robot_and_person_locations() -> None:
    fixtures = json.loads(FIXTURE_PATH.read_text(encoding='utf-8'))['environments']

    failures = {}
    for fixture_id, fixture in fixtures.items():
        errors = validate_environment_fixture(fixture_id, fixture)
        if errors:
            failures[fixture_id] = errors

    assert failures == {}


def test_packaged_svgs_follow_rqt_human_radar_environment_contract() -> None:
    for path in sorted(SVG_PATH.glob('*.svg')):
        root = ET.parse(path).getroot()
        groups = {
            group.get(INKSCAPE_LABEL): group
            for group in root.findall('{http://www.w3.org/2000/svg}g')
        }
        assert {'walls', 'zones', 'static_objects'} <= groups.keys(), path.name
        static_objects = list(groups['static_objects'])
        assert static_objects, path.name
        assert all(item.get('id') and item.get(INKSCAPE_LABEL) for item in static_objects)
        view_box = [float(value) for value in root.get('viewBox', '').split()]
        assert len(view_box) == 4 and view_box[2] >= 5000 and view_box[3] >= 3000


def test_fixture_validation_rejects_unlocated_person_and_robot() -> None:
    errors = validate_environment_fixture(
        'broken_scene',
        {
            'statements': [
                'person_1 rdf:type Human',
                'person_1 dbp:name ALEX',
            ]
        },
    )

    assert 'fixture has no explicit robot location' in errors
    assert 'person person_1 has no explicit location' in errors


def test_fixture_validation_rejects_person_without_reciprocal_contains() -> None:
    errors = validate_environment_fixture(
        'broken_scene',
        {
            'statements': [
                'myself oro:isAt robot_station',
                'robot_station rdf:type Place',
                'person_1 rdf:type Human',
                'person_1 oro:isIn handoff_area',
                'handoff_area rdf:type Place',
            ]
        },
    )

    assert 'person person_1 location handoff_area lacks reciprocal contains' in errors
