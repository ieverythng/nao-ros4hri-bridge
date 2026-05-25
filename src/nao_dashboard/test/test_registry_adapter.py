import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nao_dashboard import registry_adapter


class _Obj:
    def __init__(self, payload):
        self._payload = dict(payload)

    def to_dict(self):
        return dict(self._payload)


class _Registry:
    def __init__(self, objects):
        self._objects = list(objects)

    def objects(self):
        return list(self._objects)


def test_build_registry_snapshot_with_skill_common_unavailable(monkeypatch) -> None:
    monkeypatch.setattr(registry_adapter, 'load_default_ab_registry', None)

    snapshot = registry_adapter.build_registry_snapshot()

    assert snapshot.objects == []
    assert snapshot.edges == []
    assert snapshot.validation_errors
    assert 'skill_common unavailable' in snapshot.validation_errors[0]


def test_build_registry_snapshot_extracts_decomposition_edges(monkeypatch) -> None:
    registry = _Registry(
        [
            _Obj({'object_id': 'scan', 'ab_level': 1, 'decomposes_to': ['look_at']}),
            _Obj({'object_id': 'macro_scan_and_report', 'ab_level': 2, 'decomposes_to': ['scan', 'report_result']}),
        ]
    )
    monkeypatch.setattr(registry_adapter, 'load_default_ab_registry', lambda: registry)

    snapshot = registry_adapter.build_registry_snapshot()

    assert len(snapshot.objects) == 2
    assert snapshot.edges == [
        {'from': 'macro_scan_and_report', 'to': 'scan', 'kind': 'decomposes_to'},
        {'from': 'macro_scan_and_report', 'to': 'report_result', 'kind': 'decomposes_to'},
    ]
    assert snapshot.validation_errors == []
