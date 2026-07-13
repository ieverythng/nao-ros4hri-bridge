"""Read-only adapter over a canonical AB registry payload."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from ab_harness.contracts import ABObjectView


class RegistrySnapshot:
    def __init__(self, objects: tuple[ABObjectView, ...], *, source: str, version: str) -> None:
        self.objects = objects
        self.source = source
        self.version = version
        self._by_id = {item.object_id: item for item in objects}

    def get(self, object_id: str) -> ABObjectView | None:
        return self._by_id.get(object_id)

    @classmethod
    def from_json_file(cls, path: str | Path) -> 'RegistrySnapshot':
        registry_path = Path(path)
        raw = registry_path.read_bytes()
        payload = json.loads(raw)
        objects = tuple(_object_view(item) for item in payload.get('objects', ()))
        version = 'sha256:' + hashlib.sha256(raw).hexdigest()
        return cls(objects, source=str(registry_path), version=version)


def _object_view(payload: dict) -> ABObjectView:
    metadata = payload.get('metadata', {}) if isinstance(payload.get('metadata'), dict) else {}
    return ABObjectView(
        object_id=str(payload.get('object_id', '')).strip(),
        ab_level=int(payload.get('ab_level', 0)),
        kind=str(payload.get('kind', '')).strip(),
        category=str(payload.get('category', '')).strip(),
        owner_package=str(payload.get('owner_package', '')).strip(),
        expected_effects=tuple(str(item) for item in payload.get('expected_effects', ())),
        observable_success=tuple(str(item) for item in payload.get('observable_success', ())),
        decomposes_to=tuple(str(item) for item in payload.get('decomposes_to', ())),
        runtime_callable=bool(metadata.get('runtime_callable', False)),
    )
