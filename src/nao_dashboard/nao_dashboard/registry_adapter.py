"""AB registry projection helpers for dashboard views."""

from __future__ import annotations

import time

from nao_dashboard.models import ABRegistrySnapshot

try:  # pragma: no cover - runtime dependency
    from skill_common import load_default_ab_registry
except ImportError:  # pragma: no cover - local fallback
    load_default_ab_registry = None


def build_registry_snapshot() -> ABRegistrySnapshot:
    if load_default_ab_registry is None:
        return ABRegistrySnapshot(
            timestamp=time.time(),
            objects=[],
            edges=[],
            validation_errors=['skill_common unavailable in current environment'],
        )

    try:
        registry = load_default_ab_registry()
    except Exception as err:
        return ABRegistrySnapshot(
            timestamp=time.time(),
            objects=[],
            edges=[],
            validation_errors=['failed to load AB registry: %s' % err],
        )

    objects = []
    edges = []
    validation_errors: list[str] = []

    for item in registry.objects():
        object_payload = item.to_dict()
        objects.append(object_payload)

        if int(object_payload.get('ab_level', 0) or 0) >= 2:
            targets = object_payload.get('decomposes_to', [])
            if isinstance(targets, (list, tuple)):
                for target in targets:
                    clean_target = str(target).strip()
                    if not clean_target:
                        continue
                    edges.append(
                        {
                            'from': str(object_payload.get('object_id', '')).strip(),
                            'to': clean_target,
                            'kind': 'decomposes_to',
                        }
                    )

    return ABRegistrySnapshot(
        timestamp=time.time(),
        objects=objects,
        edges=edges,
        validation_errors=validation_errors,
    )
