#!/usr/bin/env python3
"""Sync planner/docs skill registry views from canonical AB registry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


REPO_ROOT = Path(__file__).resolve().parent.parent
SKILL_COMMON_ROOT = REPO_ROOT / 'src/Neural-Wokbench/src/skill_common/skill_common/defaults'
AB_REGISTRY_PATH = SKILL_COMMON_ROOT / 'ab_registry.json'
SKILL_REGISTRY_YAML_PATH = SKILL_COMMON_ROOT / 'skill_registry.yaml'
PLANNER_REGISTRY_PATH = REPO_ROOT / 'src/planner_llm/config/skill_registry.json'
DOCS_AB_REGISTRY_PATH = REPO_ROOT / 'docs/architecture/ab_registry_input.json'
NW_DOCS_AB_REGISTRY_PATH = (
    REPO_ROOT / 'src/Neural-Wokbench/docs/neural_workbench/data/ab_registry_input.json'
)
INTERACTIVE_ARCHITECTURE_HTML_PATH = (
    REPO_ROOT / 'docs/architecture/ros4hri_neural_workbench_interactive_architecture.html'
)

DEFAULT_STEP_TYPES = ['noop', 'say', 'skill', 'look_at']


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _clean_list(value) -> list[str]:
    if isinstance(value, str):
        value = [value]
    if not isinstance(value, (list, tuple)):
        return []
    return [str(item).strip() for item in value if str(item).strip()]


def _runtime_callable(item: dict) -> bool:
    metadata = item.get('metadata', {})
    if isinstance(metadata, dict) and 'runtime_callable' in metadata:
        return bool(metadata.get('runtime_callable'))
    return int(item.get('ab_level', 0) or 0) >= 1 and str(item.get('kind', '')).strip().lower() in {
        'skill',
        'dialogue_act',
    }


def _is_skill_like(item: dict) -> bool:
    return int(item.get('ab_level', 0) or 0) >= 1 and str(item.get('kind', '')).strip().lower() in {
        'skill',
        'dialogue_act',
    }


def _skill_payload(item: dict) -> dict:
    metadata = item.get('metadata', {}) if isinstance(item.get('metadata', {}), dict) else {}
    decomposition = metadata.get('decomposition', {}) if isinstance(metadata.get('decomposition', {}), dict) else {}
    payload = {
        'name': str(item.get('object_id', '')).strip(),
        'category': str(item.get('category', '')).strip(),
        'aliases': _clean_list(item.get('aliases', [])),
        'abstraction_level': int(item.get('ab_level', 0) or 0),
        'params': _clean_list(item.get('params', [])),
        'required_params': _clean_list(item.get('required_params', [])),
        'preconditions': _clean_list(item.get('preconditions', [])),
        'expected_effects': _clean_list(item.get('expected_effects', [])),
        'observable_success': _clean_list(item.get('observable_success', [])),
        'failure_modes': _clean_list(item.get('failure_modes', [])),
        'planner_guidance': _clean_list(item.get('planner_guidance', [])),
        'robot_adapter_mapping': str(item.get('robot_adapter_mapping', '')).strip(),
        'result_schema': dict(item.get('result_schema', {}) or {}),
        'safety_flags': _clean_list(item.get('safety_flags', [])),
        'supports_failure_injection': bool(item.get('supports_failure_injection', False)),
        'is_composite': bool(item.get('is_composite', False)),
        'is_fake': bool(item.get('is_fake', False)),
        'runtime_callable': _runtime_callable(item),
        'decomposes_to': _clean_list(item.get('decomposes_to', [])),
        'decomposition': {
            'max_depth': int(decomposition.get('max_depth', -1)),
            'policy': str(decomposition.get('policy', '')).strip(),
            'notes': str(decomposition.get('notes', '')).strip(),
        },
    }
    return payload


def _interactive_registry_payload(canonical: dict) -> list[dict]:
    objects = [item for item in canonical.get('objects', []) if isinstance(item, dict)]
    records: list[dict] = []
    for item in objects:
        transport = item.get('transport', '')
        if isinstance(transport, dict):
            transport = str(transport.get('type', '')).strip()
        else:
            transport = str(transport or '').strip()
        records.append(
            {
                'id': str(item.get('object_id', '')).strip(),
                'ab': int(item.get('ab_level', 0) or 0),
                'kind': str(item.get('kind', '')).strip(),
                'category': str(item.get('category', '')).strip(),
                'owner': str(item.get('owner_package', '')).strip(),
                'status': str(item.get('implementation_status', '')).strip(),
                'mapping': str(item.get('robot_adapter_mapping', '')).strip(),
                'transport': transport,
                'safety': ', '.join(_clean_list(item.get('safety_flags', []))),
                'effects': '; '.join(_clean_list(item.get('expected_effects', []))),
            }
        )
    return sorted(records, key=lambda record: (record['ab'], record['id']))


def build_views(canonical: dict) -> tuple[dict, dict]:
    objects = [item for item in canonical.get('objects', []) if isinstance(item, dict)]
    skill_items = sorted(
        (_skill_payload(item) for item in objects if _is_skill_like(item)),
        key=lambda item: item['name'],
    )
    runtime_skill_items = [item for item in skill_items if bool(item.get('runtime_callable', False))]

    skill_yaml_payload = {'skills': skill_items}
    planner_payload = {
        'step_types': list(DEFAULT_STEP_TYPES),
        'skills': runtime_skill_items,
    }
    return skill_yaml_payload, planner_payload


def _emit_yaml(payload: dict) -> str:
    if yaml is None:
        raise RuntimeError('PyYAML is required to emit skill_registry.yaml')
    return yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)


def _compare_or_write(path: Path, expected_text: str, *, write: bool, errors: list[str]) -> None:
    current_text = path.read_text(encoding='utf-8') if path.exists() else ''
    if current_text == expected_text:
        return
    if write:
        path.write_text(expected_text, encoding='utf-8')
        return
    errors.append(f'{path} is out of sync with canonical AB registry')


def _interactive_html_text_with_registry(current_text: str, registry_records: list[dict]) -> str:
    registry_js = 'const REGISTRY=' + json.dumps(registry_records, ensure_ascii=False) + ';'
    pattern = re.compile(r'const REGISTRY=.*?;\nlet mode=', flags=re.DOTALL)
    replacement = registry_js + '\nlet mode='
    updated, count = pattern.subn(replacement, current_text, count=1)
    if count != 1:
        raise RuntimeError(
            f'Could not locate REGISTRY block in {INTERACTIVE_ARCHITECTURE_HTML_PATH}'
        )
    return updated


def _compare_or_write_interactive_html(
    *,
    path: Path,
    registry_records: list[dict],
    write: bool,
    errors: list[str],
) -> None:
    current_text = path.read_text(encoding='utf-8') if path.exists() else ''
    try:
        expected_text = _interactive_html_text_with_registry(current_text, registry_records)
    except RuntimeError as err:
        errors.append(str(err))
        return
    if current_text == expected_text:
        return
    if write:
        path.write_text(expected_text, encoding='utf-8')
        return
    errors.append(f'{path} is out of sync with canonical AB registry')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Sync registry views from canonical AB registry')
    parser.add_argument('--check', action='store_true', help='Fail if generated views differ')
    parser.add_argument('--write', action='store_true', help='Write generated views')
    args = parser.parse_args(argv)

    write = bool(args.write or not args.check)

    canonical = _load_json(AB_REGISTRY_PATH)
    skill_yaml_payload, planner_payload = build_views(canonical)
    interactive_registry_records = _interactive_registry_payload(canonical)

    canonical_text = json.dumps(canonical, ensure_ascii=False, indent=2) + '\n'
    skill_yaml_text = _emit_yaml(skill_yaml_payload)
    planner_text = json.dumps(planner_payload, ensure_ascii=False, indent=2) + '\n'

    errors: list[str] = []
    _compare_or_write(SKILL_REGISTRY_YAML_PATH, skill_yaml_text, write=write, errors=errors)
    _compare_or_write(PLANNER_REGISTRY_PATH, planner_text, write=write, errors=errors)
    _compare_or_write(DOCS_AB_REGISTRY_PATH, canonical_text, write=write, errors=errors)
    _compare_or_write(NW_DOCS_AB_REGISTRY_PATH, canonical_text, write=write, errors=errors)
    _compare_or_write_interactive_html(
        path=INTERACTIVE_ARCHITECTURE_HTML_PATH,
        registry_records=interactive_registry_records,
        write=write,
        errors=errors,
    )

    if errors:
        print('Registry sync check failed:')
        for item in errors:
            print('- %s' % item)
        return 1

    if write:
        print('Registry views synchronized from canonical AB registry.')
    else:
        print('Registry views already synchronized.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
