#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import re
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - pre-commit env should provide PyYAML
    yaml = None


REPO_ROOT = Path(__file__).resolve().parent.parent
SKILL_COMMON_ROOT = REPO_ROOT / "src/Neural-Wokbench/src/skill_common/skill_common/defaults"

AB_REGISTRY_PATH = SKILL_COMMON_ROOT / "ab_registry.json"
SKILL_REGISTRY_YAML_PATH = SKILL_COMMON_ROOT / "skill_registry.yaml"
PLANNER_REGISTRY_PATH = REPO_ROOT / "src/planner_llm/config/skill_registry.json"
DOCS_AB_REGISTRY_PATH = REPO_ROOT / "docs/architecture/ab_registry_input.json"
NW_DOCS_AB_REGISTRY_PATH = (
    REPO_ROOT / "src/Neural-Wokbench/docs/neural_workbench/data/ab_registry_input.json"
)
INTERACTIVE_ARCHITECTURE_HTML_PATH = (
    REPO_ROOT / "docs/architecture/ros4hri_neural_workbench_interactive_architecture.html"
)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_yaml(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required to check skill registry consistency")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return dict(payload or {})


def _norm_list(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        values = value
    else:
        values = [value]
    return [str(item).strip() for item in values if str(item).strip()]


def _skill_projection_from_ab(ab_payload: dict) -> dict[str, dict]:
    projection: dict[str, dict] = {}
    for item in ab_payload.get("objects", []):
        if not isinstance(item, dict):
            continue
        if int(item.get("ab_level", 0)) < 1:
            continue
        if str(item.get("kind", "")).strip().lower() not in {"skill", "dialogue_act"}:
            continue
        name = str(item.get("object_id", "")).strip().lower()
        if not name:
            continue
        projection[name] = item
    return projection


def _runtime_callable_from_ab(ab_item: dict) -> bool:
    metadata = ab_item.get("metadata", {})
    if isinstance(metadata, dict) and "runtime_callable" in metadata:
        return bool(metadata.get("runtime_callable"))
    return True


def _interactive_registry_payload(canonical: dict) -> list[dict]:
    records: list[dict] = []
    for item in canonical.get("objects", []):
        if not isinstance(item, dict):
            continue
        transport = item.get("transport", "")
        if isinstance(transport, dict):
            transport = str(transport.get("type", "")).strip()
        else:
            transport = str(transport or "").strip()
        records.append(
            {
                "id": str(item.get("object_id", "")).strip(),
                "ab": int(item.get("ab_level", 0) or 0),
                "kind": str(item.get("kind", "")).strip(),
                "category": str(item.get("category", "")).strip(),
                "owner": str(item.get("owner_package", "")).strip(),
                "status": str(item.get("implementation_status", "")).strip(),
                "mapping": str(item.get("robot_adapter_mapping", "")).strip(),
                "transport": transport,
                "safety": ", ".join(_norm_list(item.get("safety_flags", []))),
                "effects": "; ".join(_norm_list(item.get("expected_effects", []))),
            }
        )
    return sorted(records, key=lambda record: (record["ab"], record["id"]))


def _interactive_html_text_with_registry(current_text: str, registry_records: list[dict]) -> str:
    registry_js = "const REGISTRY=" + json.dumps(registry_records, ensure_ascii=False) + ";"
    pattern = re.compile(r"const REGISTRY=.*?;\nlet mode=", flags=re.DOTALL)
    replacement = registry_js + "\nlet mode="
    updated, count = pattern.subn(replacement, current_text, count=1)
    if count != 1:
        raise RuntimeError(
            f"Could not locate REGISTRY block in {INTERACTIVE_ARCHITECTURE_HTML_PATH}"
        )
    return updated


def _assert_equal(errors: list[str], context: str, left, right) -> None:
    if left != right:
        errors.append("%s mismatch | expected=%r actual=%r" % (context, left, right))


def _compare_skill_yaml_against_ab(
    *,
    errors: list[str],
    skill_yaml_payload: dict,
    ab_projection: dict[str, dict],
) -> None:
    yaml_skills = skill_yaml_payload.get("skills", [])
    if not isinstance(yaml_skills, list):
        errors.append("skill_registry.yaml is missing a valid top-level `skills` list")
        return

    yaml_map: dict[str, dict] = {}
    for item in yaml_skills:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name", "")).strip().lower()
        if not name:
            continue
        yaml_map[name] = item

    for name, yaml_item in sorted(yaml_map.items()):
        ab_item = ab_projection.get(name)
        if ab_item is None:
            errors.append("skill_registry.yaml skill `%s` is missing from ab_registry.json" % name)
            continue

        _assert_equal(
            errors,
            "skill_registry.yaml:%s.abstraction_level" % name,
            int(ab_item.get("ab_level", 0)),
            int(yaml_item.get("abstraction_level", 0)),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.category" % name,
            str(ab_item.get("category", "")).strip(),
            str(yaml_item.get("category", "")).strip(),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.aliases" % name,
            _norm_list(ab_item.get("aliases", [])),
            _norm_list(yaml_item.get("aliases", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.params" % name,
            _norm_list(ab_item.get("params", [])),
            _norm_list(yaml_item.get("params", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.required_params" % name,
            _norm_list(ab_item.get("required_params", [])),
            _norm_list(yaml_item.get("required_params", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.preconditions" % name,
            _norm_list(ab_item.get("preconditions", [])),
            _norm_list(yaml_item.get("preconditions", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.expected_effects" % name,
            _norm_list(ab_item.get("expected_effects", [])),
            _norm_list(yaml_item.get("expected_effects", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.observable_success" % name,
            _norm_list(ab_item.get("observable_success", [])),
            _norm_list(yaml_item.get("observable_success", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.failure_modes" % name,
            _norm_list(ab_item.get("failure_modes", [])),
            _norm_list(yaml_item.get("failure_modes", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.planner_guidance" % name,
            _norm_list(ab_item.get("planner_guidance", [])),
            _norm_list(yaml_item.get("planner_guidance", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.robot_adapter_mapping" % name,
            str(ab_item.get("robot_adapter_mapping", "")).strip(),
            str(yaml_item.get("robot_adapter_mapping", "")).strip(),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.safety_flags" % name,
            _norm_list(ab_item.get("safety_flags", [])),
            _norm_list(yaml_item.get("safety_flags", [])),
        )
        _assert_equal(
            errors,
            "skill_registry.yaml:%s.result_schema" % name,
            dict(ab_item.get("result_schema", {})),
            dict(yaml_item.get("result_schema", {})),
        )

    missing_yaml_names = sorted(set(ab_projection) - set(yaml_map))
    for name in missing_yaml_names:
        errors.append("skill_registry.yaml is missing canonical skill `%s`" % name)


def _compare_planner_registry_against_ab(
    *,
    errors: list[str],
    planner_payload: dict,
    ab_projection: dict[str, dict],
) -> None:
    planner_skills = planner_payload.get("skills", [])
    if not isinstance(planner_skills, list):
        errors.append("planner_llm skill_registry.json is missing a valid top-level `skills` list")
        return

    planner_names: set[str] = set()
    for item in planner_skills:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name", "")).strip().lower()
        if not name:
            continue
        planner_names.add(name)
        ab_item = ab_projection.get(name)
        if ab_item is None:
            errors.append("planner skill `%s` is missing from canonical ab_registry.json" % name)
            continue

        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.aliases" % name,
            _norm_list(ab_item.get("aliases", [])),
            _norm_list(item.get("aliases", [])),
        )
        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.category" % name,
            str(ab_item.get("category", "")).strip(),
            str(item.get("category", "")).strip(),
        )
        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.params" % name,
            _norm_list(ab_item.get("params", [])),
            _norm_list(item.get("params", [])),
        )
        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.required_params" % name,
            _norm_list(ab_item.get("required_params", [])),
            _norm_list(item.get("required_params", [])),
        )
        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.robot_adapter_mapping" % name,
            str(ab_item.get("robot_adapter_mapping", "")).strip(),
            str(item.get("robot_adapter_mapping", "")).strip(),
        )
        _assert_equal(
            errors,
            "planner_llm.skill_registry:%s.safety_flags" % name,
            _norm_list(ab_item.get("safety_flags", [])),
            _norm_list(item.get("safety_flags", [])),
        )

    expected_runtime_skills = {
        name
        for name, ab_item in ab_projection.items()
        if _runtime_callable_from_ab(ab_item)
    }
    missing_runtime = sorted(expected_runtime_skills - planner_names)
    extra_runtime = sorted(planner_names - expected_runtime_skills)
    if missing_runtime:
        errors.append(
            "planner registry missing runtime-callable skills: %s" % ", ".join(missing_runtime)
        )
    if extra_runtime:
        errors.append(
            "planner registry has non-runtime-callable skills: %s" % ", ".join(extra_runtime)
        )


def _compare_docs_copies(
    *,
    errors: list[str],
    canonical_ab_payload: dict,
    docs_ab_payload: dict,
    nw_docs_ab_payload: dict,
) -> None:
    if canonical_ab_payload != docs_ab_payload:
        errors.append(
            "docs/architecture/ab_registry_input.json drifted from canonical skill_common ab_registry.json"
        )
    if canonical_ab_payload != nw_docs_ab_payload:
        errors.append(
            "Neural-Wokbench docs ab_registry_input.json drifted from canonical skill_common ab_registry.json"
        )


def _validate_runtime_callability_policy(*, errors: list[str], canonical_ab_payload: dict) -> None:
    for item in canonical_ab_payload.get("objects", []):
        if not isinstance(item, dict):
            continue
        object_id = str(item.get("object_id", "")).strip() or "<unnamed>"
        ab_level = int(item.get("ab_level", 0) or 0)
        runtime_callable = _runtime_callable_from_ab(item)
        if ab_level == 0 and runtime_callable:
            errors.append(
                "ab_registry runtime policy violation: AB=0 object marked runtime callable: %s"
                % object_id
            )
        if ab_level >= 2 and runtime_callable:
            errors.append(
                "ab_registry runtime policy violation: AB>=2 proposal marked runtime callable: %s"
                % object_id
            )


def _compare_interactive_html_registry(
    *,
    errors: list[str],
    canonical_ab_payload: dict,
    interactive_html_text: str,
) -> None:
    expected_records = _interactive_registry_payload(canonical_ab_payload)
    try:
        expected_text = _interactive_html_text_with_registry(
            interactive_html_text, expected_records
        )
    except RuntimeError as err:
        errors.append(str(err))
        return
    if interactive_html_text != expected_text:
        errors.append(
            "docs/architecture/ros4hri_neural_workbench_interactive_architecture.html "
            "REGISTRY block drifted from canonical skill_common ab_registry.json"
        )


def main() -> int:
    errors: list[str] = []

    canonical_ab_payload = _load_json(AB_REGISTRY_PATH)
    skill_yaml_payload = _load_yaml(SKILL_REGISTRY_YAML_PATH)
    planner_payload = _load_json(PLANNER_REGISTRY_PATH)
    docs_ab_payload = _load_json(DOCS_AB_REGISTRY_PATH)
    nw_docs_ab_payload = _load_json(NW_DOCS_AB_REGISTRY_PATH)
    interactive_html_text = INTERACTIVE_ARCHITECTURE_HTML_PATH.read_text(encoding="utf-8")

    ab_projection = _skill_projection_from_ab(canonical_ab_payload)

    _compare_skill_yaml_against_ab(
        errors=errors,
        skill_yaml_payload=skill_yaml_payload,
        ab_projection=ab_projection,
    )
    _compare_planner_registry_against_ab(
        errors=errors,
        planner_payload=planner_payload,
        ab_projection=ab_projection,
    )
    _compare_docs_copies(
        errors=errors,
        canonical_ab_payload=canonical_ab_payload,
        docs_ab_payload=docs_ab_payload,
        nw_docs_ab_payload=nw_docs_ab_payload,
    )
    _validate_runtime_callability_policy(errors=errors, canonical_ab_payload=canonical_ab_payload)
    _compare_interactive_html_registry(
        errors=errors,
        canonical_ab_payload=canonical_ab_payload,
        interactive_html_text=interactive_html_text,
    )

    if errors:
        print("Skill/AB registry consistency check failed:")
        for item in errors:
            print("- %s" % item)
        return 1

    print("Skill/AB registry consistency check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
