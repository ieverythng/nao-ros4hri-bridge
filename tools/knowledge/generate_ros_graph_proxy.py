#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ast
import json
import re
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path


ENDPOINT_MD_PATH = Path("docs/knowledge/ROS_RUNTIME_GRAPH.md")
ENDPOINT_JSON_PATH = Path("docs/knowledge/ros_runtime_graph.json")
ENDPOINT_PROXY_PATH = Path("docs/knowledge/ros_runtime_proxy.py")
OVERRIDES_PATH = Path("tools/knowledge/ros_graph_overrides.json")

SKIP_DIR_NAMES = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "pages",
    "node_modules",
    "test",
    "tests",
}


@dataclass
class PackageRecord:
    name: str
    root: str
    node_names: set[str] = field(default_factory=set)
    namespaces: set[str] = field(default_factory=set)
    publishes: set[str] = field(default_factory=set)
    subscribes: set[str] = field(default_factory=set)
    service_clients: set[str] = field(default_factory=set)
    service_servers: set[str] = field(default_factory=set)
    action_clients: set[str] = field(default_factory=set)
    action_servers: set[str] = field(default_factory=set)
    contracts: set[str] = field(default_factory=set)
    contract_kinds: dict[str, str] = field(default_factory=dict)
    files: set[str] = field(default_factory=set)
    endpoint_rewrites: dict[str, str] = field(default_factory=dict)

    def node_name(self) -> str:
        if self.node_names:
            return sorted(self.node_names)[0]
        return self.name

    def namespace(self) -> str:
        non_empty = [value for value in self.namespaces if value]
        if non_empty:
            return sorted(non_empty)[0].strip("/")
        return ""


@dataclass
class PackageOverride:
    node_name: str | None = None
    namespace: str | None = None
    endpoint_rewrites: dict[str, str] = field(default_factory=dict)


@dataclass
class LaunchMetadata:
    node_names: set[str] = field(default_factory=set)
    namespaces: set[str] = field(default_factory=set)


class PythonRosAnalyzer(ast.NodeVisitor):
    def __init__(self) -> None:
        self.param_defaults: dict[str, str] = {}
        self.symbol_values: dict[str, str] = {}
        self.node_names: set[str] = set()
        self.calls: dict[str, list[str]] = defaultdict(list)

    def visit_Call(self, node: ast.Call) -> None:
        call_name = self._call_name(node.func)
        if call_name == "declare_parameter" and len(node.args) >= 2:
            param_name = self._extract_string(node.args[0])
            default_value = self._resolve_expr(node.args[1])
            if param_name and default_value is not None:
                self.param_defaults[param_name] = default_value
        elif call_name == "create_publisher" and len(node.args) >= 2:
            endpoint = self._resolve_expr(node.args[1])
            if endpoint:
                self.calls["publishes"].append(endpoint)
        elif call_name == "create_subscription" and len(node.args) >= 2:
            endpoint = self._resolve_expr(node.args[1])
            if endpoint:
                self.calls["subscribes"].append(endpoint)
        elif call_name == "create_client" and len(node.args) >= 2:
            endpoint = self._resolve_expr(node.args[1])
            if endpoint:
                self.calls["service_clients"].append(endpoint)
        elif call_name == "create_service" and len(node.args) >= 2:
            endpoint = self._resolve_expr(node.args[1])
            if endpoint:
                self.calls["service_servers"].append(endpoint)
        elif call_name == "ActionClient" and len(node.args) >= 3:
            endpoint = self._resolve_expr(node.args[2])
            if endpoint:
                self.calls["action_clients"].append(endpoint)
        elif call_name == "ActionServer" and len(node.args) >= 3:
            endpoint = self._resolve_expr(node.args[2])
            if endpoint:
                self.calls["action_servers"].append(endpoint)
        elif self._is_super_init(node):
            node_name = self._extract_string(node.args[0]) if node.args else None
            if node_name:
                self.node_names.add(node_name)

        self.generic_visit(node)

    def visit_Assign(self, node: ast.Assign) -> None:
        value = self._resolve_expr(node.value)
        if value is not None:
            for target in node.targets:
                name = self._assignment_target_name(target)
                if name:
                    self.symbol_values[name] = value
        self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> None:
        if node.value is not None:
            value = self._resolve_expr(node.value)
            if value is not None:
                name = self._assignment_target_name(node.target)
                if name:
                    self.symbol_values[name] = value
        self.generic_visit(node)

    def _call_name(self, func: ast.expr) -> str | None:
        if isinstance(func, ast.Name):
            return func.id
        if isinstance(func, ast.Attribute):
            return func.attr
        return None

    def _assignment_target_name(self, target: ast.expr) -> str | None:
        if isinstance(target, ast.Name):
            return target.id
        if (
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "self"
        ):
            return f"self.{target.attr}"
        return None

    def _is_super_init(self, node: ast.Call) -> bool:
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr != "__init__":
            return False
        value = func.value
        return (
            isinstance(value, ast.Call)
            and isinstance(value.func, ast.Name)
            and value.func.id == "super"
        )

    def _extract_string(self, expr: ast.expr) -> str | None:
        if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
            return expr.value
        if isinstance(expr, ast.JoinedStr):
            parts: list[str] = []
            for value in expr.values:
                if isinstance(value, ast.Constant) and isinstance(value.value, str):
                    parts.append(value.value)
                elif isinstance(value, ast.FormattedValue):
                    resolved = self._resolve_expr(value.value)
                    parts.append(resolved if resolved is not None else "*")
                else:
                    parts.append("*")
            return "".join(parts)
        if isinstance(expr, ast.Name):
            return self.symbol_values.get(expr.id)
        if (
            isinstance(expr, ast.Attribute)
            and isinstance(expr.value, ast.Name)
            and expr.value.id == "self"
        ):
            return self.symbol_values.get(f"self.{expr.attr}")
        if isinstance(expr, ast.BinOp) and isinstance(expr.op, ast.Add):
            left = self._extract_string(expr.left)
            right = self._extract_string(expr.right)
            if left is not None and right is not None:
                return f"{left}{right}"
        return None

    def _find_param_name(self, expr: ast.expr) -> str | None:
        if isinstance(expr, ast.Call):
            if (
                isinstance(expr.func, ast.Attribute)
                and expr.func.attr == "get_parameter"
                and expr.args
            ):
                return self._extract_string(expr.args[0])
            if expr.args:
                for arg in expr.args:
                    found = self._find_param_name(arg)
                    if found:
                        return found
            return self._find_param_name(expr.func)
        if isinstance(expr, ast.Attribute):
            return self._find_param_name(expr.value)
        if isinstance(expr, ast.BoolOp):
            for value in expr.values:
                found = self._find_param_name(value)
                if found:
                    return found
        return None

    def _resolve_expr(self, expr: ast.expr) -> str | None:
        literal = self._extract_string(expr)
        if literal is not None:
            return literal

        if isinstance(expr, ast.BoolOp):
            for value in expr.values:
                resolved = self._resolve_expr(value)
                if resolved:
                    return resolved
            return None

        if isinstance(expr, ast.Call):
            param_name = self._find_param_name(expr)
            if param_name and param_name in self.param_defaults:
                return self.param_defaults[param_name]
            if expr.args:
                for arg in expr.args:
                    resolved = self._resolve_expr(arg)
                    if resolved:
                        return resolved
            return self._resolve_expr(expr.func)

        if isinstance(expr, ast.Attribute):
            if (
                isinstance(expr.value, ast.Name)
                and expr.value.id == "self"
                and f"self.{expr.attr}" in self.symbol_values
            ):
                return self.symbol_values[f"self.{expr.attr}"]
            return self._resolve_expr(expr.value)

        return None


class LaunchRosAnalyzer(ast.NodeVisitor):
    def __init__(self) -> None:
        self.symbol_values: dict[str, str] = {}
        self.node_names_by_package: dict[str, set[str]] = defaultdict(set)
        self.namespaces_by_package: dict[str, set[str]] = defaultdict(set)

    def visit_Assign(self, node: ast.Assign) -> None:
        value = self._resolve_expr(node.value)
        if value is not None:
            for target in node.targets:
                if isinstance(target, ast.Name):
                    self.symbol_values[target.id] = value
        self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> None:
        if node.value is not None:
            value = self._resolve_expr(node.value)
            if value is not None and isinstance(node.target, ast.Name):
                self.symbol_values[node.target.id] = value
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:
        call_name = self._call_name(node.func)
        if call_name not in {"Node", "LifecycleNode"}:
            self.generic_visit(node)
            return

        package_name = self._keyword_value(node, "package")
        if not package_name:
            self.generic_visit(node)
            return

        name = self._keyword_value(node, "name")
        namespace = self._keyword_value(node, "namespace")
        if name:
            self.node_names_by_package[package_name].add(name)
        if namespace:
            self.namespaces_by_package[package_name].add(namespace)
        self.generic_visit(node)

    def _keyword_value(self, node: ast.Call, keyword_name: str) -> str | None:
        for keyword in node.keywords:
            if keyword.arg == keyword_name:
                return self._resolve_expr(keyword.value)
        return None

    def _call_name(self, func: ast.expr) -> str | None:
        if isinstance(func, ast.Name):
            return func.id
        if isinstance(func, ast.Attribute):
            return func.attr
        return None

    def _resolve_expr(self, expr: ast.expr) -> str | None:
        if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
            return expr.value
        if isinstance(expr, ast.Name):
            return self.symbol_values.get(expr.id)
        return None


def should_skip(path: Path) -> bool:
    return any(part in SKIP_DIR_NAMES for part in path.parts)


def endpoint_like(value: str) -> bool:
    if not value:
        return False
    if "://" in value:
        return False
    if value.endswith((".py", ".yaml", ".yml", ".json", ".md", ".txt")):
        return False
    if value.startswith("/") or value.startswith("~/") or "/" in value:
        return True
    return bool(re.fullmatch(r"[A-Za-z0-9_*]+", value))


def fully_qualified_private_endpoint(
    endpoint: str,
    *,
    node_name: str | None,
    namespace: str,
) -> str:
    suffix = endpoint.removeprefix("~/").lstrip("/")
    namespace_prefix = f"/{namespace}" if namespace else ""
    if node_name:
        return f"{namespace_prefix}/{node_name}/{suffix}".replace("//", "/")
    return f"{namespace_prefix}/{suffix}".replace("//", "/")


def normalize_endpoint(raw: str, node_name: str | None, namespace: str = "") -> str | None:
    value = raw.strip()
    value = re.sub(r"\{[^}]+\}", "*", value)
    value = value.replace("**", "*")
    if not endpoint_like(value):
        return None
    if value.startswith("~/"):
        return fully_qualified_private_endpoint(
            value,
            node_name=node_name,
            namespace=namespace,
        )
    if value.startswith("/"):
        return re.sub(r"/{2,}", "/", value)
    if "/" not in value and namespace:
        return f"/{namespace}/{value}"
    if "/" not in value:
        return f"/{value}"
    return f"/{value.lstrip('/')}"


def parse_contract_interfaces(package_xml_path: Path) -> list[tuple[str, str]]:
    text = package_xml_path.read_text(encoding="utf-8", errors="ignore")
    matches = re.finditer(
        r"interface:\s*(action|service|topic).*?default_interface_path:\s*([^\s<]+)",
        text,
        re.DOTALL,
    )
    return [(match.group(1).strip(), match.group(2).strip()) for match in matches]


def load_overrides(repo_root: Path) -> dict[str, PackageOverride]:
    path = repo_root / OVERRIDES_PATH
    if not path.exists():
        return {}

    payload = json.loads(path.read_text(encoding="utf-8"))
    packages = payload.get("packages", {})
    overrides: dict[str, PackageOverride] = {}
    for package_name, package_payload in packages.items():
        overrides[package_name] = PackageOverride(
            node_name=package_payload.get("node_name"),
            namespace=package_payload.get("namespace"),
            endpoint_rewrites=dict(package_payload.get("endpoint_rewrites", {})),
        )
    return overrides


def collect_launch_metadata(package_root: Path) -> LaunchMetadata:
    metadata = LaunchMetadata()
    launch_dir = package_root / "launch"
    if not launch_dir.exists():
        return metadata

    for path in sorted(launch_dir.rglob("*.py")):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="ignore"))
        except SyntaxError:
            continue

        analyzer = LaunchRosAnalyzer()
        analyzer.visit(tree)
        metadata.node_names.update(analyzer.node_names_by_package.get(package_root.name, set()))
        metadata.namespaces.update(analyzer.namespaces_by_package.get(package_root.name, set()))
    return metadata


def apply_endpoint_rewrite(record: PackageRecord, endpoint: str | None) -> str | None:
    if endpoint is None:
        return None
    return record.endpoint_rewrites.get(endpoint, endpoint)


def analyze_package(
    repo_root: Path,
    package_root: Path,
    overrides: dict[str, PackageOverride],
) -> PackageRecord:
    record = PackageRecord(
        name=package_root.name,
        root=str(package_root.relative_to(repo_root)),
    )
    launch_metadata = collect_launch_metadata(package_root)
    record.node_names.update(launch_metadata.node_names)
    record.namespaces.update(launch_metadata.namespaces)

    override = overrides.get(package_root.name)
    if override is not None:
        if override.node_name:
            record.node_names.add(override.node_name)
        if override.namespace:
            record.namespaces.add(override.namespace)
        record.endpoint_rewrites.update(override.endpoint_rewrites)

    for package_xml in package_root.glob("package.xml"):
        for kind, contract in parse_contract_interfaces(package_xml):
            normalized = normalize_endpoint(
                contract,
                record.node_name(),
                record.namespace(),
            )
            normalized = apply_endpoint_rewrite(record, normalized)
            if normalized:
                record.contracts.add(normalized)
                record.contract_kinds[normalized] = kind

    for path in package_root.rglob("*.py"):
        if should_skip(path.relative_to(package_root)):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="ignore"))
        except SyntaxError:
            continue

        analyzer = PythonRosAnalyzer()
        analyzer.visit(tree)

        record.node_names.update(analyzer.node_names)
        record.files.add(str(path.relative_to(repo_root)))

        default_node_name = record.node_name()
        default_namespace = record.namespace()
        for bucket, values in analyzer.calls.items():
            target = getattr(record, bucket)
            for raw in values:
                normalized = normalize_endpoint(raw, default_node_name, default_namespace)
                normalized = apply_endpoint_rewrite(record, normalized)
                if normalized:
                    target.add(normalized)

    return record


def collect_packages(repo_root: Path) -> list[PackageRecord]:
    overrides = load_overrides(repo_root)
    candidates: list[PackageRecord] = []
    root_dirs = [
        repo_root / "src",
        repo_root / "ref_src" / "knowledge_sources",
    ]

    for base in root_dirs:
        if not base.exists():
            continue
        for child in sorted(base.iterdir()):
            if not child.is_dir():
                continue
            if child.name.startswith("."):
                continue
            if should_skip(child.relative_to(repo_root)):
                continue
            if not any(child.rglob("*.py")) and not (child / "package.xml").exists():
                continue
            candidates.append(analyze_package(repo_root, child, overrides))

    return [record for record in candidates if any(
        [
            record.publishes,
            record.subscribes,
            record.service_clients,
            record.service_servers,
            record.action_clients,
            record.action_servers,
            record.contracts,
        ]
    )]


def sanitize_name(prefix: str, value: str) -> str:
    cleaned = value.strip("/")
    cleaned = cleaned.replace("~/", "private/")
    cleaned = cleaned.replace("*", "any")
    cleaned = re.sub(r"[^A-Za-z0-9]+", "_", cleaned).strip("_").lower()
    if not cleaned:
        cleaned = "root"
    return f"{prefix}_{cleaned}"


def build_endpoint_index(packages: list[PackageRecord]) -> dict[str, dict[str, set[str]]]:
    endpoint_index: dict[str, dict[str, set[str]]] = defaultdict(lambda: defaultdict(set))
    mappings = {
        "publishes": "publishers",
        "subscribes": "subscribers",
        "service_clients": "service_clients",
        "service_servers": "service_servers",
        "action_clients": "action_clients",
        "action_servers": "action_servers",
        "contracts": "contracts",
    }
    for record in packages:
        for attr_name, bucket_name in mappings.items():
            for endpoint in getattr(record, attr_name):
                endpoint_index[endpoint][bucket_name].add(record.name)
        for endpoint in record.contracts:
            endpoint_index[endpoint]["contracts"].add(record.name)
            if endpoint in record.contract_kinds:
                endpoint_index[endpoint]["contract_kinds"].add(record.contract_kinds[endpoint])
    return endpoint_index


def infer_endpoint_kind(endpoint: str, roles: dict[str, set[str]]) -> str:
    contract_kinds = roles.get("contract_kinds", set())
    if roles.get("action_clients") or roles.get("action_servers") or "action" in contract_kinds:
        return "action"
    if roles.get("service_clients") or roles.get("service_servers") or "service" in contract_kinds:
        return "service"
    return "topic"


def render_proxy(packages: list[PackageRecord], endpoint_index: dict[str, dict[str, set[str]]]) -> str:
    lines = [
        '"""Generated ROS interaction proxy for GitNexus.',
        "",
        "This file makes ROS runtime seams explicit so GitNexus can index",
        "node/topic/service/action relationships that are otherwise hidden",
        "behind ROS APIs and launch indirection.",
        '"""',
        "",
    ]

    endpoints = sorted(endpoint_index)
    for endpoint in endpoints:
        kind = infer_endpoint_kind(endpoint, endpoint_index[endpoint])
        fn_name = sanitize_name(f"ros_{kind}", endpoint)
        lines.extend(
            [
                f"def {fn_name}():",
                f'    """ROS {kind} endpoint {endpoint}."""',
                f'    return "{endpoint}"',
                "",
            ]
        )

    for record in sorted(packages, key=lambda item: item.name):
        contract_name = sanitize_name("ros_contract", record.name)
        lines.append(f"def {contract_name}():")
        lines.append(f'    """Interface contracts exported by {record.name}."""')
        if record.contracts:
            for endpoint in sorted(record.contracts):
                contract_kind = record.contract_kinds.get(endpoint, "topic")
                prefix = f"ros_{contract_kind}"
                lines.append(f"    {sanitize_name(prefix, endpoint)}()")
        else:
            lines.append("    return None")
        lines.append("")

        node_name = sanitize_name("ros_node", record.name)
        lines.append(f"def {node_name}():")
        lines.append(f'    """Runtime ROS proxy for package/node {record.name}."""')
        emitted = False
        for bucket, prefix in (
            ("publishes", "ros_topic"),
            ("subscribes", "ros_topic"),
            ("service_clients", "ros_service"),
            ("service_servers", "ros_service"),
            ("action_clients", "ros_action"),
            ("action_servers", "ros_action"),
        ):
            for endpoint in sorted(getattr(record, bucket)):
                lines.append(f"    {sanitize_name(prefix, endpoint)}()")
                emitted = True
        if not emitted:
            lines.append("    return None")
        lines.append("")

    for endpoint in endpoints:
        roles = endpoint_index[endpoint]
        if roles.get("publishers") and roles.get("subscribers"):
            flow_name = sanitize_name("ros_flow_topic", endpoint)
            lines.append(f"def {flow_name}():")
            lines.append(f'    """Publisher/subscriber flow for {endpoint}."""')
            for package_name in sorted(roles["publishers"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append(f"    {sanitize_name('ros_topic', endpoint)}()")
            for package_name in sorted(roles["subscribers"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append("")
        if roles.get("service_servers") and roles.get("service_clients"):
            flow_name = sanitize_name("ros_flow_service", endpoint)
            lines.append(f"def {flow_name}():")
            lines.append(f'    """Client/server flow for {endpoint}."""')
            for package_name in sorted(roles["service_servers"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append(f"    {sanitize_name('ros_service', endpoint)}()")
            for package_name in sorted(roles["service_clients"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append("")
        if roles.get("action_servers") and roles.get("action_clients"):
            flow_name = sanitize_name("ros_flow_action", endpoint)
            lines.append(f"def {flow_name}():")
            lines.append(f'    """Action server/client flow for {endpoint}."""')
            for package_name in sorted(roles["action_servers"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append(f"    {sanitize_name('ros_action', endpoint)}()")
            for package_name in sorted(roles["action_clients"]):
                lines.append(f"    {sanitize_name('ros_node', package_name)}()")
            lines.append("")

    return "\n".join(lines).strip() + "\n"


def render_markdown(packages: list[PackageRecord], endpoint_index: dict[str, dict[str, set[str]]]) -> str:
    lines = [
        "# ROS Runtime Graph",
        "",
        "This file is generated from the repository source to make ROS runtime",
        "publish/subscribe, service, and action seams explicit for humans and for",
        "GitNexus indexing.",
        "",
        f"Generated at: {datetime.now(timezone.utc).isoformat()}",
        "",
        "## Packages",
        "",
        "| Package | Publishes | Subscribes | Service Clients | Service Servers | Action Clients | Action Servers | Contracts |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    def fmt(values: set[str]) -> str:
        return "<br>".join(sorted(values)) if values else "-"

    for record in sorted(packages, key=lambda item: item.name):
        lines.append(
            "| "
            + " | ".join(
                [
                    record.name,
                    fmt(record.publishes),
                    fmt(record.subscribes),
                    fmt(record.service_clients),
                    fmt(record.service_servers),
                    fmt(record.action_clients),
                    fmt(record.action_servers),
                    fmt(record.contracts),
                ]
            )
            + " |"
        )

    lines.extend(["", "## Shared Runtime Endpoints", ""])
    for endpoint in sorted(endpoint_index):
        roles = endpoint_index[endpoint]
        lines.append(f"### `{endpoint}`")
        lines.append("")
        for label, bucket in (
            ("Publishers", "publishers"),
            ("Subscribers", "subscribers"),
            ("Service Servers", "service_servers"),
            ("Service Clients", "service_clients"),
            ("Action Servers", "action_servers"),
            ("Action Clients", "action_clients"),
            ("Contracts", "contracts"),
        ):
            values = sorted(roles.get(bucket, []))
            if values:
                lines.append(f"- {label}: {', '.join(values)}")
        lines.append("")

    return "\n".join(lines).strip() + "\n"


def write_outputs(repo_root: Path, packages: list[PackageRecord]) -> None:
    endpoint_index = build_endpoint_index(packages)
    data = {
        "generatedAt": datetime.now(timezone.utc).isoformat(),
        "packages": [
            {
                "name": record.name,
                "root": record.root,
                "nodeNames": sorted(record.node_names),
                "publishes": sorted(record.publishes),
                "subscribes": sorted(record.subscribes),
                "serviceClients": sorted(record.service_clients),
                "serviceServers": sorted(record.service_servers),
                "actionClients": sorted(record.action_clients),
                "actionServers": sorted(record.action_servers),
                "contracts": sorted(record.contracts),
                "files": sorted(record.files),
            }
            for record in sorted(packages, key=lambda item: item.name)
        ],
        "endpoints": {
            endpoint: {bucket: sorted(values) for bucket, values in sorted(roles.items())}
            for endpoint, roles in sorted(endpoint_index.items())
        },
    }

    for relative_path in (ENDPOINT_MD_PATH, ENDPOINT_JSON_PATH, ENDPOINT_PROXY_PATH):
        target = repo_root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)

    (repo_root / ENDPOINT_JSON_PATH).write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (repo_root / ENDPOINT_MD_PATH).write_text(
        render_markdown(packages, endpoint_index),
        encoding="utf-8",
    )
    (repo_root / ENDPOINT_PROXY_PATH).write_text(
        render_proxy(packages, endpoint_index),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ROS runtime proxy artifacts for GitNexus.")
    parser.add_argument("--repo-root", required=True, help="Repository root to analyze.")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    packages = collect_packages(repo_root)
    write_outputs(repo_root, packages)


if __name__ == "__main__":
    main()
