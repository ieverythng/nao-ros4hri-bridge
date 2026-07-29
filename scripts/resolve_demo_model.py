#!/usr/bin/env python3
"""Resolve a live demo LLM with vLLM-first, Ollama-second policy.

The resolver is deliberately independent of ROS. It checks model inventory and
performs a small non-thinking completion before returning launch arguments.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from dataclasses import asdict, dataclass
from typing import Any
from urllib import error as url_error
from urllib import request as url_request


DEFAULT_VLLM_BASE_URL = "http://10.7.138.215:8004"
DEFAULT_VLLM_PREFERENCE = (
    "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ",
    "cyankiwi/Qwen3.5-35B-A3B-AWQ-4bit",
)
DEFAULT_OLLAMA_HOST = "127.0.0.1:11434"
DEFAULT_OLLAMA_PREFERENCE = (
    "gemma4:31b-cloud",
    "nemotron-3-super:cloud",
    "gemma4:cloud",
)


class BackendUnavailable(RuntimeError):
    """Raised when an advertised backend cannot complete a readiness probe."""


@dataclass(frozen=True)
class BackendChoice:
    backend: str
    model: str
    vllm_base_url: str = ""
    ollama_host: str = ""

    @property
    def chatbot_server_url(self) -> str:
        if self.backend == "vllm":
            return self.vllm_base_url.rstrip("/") + "/v1/chat/completions"
        return "http://%s/api/chat" % self.ollama_host

    @property
    def launch_args(self) -> list[str]:
        common = [
            "chatbot_model:=%s" % self.model,
            "chatbot_intent_model:=%s" % self.model,
            "chatbot_server_url:=%s" % self.chatbot_server_url,
            "chatbot_think:=false",
            "planner_llm_model:=%s" % self.model,
            "planner_llm_think:=false",
            "start_managed_ollama:=false",
        ]
        if self.backend == "vllm":
            return common + [
                "planner_llm_provider:=openai_compatible",
                "planner_llm_base_url:=%s" % self.vllm_base_url.rstrip("/"),
            ]
        return common + [
            "planner_llm_provider:=ollama",
            "planner_llm_base_url:=http://%s" % self.ollama_host,
        ]


def _split_preference(value: str | None, default: tuple[str, ...]) -> tuple[str, ...]:
    if value is None:
        return default
    values = tuple(item.strip() for item in value.split(",") if item.strip())
    return values or default


def _normalise_host(value: str) -> str:
    value = value.strip().rstrip("/")
    if value.startswith("http://"):
        return value[7:]
    if value.startswith("https://"):
        return value[8:]
    return value


def _get_json(url: str, *, timeout_sec: float) -> dict[str, Any]:
    request = url_request.Request(url, method="GET")
    try:
        with url_request.urlopen(request, timeout=timeout_sec) as response:
            if response.status != 200:
                raise BackendUnavailable("HTTP %s from %s" % (response.status, url))
            payload = json.loads(response.read().decode("utf-8"))
    except url_error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise BackendUnavailable("HTTP %s from %s: %s" % (exc.code, url, body[:240])) from exc
    except (OSError, ValueError) as exc:
        raise BackendUnavailable("%s: %s" % (url, exc)) from exc
    if not isinstance(payload, dict):
        raise BackendUnavailable("non-object JSON from %s" % url)
    return payload


def _post_json(url: str, payload: dict[str, Any], *, timeout_sec: float) -> dict[str, Any]:
    request = url_request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with url_request.urlopen(request, timeout=timeout_sec) as response:
            if response.status != 200:
                raise BackendUnavailable("HTTP %s from %s" % (response.status, url))
            result = json.loads(response.read().decode("utf-8"))
    except url_error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise BackendUnavailable("HTTP %s from %s: %s" % (exc.code, url, body[:240])) from exc
    except (OSError, ValueError) as exc:
        raise BackendUnavailable("%s: %s" % (url, exc)) from exc
    if not isinstance(result, dict):
        raise BackendUnavailable("non-object JSON from %s" % url)
    return result


def _vllm_models(base_url: str, *, timeout_sec: float) -> list[str]:
    payload = _get_json(base_url.rstrip("/") + "/v1/models", timeout_sec=timeout_sec)
    models = payload.get("data", [])
    if not isinstance(models, list):
        raise BackendUnavailable("invalid /v1/models response from %s" % base_url)
    return [
        str(item.get("id", "")).strip()
        for item in models
        if isinstance(item, dict) and str(item.get("id", "")).strip()
    ]


def _ollama_models(host: str, *, timeout_sec: float) -> list[str]:
    payload = _get_json("http://%s/api/tags" % host, timeout_sec=timeout_sec)
    models = payload.get("models", [])
    if not isinstance(models, list):
        raise BackendUnavailable("invalid /api/tags response from %s" % host)
    return [
        str(item.get("name", "")).strip()
        for item in models
        if isinstance(item, dict) and str(item.get("name", "")).strip()
    ]


def _no_thinking_options(model: str) -> dict[str, Any]:
    leaf = model.rsplit("/", 1)[-1].lower()
    if leaf.startswith(("qwen3", "qwen3.5")):
        return {"chat_template_kwargs": {"enable_thinking": False}}
    return {}


def _probe_vllm(base_url: str, model: str, *, timeout_sec: float) -> None:
    payload: dict[str, Any] = {
        "model": model,
        "messages": [
            {"role": "system", "content": "Return exactly READY."},
            {"role": "user", "content": "Return exactly READY."},
        ],
        "max_tokens": 64,
        "temperature": 0.0,
        "stream": False,
    }
    payload.update(_no_thinking_options(model))
    response = _post_json(
        base_url.rstrip("/") + "/v1/chat/completions",
        payload,
        timeout_sec=timeout_sec,
    )
    choices = response.get("choices", [])
    if not isinstance(choices, list) or not choices:
        raise BackendUnavailable("vLLM returned no choices for %s" % model)
    message = choices[0].get("message", {})
    content = message.get("content") if isinstance(message, dict) else None
    if not isinstance(content, str) or not content.strip():
        raise BackendUnavailable("vLLM returned empty assistant content for %s" % model)


def _probe_ollama(host: str, model: str, *, timeout_sec: float) -> None:
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": "Return exactly READY."},
            {"role": "user", "content": "Return exactly READY."},
        ],
        "stream": False,
        "think": False,
        "options": {"temperature": 0.0, "num_predict": 32},
    }
    response = _post_json(
        "http://%s/api/chat" % host,
        payload,
        timeout_sec=timeout_sec,
    )
    message = response.get("message", {})
    content = message.get("content") if isinstance(message, dict) else None
    if not isinstance(content, str) or not content.strip():
        raise BackendUnavailable("Ollama returned empty assistant content for %s" % model)


def _choose_from_inventory(inventory: list[str], preference: tuple[str, ...]) -> list[str]:
    ordered = [model for model in preference if model in inventory]
    ordered.extend(model for model in inventory if model not in ordered)
    return ordered


def resolve(
    *,
    vllm_base_url: str,
    vllm_preference: tuple[str, ...],
    ollama_host: str,
    ollama_preference: tuple[str, ...],
    timeout_sec: float,
    model_override: str = '',
) -> tuple[BackendChoice, list[str]]:
    diagnostics: list[str] = []
    vllm_models: list[str] | None = None
    try:
        vllm_models = _vllm_models(vllm_base_url, timeout_sec=timeout_sec)
        if not vllm_models:
            raise BackendUnavailable("/v1/models returned no model ids")
    except BackendUnavailable as exc:
        diagnostics.append("vLLM unavailable: %s" % exc)

    ollama_models: list[str] | None = None
    try:
        ollama_models = _ollama_models(ollama_host, timeout_sec=timeout_sec)
        if not ollama_models:
            raise BackendUnavailable("/api/tags returned no model names")
    except BackendUnavailable as exc:
        diagnostics.append("Ollama unavailable: %s" % exc)

    def try_vllm(model: str) -> BackendChoice | None:
        if vllm_models is None:
            return None
        try:
            _probe_vllm(vllm_base_url, model, timeout_sec=timeout_sec)
        except BackendUnavailable as exc:
            diagnostics.append("vLLM %s rejected: %s" % (model, exc))
            return None
        return BackendChoice("vllm", model, vllm_base_url=vllm_base_url)

    def try_ollama(model: str) -> BackendChoice | None:
        if ollama_models is None:
            return None
        try:
            _probe_ollama(ollama_host, model, timeout_sec=timeout_sec)
        except BackendUnavailable as exc:
            diagnostics.append("Ollama %s rejected: %s" % (model, exc))
            return None
        return BackendChoice("ollama", model, ollama_host=ollama_host)

    override = str(model_override or '').strip()
    if override:
        choice = try_vllm(override)
        if choice is not None:
            return choice, diagnostics
        choice = try_ollama(override)
        if choice is not None:
            return choice, diagnostics

    if vllm_models is not None:
        for model in _choose_from_inventory(vllm_models, vllm_preference):
            if model == override:
                continue
            choice = try_vllm(model)
            if choice is not None:
                return choice, diagnostics
        diagnostics.append("vLLM advertised models but none passed the readiness probe")

    if ollama_models is not None:
        for model in _choose_from_inventory(ollama_models, ollama_preference):
            if model == override:
                continue
            choice = try_ollama(model)
            if choice is not None:
                return choice, diagnostics
        diagnostics.append("Ollama advertised models but none passed the readiness probe")

    raise BackendUnavailable("No live model backend passed readiness checks")


def _write_fallback_event(
    *,
    path: str,
    requested_model: str,
    choice: BackendChoice,
    diagnostics: list[str],
) -> None:
    event = {
        "event": "llm_model_fallback",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "requested_model": requested_model,
        "unavailable_model": requested_model,
        "current_model": choice.model,
        "current_backend": choice.backend,
        "reason": diagnostics[0] if diagnostics else "preferred model readiness probe failed",
    }
    try:
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")
    except OSError as exc:
        print("[model resolver] could not write fallback log %s: %s" % (path, exc), file=sys.stderr)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vllm-base-url", default=os.environ.get("VLLM_BASE_URL", DEFAULT_VLLM_BASE_URL))
    parser.add_argument(
        "--vllm-preference",
        default=os.environ.get("VLLM_MODEL_PREFERENCE"),
        help="Comma-separated preferred vLLM ids. The first advertised id is used otherwise.",
    )
    parser.add_argument("--ollama-host", default=os.environ.get("OLLAMA_HOST", DEFAULT_OLLAMA_HOST))
    parser.add_argument(
        "--ollama-preference",
        default=os.environ.get("OLLAMA_MODEL_PREFERENCE"),
        help="Comma-separated preferred Ollama names.",
    )
    parser.add_argument(
        "--model-override",
        default="",
        help="Explicit CLI model to try before the normal backend preferences.",
    )
    parser.add_argument("--timeout-sec", type=float, default=float(os.environ.get("MODEL_PROBE_TIMEOUT_SEC", "8")))
    parser.add_argument("--json", action="store_true", help="Print the choice and diagnostics as JSON.")
    parser.add_argument("--launch-args", action="store_true", help="Print ROS launch arguments, one per line.")
    parser.add_argument(
        "--log-selection",
        action="store_true",
        help="Append a JSONL fallback event when the preferred model is not selected.",
    )
    parser.add_argument(
        "--selection-log",
        default=os.environ.get("MODEL_SELECTION_LOG", "/tmp/nao_model_selection.jsonl"),
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    choice, diagnostics = resolve(
        vllm_base_url=args.vllm_base_url,
        vllm_preference=_split_preference(args.vllm_preference, DEFAULT_VLLM_PREFERENCE),
        ollama_host=_normalise_host(args.ollama_host),
        ollama_preference=_split_preference(args.ollama_preference, DEFAULT_OLLAMA_PREFERENCE),
        timeout_sec=args.timeout_sec,
        model_override=args.model_override,
    )
    requested_model = args.model_override.strip() or _split_preference(
        args.vllm_preference,
        DEFAULT_VLLM_PREFERENCE,
    )[0]
    fallback_used = choice.backend != "vllm" or choice.model != requested_model
    if fallback_used:
        print(
            "[model resolver] fallback model used, current model %s unavailable: selected %s (%s)"
            % (requested_model, choice.model, choice.backend),
            file=sys.stderr,
        )
        if args.log_selection:
            _write_fallback_event(
                path=args.selection_log,
                requested_model=requested_model,
                choice=choice,
                diagnostics=diagnostics,
            )
    if args.launch_args:
        for launch_arg in choice.launch_args:
            print(launch_arg)
        return 0
    if args.json:
        print(
            json.dumps(
                {
                    "choice": asdict(choice),
                    "chatbot_server_url": choice.chatbot_server_url,
                    "launch_args": choice.launch_args,
                    "diagnostics": diagnostics,
                    "fallback_used": fallback_used,
                    "requested_model": requested_model,
                },
                indent=2,
            )
        )
        return 0
    print("[model resolver] backend=%s model=%s" % (choice.backend, choice.model))
    print("[model resolver] chatbot=%s" % choice.chatbot_server_url)
    for diagnostic in diagnostics:
        print("[model resolver] %s" % diagnostic, file=sys.stderr)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BackendUnavailable as exc:
        print("[model resolver] ERROR: %s" % exc, file=sys.stderr)
        raise SystemExit(1)
