#!/usr/bin/env python3
"""Probe or chat with an OpenAI-compatible vLLM endpoint."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from urllib import error as url_error
from urllib import request as url_request


DEFAULT_MODEL = "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ"
DEFAULT_LAB_BASE_URL = "http://10.7.138.215:8004"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("VLLM_BASE_URL", ""),
        help="Server base URL, for example http://10.0.0.12:8004.",
    )
    parser.add_argument(
        "--lab-pc",
        action="store_true",
        help="Use the current lab PC ZeroTier endpoint (%s)." % DEFAULT_LAB_BASE_URL,
    )
    parser.add_argument("--model", default=os.environ.get("VLLM_MODEL", DEFAULT_MODEL))
    parser.add_argument("--system", default="Eres un robot de terapia amigable.")
    parser.add_argument("--user", default="Hola, ¿qué tal?")
    parser.add_argument("--max-tokens", type=int, default=512)
    parser.add_argument("--temperature", type=float, default=0.7)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument(
        "--list-models",
        action="store_true",
        help="Print /v1/models before sending a chat request.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Run a simple terminal chat loop with conversation history.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print the full JSON response instead of only assistant text.",
    )
    parser.add_argument(
        "--api-key-env",
        default=os.environ.get("VLLM_API_KEY_ENV", "VLLM_API_KEY"),
        help="Optional environment variable containing a bearer token.",
    )
    args = parser.parse_args()

    if args.lab_pc and not args.base_url:
        args.base_url = DEFAULT_LAB_BASE_URL

    if not args.base_url:
        print(
            "Provide --base-url, --lab-pc, or VLLM_BASE_URL, e.g. http://IP:8004",
            file=sys.stderr,
        )
        return 2

    headers = {"Content-Type": "application/json"}
    api_key = os.environ.get(args.api_key_env, "").strip()
    if api_key:
        headers["Authorization"] = "Bearer %s" % api_key

    if args.list_models:
        if not _print_models(args.base_url, headers=headers, timeout_sec=args.timeout_sec):
            return 1

    if args.interactive:
        return _run_interactive_chat(args, headers=headers)

    payload = _chat_payload(
        model=args.model,
        messages=[
            {"role": "system", "content": args.system},
            {"role": "user", "content": args.user},
        ],
        max_tokens=args.max_tokens,
        temperature=args.temperature,
    )
    url = _chat_url(args.base_url)
    started = time.monotonic()
    try:
        response = _post_json(url, payload, headers=headers, timeout_sec=args.timeout_sec)
    except Exception as exc:
        print("vLLM probe failed: %s" % exc, file=sys.stderr)
        return 1

    elapsed = time.monotonic() - started
    print("vLLM probe OK | elapsed=%.2fs model=%s" % (elapsed, response.get("model", args.model)))
    if args.raw:
        print(json.dumps(response, ensure_ascii=False, indent=2))
    else:
        print(_assistant_text(response) or json.dumps(response, ensure_ascii=False)[:1000])
    return 0


def _chat_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/v1/chat/completions"


def _models_url(base_url: str) -> str:
    return base_url.rstrip("/") + "/v1/models"


def _chat_payload(
    *,
    model: str,
    messages: list[dict],
    max_tokens: int,
    temperature: float,
) -> dict:
    return {
        "model": model,
        "messages": messages,
        "max_tokens": max_tokens,
        "temperature": temperature,
    }


def _post_json(url: str, payload: dict, *, headers: dict[str, str], timeout_sec: float) -> dict:
    request = url_request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers=headers,
    )
    try:
        with url_request.urlopen(request, timeout=timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))
    except url_error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError("HTTP %s %s | %s" % (exc.code, exc.reason, body[:500])) from exc


def _get_json(url: str, *, headers: dict[str, str], timeout_sec: float) -> dict:
    request = url_request.Request(url, method="GET", headers=headers)
    try:
        with url_request.urlopen(request, timeout=timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))
    except url_error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError("HTTP %s %s | %s" % (exc.code, exc.reason, body[:500])) from exc


def _print_models(base_url: str, *, headers: dict[str, str], timeout_sec: float) -> bool:
    try:
        payload = _get_json(_models_url(base_url), headers=headers, timeout_sec=timeout_sec)
    except Exception as exc:
        print("vLLM model listing failed: %s" % exc, file=sys.stderr)
        return False
    model_ids = [
        str(item.get("id", "")).strip()
        for item in payload.get("data", [])
        if isinstance(item, dict) and str(item.get("id", "")).strip()
    ]
    print("Available models:")
    for model_id in model_ids:
        print("  %s" % model_id)
    if not model_ids:
        print("  <none reported>")
    return True


def _run_interactive_chat(args: argparse.Namespace, *, headers: dict[str, str]) -> int:
    messages = [{"role": "system", "content": args.system}]
    print("Connected to %s using model %s" % (args.base_url.rstrip("/"), args.model))
    print("Press Enter on an empty line to exit.")
    while True:
        try:
            user_text = input("Tu: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not user_text:
            return 0
        messages.append({"role": "user", "content": user_text})
        payload = _chat_payload(
            model=args.model,
            messages=messages,
            max_tokens=args.max_tokens,
            temperature=args.temperature,
        )
        started = time.monotonic()
        try:
            response = _post_json(
                _chat_url(args.base_url),
                payload,
                headers=headers,
                timeout_sec=args.timeout_sec,
            )
        except Exception as exc:
            print("vLLM chat failed: %s" % exc, file=sys.stderr)
            return 1
        assistant_text = _assistant_text(response)
        if not assistant_text:
            print("vLLM chat returned no assistant text: %s" % json.dumps(response, ensure_ascii=False)[:1000])
            return 1
        elapsed = time.monotonic() - started
        print("LLM (%.2fs): %s" % (elapsed, assistant_text))
        messages.append({"role": "assistant", "content": assistant_text})


def _assistant_text(payload: dict) -> str:
    choice = (payload.get("choices") or [{}])[0]
    message = choice.get("message") if isinstance(choice, dict) else {}
    return str((message or {}).get("content", "")).strip()


if __name__ == "__main__":
    raise SystemExit(main())
