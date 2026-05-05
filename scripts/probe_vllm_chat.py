#!/usr/bin/env python3
"""Probe an OpenAI-compatible vLLM chat endpoint."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from urllib import error as url_error
from urllib import request as url_request


DEFAULT_MODEL = "QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("VLLM_BASE_URL", ""),
        help="Server base URL, for example http://10.0.0.12:8004.",
    )
    parser.add_argument("--model", default=os.environ.get("VLLM_MODEL", DEFAULT_MODEL))
    parser.add_argument("--system", default="Eres un robot de terapia amigable.")
    parser.add_argument("--user", default="Hola, ¿qué tal?")
    parser.add_argument("--max-tokens", type=int, default=512)
    parser.add_argument("--temperature", type=float, default=0.7)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument(
        "--api-key-env",
        default=os.environ.get("VLLM_API_KEY_ENV", "VLLM_API_KEY"),
        help="Optional environment variable containing a bearer token.",
    )
    args = parser.parse_args()

    if not args.base_url:
        print("Provide --base-url or VLLM_BASE_URL, e.g. http://IP:8004", file=sys.stderr)
        return 2

    url = args.base_url.rstrip("/") + "/v1/chat/completions"
    payload = {
        "model": args.model,
        "messages": [
            {"role": "system", "content": args.system},
            {"role": "user", "content": args.user},
        ],
        "max_tokens": args.max_tokens,
        "temperature": args.temperature,
    }
    headers = {"Content-Type": "application/json"}
    api_key = os.environ.get(args.api_key_env, "").strip()
    if api_key:
        headers["Authorization"] = "Bearer %s" % api_key

    started = time.monotonic()
    try:
        response = _post_json(url, payload, headers=headers, timeout_sec=args.timeout_sec)
    except Exception as exc:
        print("vLLM probe failed: %s" % exc, file=sys.stderr)
        return 1

    elapsed = time.monotonic() - started
    choice = (response.get("choices") or [{}])[0]
    message = choice.get("message") if isinstance(choice, dict) else {}
    content = str((message or {}).get("content", "")).strip()
    print("vLLM probe OK | elapsed=%.2fs model=%s" % (elapsed, response.get("model", args.model)))
    print(content or json.dumps(response, ensure_ascii=False)[:1000])
    return 0


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


if __name__ == "__main__":
    raise SystemExit(main())
