#!/usr/bin/env python3
"""Probe Ollama chat models for planner/chatbot suitability.

The script is intentionally dependency-light so it can run on the host or
inside the ROS container. It prints JSON by default and can also emit a compact
Markdown table for docs.
"""

from __future__ import annotations

import argparse
import json
import time
from urllib import error as url_error
from urllib import request as url_request


DEFAULT_MODELS = (
    'gemma4:31b-cloud',
    'glm-5.1:cloud',
    'kimi-k2.6:cloud',
    'deepseek-v4-flash:cloud',
    'gpt-oss:120b-cloud',
)

PLANNER_MESSAGES = (
    {
        'role': 'system',
        'content': (
            'Return only valid compact JSON. Plan over abstract robot skills. '
            'Allowed skill names: perform_motion, look_at, scan. '
            'Allowed step types: skill, look_at, say, noop. '
            'The JSON object must contain a top-level "steps" array; do not '
            'return a top-level "plan" array.'
        ),
    },
    {
        'role': 'user',
        'content': json.dumps(
            {
                'goal_text': 'look around and tell me whether you see a person',
                'normalized_intents': ['inspect_people'],
                'scene_targets': ['person'],
                'planner_mode': 'multi_step',
                'expected_shape': {
                    'steps': [
                        {
                            'type': 'skill',
                            'name': 'scan',
                            'args': {'target': 'person'},
                        }
                    ]
                },
            },
            separators=(',', ':'),
        ),
    },
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-url', default='http://127.0.0.1:11434')
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument('--max-tokens', type=int, default=320)
    parser.add_argument('--temperature', type=float, default=0.0)
    parser.add_argument('--think', action='store_true')
    parser.add_argument('--markdown', action='store_true')
    parser.add_argument('models', nargs='*', default=list(DEFAULT_MODELS))
    args = parser.parse_args()

    results = [
        _probe_model(
            model=model,
            base_url=args.base_url,
            timeout_sec=args.timeout_sec,
            max_tokens=args.max_tokens,
            temperature=args.temperature,
            think=args.think,
        )
        for model in args.models
    ]
    if args.markdown:
        print(_markdown_table(results))
    else:
        print(json.dumps({'results': results}, indent=2, sort_keys=True))
    return 0


def _probe_model(
    *,
    model: str,
    base_url: str,
    timeout_sec: float,
    max_tokens: int,
    temperature: float,
    think: bool,
) -> dict:
    started = time.monotonic()
    payload = {
        'model': model,
        'messages': list(PLANNER_MESSAGES),
        'stream': False,
        'think': bool(think),
        'options': {
            'temperature': float(temperature),
            'num_predict': int(max_tokens),
        },
    }
    try:
        response = _post_json(
            str(base_url).rstrip('/') + '/api/chat',
            payload,
            timeout_sec=timeout_sec,
        )
        latency = round(time.monotonic() - started, 3)
        text = _assistant_text(response)
        parsed = _extract_json(text)
        score, notes = _score_response(text, parsed)
        return {
            'model': model,
            'ok': bool(text),
            'latency_sec': latency,
            'score': score,
            'notes': notes,
            'response_preview': text[:500],
        }
    except Exception as err:  # pragma: no cover - runtime diagnostics
        return {
            'model': model,
            'ok': False,
            'latency_sec': round(time.monotonic() - started, 3),
            'score': 0,
            'notes': ['request_failed: %s' % err],
            'response_preview': '',
        }


def _post_json(url: str, payload: dict, *, timeout_sec: float) -> dict:
    request = url_request.Request(
        url,
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    try:
        with url_request.urlopen(request, timeout=float(timeout_sec)) as response:
            return json.loads(response.read().decode('utf-8'))
    except url_error.HTTPError as err:
        body = err.read().decode('utf-8', errors='replace')
        raise RuntimeError('HTTP %s: %s' % (err.code, body[:300])) from err


def _assistant_text(response: dict) -> str:
    message = response.get('message', {})
    if isinstance(message, dict):
        content = str(message.get('content', '')).strip()
        if content:
            return content
        for key in ('thinking', 'reasoning'):
            text = str(message.get(key, '')).strip()
            if text:
                return text
    for key in ('response', 'thinking', 'reasoning'):
        text = str(response.get(key, '')).strip()
        if text:
            return text
    return ''


def _extract_json(text: str) -> dict:
    clean = str(text or '').strip()
    if clean.startswith('```'):
        clean = clean.strip('`').strip()
        if clean.lower().startswith('json'):
            clean = clean[4:].strip()
    try:
        value = json.loads(clean)
        return value if isinstance(value, dict) else {}
    except json.JSONDecodeError:
        start = clean.find('{')
        end = clean.rfind('}')
        if start >= 0 and end > start:
            try:
                value = json.loads(clean[start : end + 1])
                return value if isinstance(value, dict) else {}
            except json.JSONDecodeError:
                return {}
    return {}


def _score_response(text: str, parsed: dict) -> tuple[int, list[str]]:
    notes: list[str] = []
    score = 0
    if text:
        score += 1
    else:
        notes.append('empty_response')
    if parsed:
        score += 2
    else:
        notes.append('not_json')
        return score, notes

    steps = parsed.get('steps')
    if not isinstance(steps, list):
        plan = parsed.get('plan', {})
        if isinstance(plan, dict):
            steps = plan.get('steps')
        elif isinstance(plan, list):
            notes.append('plan_array_not_supported')
    if isinstance(steps, list) and steps:
        score += 2
        names = {
            str(step.get('name', '')).strip().lower()
            for step in steps
            if isinstance(step, dict)
        }
        if names & {'scan', 'look_at', 'perform_motion'}:
            score += 2
        else:
            notes.append('no_supported_skill_names')
    else:
        notes.append('no_steps')

    if '```' in text:
        notes.append('fenced_output')
    return score, notes


def _markdown_table(results: list[dict]) -> str:
    lines = [
        '| Model | OK | Latency | Score | Notes |',
        '| --- | --- | ---: | ---: | --- |',
    ]
    for item in results:
        notes = ', '.join(item.get('notes', [])) or 'clean'
        lines.append(
            '| {model} | {ok} | {latency_sec:.3f}s | {score} | {notes} |'.format(
                model=item.get('model', ''),
                ok='yes' if item.get('ok') else 'no',
                latency_sec=float(item.get('latency_sec', 0.0)),
                score=int(item.get('score', 0)),
                notes=notes.replace('|', '/'),
            )
        )
    return '\n'.join(lines)


if __name__ == '__main__':
    raise SystemExit(main())
