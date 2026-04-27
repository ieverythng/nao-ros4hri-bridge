"""Provider adapters for planner_llm backends."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from urllib import error as url_error
from urllib import request as url_request


@dataclass(frozen=True)
class PlannerProviderConfig:
    provider: str = 'ollama'
    model: str = 'qwen3.5:397b-cloud'
    base_url: str = 'http://127.0.0.1:11434'
    api_key_env: str = 'OPENAI_API_KEY'
    temperature: float = 0.1
    max_tokens: int = 800
    timeout_sec: float = 20.0
    think: bool = False


class PlannerProviderError(RuntimeError):
    """Raised when a planner provider cannot produce a completion."""


class BasePlannerProvider:
    """Minimal provider interface shared by planner backends."""

    def __init__(self, config: PlannerProviderConfig) -> None:
        self.config = config

    def generate(self, messages: list[dict[str, str]]) -> str:
        raise NotImplementedError


class OllamaPlannerProvider(BasePlannerProvider):
    """Chat adapter for Ollama-compatible local models."""

    def generate(self, messages: list[dict[str, str]]) -> str:
        payload = {
            'model': self.config.model,
            'messages': messages,
            'stream': False,
            'think': bool(self.config.think),
            'options': {
                'temperature': float(self.config.temperature),
                'num_predict': int(self.config.max_tokens),
            },
        }
        response = _post_json(
            _join_url(self.config.base_url, '/api/chat'),
            payload,
            timeout_sec=self.config.timeout_sec,
            headers={},
        )
        content = _message_content(response.get('message', {}))
        if content:
            return content
        raise PlannerProviderError('Ollama response did not include message.content')


class OpenAICompatiblePlannerProvider(BasePlannerProvider):
    """Chat adapter for OpenAI-compatible endpoints such as WatsonOW."""

    def generate(self, messages: list[dict[str, str]]) -> str:
        payload = {
            'model': self.config.model,
            'messages': messages,
            'temperature': float(self.config.temperature),
            'max_tokens': int(self.config.max_tokens),
        }
        response = _post_json(
            _join_url(self.config.base_url, '/v1/chat/completions'),
            payload,
            timeout_sec=self.config.timeout_sec,
            headers=_authorization_headers(self.config.api_key_env),
        )
        choices = response.get('choices', [])
        if not isinstance(choices, list) or not choices:
            raise PlannerProviderError('OpenAI-compatible response contained no choices')
        content = _message_content(choices[0].get('message', {}))
        if content:
            return content
        raise PlannerProviderError('OpenAI-compatible response did not include message.content')


def build_provider(config: PlannerProviderConfig) -> BasePlannerProvider:
    provider_name = str(config.provider or 'ollama').strip().lower()
    if provider_name == 'ollama':
        return OllamaPlannerProvider(config)
    if provider_name in ('openai', 'openai_compatible', 'watsonow'):
        return OpenAICompatiblePlannerProvider(config)
    raise PlannerProviderError('Unsupported planner provider %s' % provider_name)


def _join_url(base_url: str, path: str) -> str:
    return str(base_url or '').rstrip('/') + path


def _authorization_headers(api_key_env: str) -> dict[str, str]:
    api_key = os.environ.get(api_key_env, '').strip()
    if not api_key:
        return {}
    return {'Authorization': 'Bearer %s' % api_key}


def _message_content(message) -> str:
    if not isinstance(message, dict):
        return ''
    return str(message.get('content', '')).strip()


def _post_json(url: str, payload: dict, *, timeout_sec: float, headers: dict[str, str]) -> dict:
    body = json.dumps(payload).encode('utf-8')
    request = url_request.Request(
        url,
        data=body,
        method='POST',
        headers={
            'Content-Type': 'application/json',
            **headers,
        },
    )
    try:
        with url_request.urlopen(request, timeout=float(timeout_sec)) as response:
            return json.loads(response.read().decode('utf-8'))
    except (url_error.HTTPError, url_error.URLError, TimeoutError, json.JSONDecodeError) as err:
        raise PlannerProviderError(str(err)) from err
