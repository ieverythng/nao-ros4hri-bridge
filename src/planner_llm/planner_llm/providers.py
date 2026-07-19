"""Provider adapters for planner_llm backends."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import warnings
from urllib import error as url_error
from urllib import request as url_request

_NO_THINK_MODEL_PREFIXES = ('qwen3', 'qwen3.5')
_NO_THINK_PREFIX = (
    '/no_think\n'
    'Return only the final answer as valid JSON. Do not emit reasoning, analysis, or thinking text.'
)


@dataclass(frozen=True)
class PlannerProviderConfig:
    provider: str = 'openai_compatible'
    model: str = ''
    base_url: str = 'http://10.7.138.215:8004'
    api_key_env: str = 'OPENAI_API_KEY'
    temperature: float = 0.1
    top_p: float = 1.0
    top_k: int = 0
    min_p: float = 0.0
    presence_penalty: float = 0.0
    repetition_penalty: float = 1.0
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
        _require_model(self.config)
        request_messages = _no_think_messages(self.config.model, messages)
        payload = {
            'model': self.config.model,
            'messages': request_messages,
            'stream': False,
            'think': bool(self.config.think),
            'options': {
                'temperature': float(self.config.temperature),
                'top_p': float(self.config.top_p),
                'top_k': int(self.config.top_k),
                'min_p': float(self.config.min_p),
                'presence_penalty': float(self.config.presence_penalty),
                'repeat_penalty': float(self.config.repetition_penalty),
                'num_predict': int(self.config.max_tokens),
            },
        }
        response = _post_json(
            _join_url(self.config.base_url, '/api/chat'),
            payload,
            timeout_sec=self.config.timeout_sec,
            headers={},
        )
        content = _assistant_message_text(response.get('message', {}))
        if content:
            return content
        content = _thinking_text(response)
        if content:
            return content
        raise PlannerProviderError('Ollama response did not include message.content')


class OpenAICompatiblePlannerProvider(BasePlannerProvider):
    """Chat adapter for OpenAI-compatible endpoints such as WatsonOW."""

    def generate(self, messages: list[dict[str, str]]) -> str:
        _require_model(self.config)
        payload = {
            'model': self.config.model,
            'messages': messages,
            'temperature': float(self.config.temperature),
            'top_p': float(self.config.top_p),
            'top_k': int(self.config.top_k),
            'min_p': float(self.config.min_p),
            'presence_penalty': float(self.config.presence_penalty),
            'repetition_penalty': float(self.config.repetition_penalty),
            'max_tokens': int(self.config.max_tokens),
        }
        template_kwargs = _openai_chat_template_kwargs(self.config.model)
        if template_kwargs:
            payload['chat_template_kwargs'] = template_kwargs
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
        if config.think:
            warnings.warn(
                'planner_llm think=True is ignored by OpenAI-compatible providers',
                RuntimeWarning,
                stacklevel=2,
            )
        return OpenAICompatiblePlannerProvider(config)
    raise PlannerProviderError('Unsupported planner provider %s' % provider_name)


def _require_model(config: PlannerProviderConfig) -> None:
    if not str(config.model or '').strip():
        raise PlannerProviderError('planner_llm provider model is not configured')


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


def _assistant_message_text(message) -> str:
    if not isinstance(message, dict):
        return ''
    content = _message_content(message)
    if content:
        return content
    return _thinking_text(message)


def _thinking_text(payload) -> str:
    if not isinstance(payload, dict):
        return ''
    for key in ('thinking', 'reasoning'):
        text = str(payload.get(key, '')).strip()
        if text:
            return text
    return ''


def _no_think_messages(model: str, messages: list[dict[str, str]]) -> list[dict[str, str]]:
    if not _is_qwen3_model(model):
        return list(messages)
    if not messages:
        return [{'role': 'system', 'content': _NO_THINK_PREFIX}]
    prepared = [dict(message) for message in messages]
    first = prepared[0]
    if first.get('role') == 'system':
        first['content'] = '%s\n\n%s' % (_NO_THINK_PREFIX, str(first.get('content', '')).strip())
    else:
        prepared.insert(0, {'role': 'system', 'content': _NO_THINK_PREFIX})
    return prepared


def _is_qwen3_model(model: str) -> bool:
    model_name = str(model or '').strip().lower().rsplit('/', 1)[-1]
    return any(model_name.startswith(prefix) for prefix in _NO_THINK_MODEL_PREFIXES)


def _openai_chat_template_kwargs(model: str) -> dict[str, bool]:
    if not _is_qwen3_model(model):
        return {}
    return {'enable_thinking': False}


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
