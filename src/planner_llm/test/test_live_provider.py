import os

import pytest

from planner_llm.providers import PlannerProviderConfig
from planner_llm.providers import build_provider


@pytest.mark.skipif(
    os.environ.get('RUN_PLANNER_LLM_LIVE_TESTS') != '1',
    reason='set RUN_PLANNER_LLM_LIVE_TESTS=1 to exercise the configured LLM backend',
)
def test_live_planner_provider_readiness_probe():
    provider = build_provider(
        PlannerProviderConfig(
            provider=os.environ.get('PLANNER_LLM_TEST_PROVIDER', 'ollama'),
            model=os.environ.get('PLANNER_LLM_TEST_MODEL', 'gemma4:31b-cloud'),
            base_url=os.environ.get('PLANNER_LLM_TEST_BASE_URL', 'http://127.0.0.1:11434'),
            api_key_env=os.environ.get('PLANNER_LLM_TEST_API_KEY_ENV', 'OPENAI_API_KEY'),
            timeout_sec=float(os.environ.get('PLANNER_LLM_TEST_TIMEOUT_SEC', '60.0')),
            max_tokens=64,
            temperature=0.0,
        )
    )

    text = provider.generate(
        [
            {'role': 'system', 'content': 'Reply only with JSON. No prose.'},
            {'role': 'user', 'content': 'Return {"ready":true}.'},
        ]
    )

    assert 'ready' in text.lower()
