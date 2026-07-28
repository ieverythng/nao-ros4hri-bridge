#!/usr/bin/env python3
"""Focused tests for the model-priority resolver."""

from __future__ import annotations

import unittest
from unittest import mock

import resolve_demo_model


class ResolveDemoModelTests(unittest.TestCase):
    def test_vllm_is_selected_before_ollama(self) -> None:
        with mock.patch.object(resolve_demo_model, "_vllm_models", return_value=["served-model"]), mock.patch.object(
            resolve_demo_model, "_probe_vllm"
        ), mock.patch.object(resolve_demo_model, "_ollama_models") as ollama_models:
            choice, diagnostics = resolve_demo_model.resolve(
                vllm_base_url="http://vllm:8004",
                vllm_preference=("served-model",),
                ollama_host="127.0.0.1:11434",
                ollama_preference=("gemma4:31b-cloud",),
                timeout_sec=1.0,
            )

        self.assertEqual(choice.backend, "vllm")
        self.assertEqual(choice.model, "served-model")
        self.assertEqual(diagnostics, [])
        ollama_models.assert_called_once()

    def test_cli_override_is_tried_before_backend_preferences(self) -> None:
        with mock.patch.object(resolve_demo_model, "_vllm_models", return_value=["preferred-model"]), mock.patch.object(
            resolve_demo_model, "_ollama_models", return_value=["fallback-model"]
        ), mock.patch.object(resolve_demo_model, "_probe_vllm") as probe_vllm, mock.patch.object(
            resolve_demo_model, "_probe_ollama"
        ):
            choice, _ = resolve_demo_model.resolve(
                vllm_base_url="http://vllm:8004",
                vllm_preference=("preferred-model",),
                ollama_host="127.0.0.1:11434",
                ollama_preference=("fallback-model",),
                timeout_sec=1.0,
                model_override="preferred-model",
            )

        self.assertEqual(choice.model, "preferred-model")
        probe_vllm.assert_called_once()

    def test_ollama_is_selected_when_vllm_is_unavailable(self) -> None:
        with mock.patch.object(
            resolve_demo_model,
            "_vllm_models",
            side_effect=resolve_demo_model.BackendUnavailable("connection refused"),
        ), mock.patch.object(resolve_demo_model, "_ollama_models", return_value=["gemma4:31b-cloud"]), mock.patch.object(
            resolve_demo_model, "_probe_ollama"
        ):
            choice, diagnostics = resolve_demo_model.resolve(
                vllm_base_url="http://vllm:8004",
                vllm_preference=("served-model",),
                ollama_host="127.0.0.1:11434",
                ollama_preference=("gemma4:31b-cloud",),
                timeout_sec=1.0,
            )

        self.assertEqual(choice.backend, "ollama")
        self.assertEqual(choice.model, "gemma4:31b-cloud")
        self.assertEqual(len(diagnostics), 1)

    def test_no_backend_is_a_hard_failure(self) -> None:
        unavailable = resolve_demo_model.BackendUnavailable("down")
        with mock.patch.object(resolve_demo_model, "_vllm_models", side_effect=unavailable), mock.patch.object(
            resolve_demo_model, "_ollama_models", side_effect=unavailable
        ):
            with self.assertRaises(resolve_demo_model.BackendUnavailable):
                resolve_demo_model.resolve(
                    vllm_base_url="http://vllm:8004",
                    vllm_preference=(),
                    ollama_host="127.0.0.1:11434",
                    ollama_preference=(),
                    timeout_sec=1.0,
                )

    def test_ollama_choice_generates_all_launch_overrides(self) -> None:
        choice = resolve_demo_model.BackendChoice(
            "ollama",
            "gemma4:31b-cloud",
            ollama_host="127.0.0.1:11434",
        )
        self.assertIn("planner_llm_provider:=ollama", choice.launch_args)
        self.assertIn("chatbot_server_url:=http://127.0.0.1:11434/api/chat", choice.launch_args)
        self.assertIn("planner_llm_base_url:=http://127.0.0.1:11434", choice.launch_args)
        self.assertIn("chatbot_think:=false", choice.launch_args)
        self.assertIn("planner_llm_think:=false", choice.launch_args)


if __name__ == "__main__":
    unittest.main()
