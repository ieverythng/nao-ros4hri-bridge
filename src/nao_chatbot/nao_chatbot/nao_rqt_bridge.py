#!/usr/bin/env python3
"""Backward-compatible alias for the dialogue manager node."""

from nao_chatbot.dialogue_manager import DialogueManager
from nao_chatbot.dialogue_manager import main


class NaoRqtBridge(DialogueManager):
    """Deprecated wrapper kept for launch/backward compatibility."""


__all__ = ["NaoRqtBridge", "main"]
