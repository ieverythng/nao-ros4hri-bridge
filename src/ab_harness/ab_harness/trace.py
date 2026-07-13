"""Append-only JSONL trace storage for H0 harness decisions."""

from __future__ import annotations

import json
from pathlib import Path

from ab_harness.contracts import HarnessTrace


class JsonlHarnessTraceStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def append(self, trace: HarnessTrace) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(trace.to_dict(), sort_keys=True) + '\n')

    def load_all(self) -> tuple[HarnessTrace, ...]:
        if not self.path.exists():
            return ()
        return tuple(
            HarnessTrace.from_dict(json.loads(line))
            for line in self.path.read_text(encoding='utf-8').splitlines()
            if line.strip()
        )
