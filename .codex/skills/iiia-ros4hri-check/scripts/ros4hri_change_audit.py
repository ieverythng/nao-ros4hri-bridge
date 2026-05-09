#!/usr/bin/env python3
"""Thin wrapper: run the canonical audit at repo ``scripts/ros4hri_change_audit.py``."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    here = Path(__file__).resolve()
    for base in here.parents:
        target = (base / "scripts" / "ros4hri_change_audit.py").resolve()
        # Avoid selecting this wrapper (under .codex/.../scripts/) as the target.
        if target.is_file() and target != here:
            return subprocess.call([sys.executable, str(target), *sys.argv[1:]])
    fallback = here.with_name("ros4hri_change_audit_standalone.py")
    if fallback.is_file():
        return subprocess.call([sys.executable, str(fallback), *sys.argv[1:]])
    print(
        "Could not find repo scripts/ros4hri_change_audit.py or fallback "
        "ros4hri_change_audit_standalone.py.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
