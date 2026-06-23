#!/usr/bin/env python3
"""Record and verify that pre-push uses a fresh pre-commit result."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CACHE_PATH = REPO_ROOT / ".git" / "precommit-success.json"


def _git(*args: str) -> bytes:
    return subprocess.check_output(("git", *args), cwd=REPO_ROOT)


def _repo_signature() -> str:
    digest = hashlib.sha256()
    paths = _git("ls-files", "-z", "--cached", "--others", "--exclude-standard")
    for raw_path in paths.split(b"\0"):
        if not raw_path:
            continue
        path = Path(raw_path.decode("utf-8", errors="surrogateescape"))
        absolute = REPO_ROOT / path
        digest.update(b"path\0")
        digest.update(raw_path)
        digest.update(b"\0")
        if not absolute.exists() or not absolute.is_file():
            digest.update(b"missing\0")
            continue
        digest.update(str(absolute.stat().st_mode & 0o777).encode())
        digest.update(b"\0")
        digest.update(absolute.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def record() -> int:
    CACHE_PATH.write_text(
        json.dumps(
            {
                "signature": _repo_signature(),
                "recorded_at": int(time.time()),
                "head": _git("rev-parse", "HEAD").decode().strip(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Recorded fresh pre-commit result in {CACHE_PATH.relative_to(REPO_ROOT)}")
    return 0


def check() -> int:
    if not CACHE_PATH.exists():
        print(
            "No fresh pre-commit cache found. Run ./scripts/run_precommit.sh before pushing.",
            file=sys.stderr,
        )
        return 1
    cache = json.loads(CACHE_PATH.read_text(encoding="utf-8"))
    current = _repo_signature()
    if cache.get("signature") != current:
        print(
            "Pre-commit cache is stale for the current repo contents. "
            "Run ./scripts/run_precommit.sh before pushing.",
            file=sys.stderr,
        )
        return 1
    print("Pre-commit cache is fresh for the current repo contents.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("record", "check"))
    args = parser.parse_args()
    return record() if args.command == "record" else check()


if __name__ == "__main__":
    raise SystemExit(main())
