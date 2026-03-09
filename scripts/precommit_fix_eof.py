#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


def _target_newline(data: bytes) -> bytes:
    if b"\r\n" in data and data.count(b"\r\n") >= data.count(b"\n") - data.count(b"\r\n"):
        return b"\r\n"
    return b"\n"


def main(argv: list[str]) -> int:
    changed = False
    for name in argv[1:]:
        path = Path(name)
        if not path.is_file():
            continue

        data = path.read_bytes()
        if not data or b"\0" in data:
            continue

        desired = data.rstrip(b"\r\n") + _target_newline(data)
        if desired != data:
            path.write_bytes(desired)
            print(f"fixed end of file: {path}")
            changed = True

    return 1 if changed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
