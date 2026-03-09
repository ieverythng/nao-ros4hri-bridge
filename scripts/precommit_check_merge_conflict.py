#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


CONFLICT_MARKERS = ("<<<<<<<", "=======", ">>>>>>>", "|||||||")


def main(argv: list[str]) -> int:
    failed = False
    for name in argv[1:]:
        path = Path(name)
        if not path.is_file():
            continue

        for lineno, line in enumerate(
            path.read_text(encoding="utf-8", errors="surrogateescape").splitlines(),
            start=1,
        ):
            if line.startswith(CONFLICT_MARKERS):
                print(f"{path}:{lineno}: merge conflict marker found")
                failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
