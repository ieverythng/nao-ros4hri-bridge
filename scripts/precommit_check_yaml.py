#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import yaml


def main(argv: list[str]) -> int:
    failed = False
    for name in argv[1:]:
        path = Path(name)
        if not path.is_file():
            continue

        try:
            with path.open("r", encoding="utf-8") as handle:
                list(yaml.safe_load_all(handle))
        except yaml.YAMLError as exc:
            print(f"{path}: invalid YAML: {exc}")
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
