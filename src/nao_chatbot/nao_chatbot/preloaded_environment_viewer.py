"""Print or open the installed preloaded-environment SVG viewer."""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path

try:  # pragma: no cover - host-side convenience when ROS is not sourced
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # pragma: no cover
    PackageNotFoundError = RuntimeError
    get_package_share_directory = None


VIEWER_RELATIVE_PATH = Path("config", "preloaded_environment_viewer.html")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--open",
        action="store_true",
        help="Try to open the viewer with the container's default browser.",
    )
    args = parser.parse_args(argv)

    try:
        if get_package_share_directory is None:
            raise PackageNotFoundError()
        viewer_path = Path(get_package_share_directory("nao_chatbot"), VIEWER_RELATIVE_PATH)
    except PackageNotFoundError:
        viewer_path = Path(__file__).resolve().parent.parent / VIEWER_RELATIVE_PATH

    if not viewer_path.exists():
        print("Preloaded environment viewer not found: %s" % viewer_path, file=sys.stderr)
        return 2

    viewer_url = viewer_path.resolve().as_uri()
    print(viewer_url)
    if args.open:
        return 0 if webbrowser.open(viewer_url) else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
