#!/usr/bin/env python3
"""Resolve TurtleBot3 Burger meshes from the ROBOTIS Menagerie submodule.

FRET no longer vendors STL copies under ``mjcf/assets/turtlebot3/``.
MJCF files reference::

    third_party/robotis_mujoco_menagerie/robotis_tb3/assets/

This script verifies the submodule is present and lists the meshes in use.

Source: https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie (Apache-2.0)

Example::

    git submodule update --init third_party/robotis_mujoco_menagerie
    python3 scripts/import_turtlebot3_assets.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_SRC = _REPO_ROOT / "third_party" / "robotis_mujoco_menagerie"

_MESHES: tuple[str, ...] = (
    "burger_base.stl",
    "left_tire.stl",
    "right_tire.stl",
    "lds.stl",
)


def verify_assets(src_root: Path) -> Path:
    """Return the TB3 assets directory after checking required meshes."""
    tb3_assets = src_root / "robotis_tb3" / "assets"
    if not tb3_assets.is_dir():
        raise FileNotFoundError(
            f"Expected TurtleBot3 assets at {tb3_assets}. "
            "Init submodule: git submodule update --init "
            "third_party/robotis_mujoco_menagerie"
        )
    for name in _MESHES:
        path = tb3_assets / name
        if not path.is_file():
            raise FileNotFoundError(f"Missing mesh: {path}")
    return tb3_assets


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="Path to robotis_mujoco_menagerie (default: submodule)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    src = args.src if args.src is not None else _DEFAULT_SRC
    if not src.is_dir():
        print(
            f"Menagerie source not found at {src}.\n"
            "Run: git submodule update --init "
            "third_party/robotis_mujoco_menagerie",
            file=sys.stderr,
        )
        return 1
    assets = verify_assets(src.resolve())
    print(f"TurtleBot3 Burger meshes OK at {assets}")
    for name in _MESHES:
        print(f"  {name}")
    print("MJCF should meshdir to this folder (no local STL copies).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
