#!/usr/bin/env python3
"""Vendor TurtleBot3 Burger MJCF meshes from ROBOTIS MuJoCo Menagerie.

Copies STL assets into ``src/fret/mjcf/assets/turtlebot3/`` for the physics
unit model (``turtlebot3_burger.xml`` / ``turtlebot3_unit.xml``).

Source: https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie (Apache-2.0)

Example::

    python3 scripts/import_turtlebot3_assets.py
    python3 scripts/import_turtlebot3_assets.py --src /path/to/robotis_mujoco_menagerie
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_OUT = _REPO_ROOT / "src/fret/mjcf/assets/turtlebot3"

_MESHES: tuple[str, ...] = (
    "burger_base.stl",
    "left_tire.stl",
    "right_tire.stl",
    "lds.stl",
)


def import_assets(src_root: Path, out_dir: Path) -> None:
    """Copy TurtleBot3 Burger meshes into the FRET MJCF asset tree."""
    tb3_assets = src_root / "robotis_tb3" / "assets"
    if not tb3_assets.is_dir():
        raise FileNotFoundError(
            f"Expected TurtleBot3 assets at {tb3_assets}. "
            "Clone https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie"
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Writing meshes to {out_dir}")
    for name in _MESHES:
        src = tb3_assets / name
        if not src.is_file():
            raise FileNotFoundError(f"Missing mesh: {src}")
        shutil.copy2(src, out_dir / name)
        print(f"  {name}")

    license_src = src_root / "robotis_tb3" / "LICENSE"
    if license_src.is_file():
        shutil.copy2(license_src, out_dir / "LICENSE")

    readme = out_dir / "README.md"
    readme.write_text(
        "TurtleBot3 Burger MJCF + meshes from ROBOTIS MuJoCo Menagerie.\n"
        "https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie\n"
        "License: Apache-2.0\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="Path to robotis_mujoco_menagerie clone",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=_DEFAULT_OUT,
        help="Output directory for vendored meshes",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    src = args.src
    if src is None:
        tmp = Path("/tmp/robotis_mujoco_menagerie")
        if not tmp.is_dir():
            print(
                "Cloning robotis_mujoco_menagerie into /tmp ...",
                file=sys.stderr,
            )
            subprocess.run(
                [
                    "git",
                    "clone",
                    "--depth",
                    "1",
                    "https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie.git",
                    str(tmp),
                ],
                check=True,
            )
        src = tmp

    import_assets(src.resolve(), args.out.resolve())
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
