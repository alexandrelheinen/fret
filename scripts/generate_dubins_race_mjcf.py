#!/usr/bin/env python3
"""Generate Dubins race MJCF structure geoms from obstacle YAML.

Keeps ``dubins_race_obstacles.yml`` as the planning source of truth and
writes collision boxes plus AWS clutter mesh visuals into ``dubins_race.xml``.

Example::

    python3 scripts/generate_dubins_race_mjcf.py
    python3 scripts/generate_dubins_race_mjcf.py --dry-run
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_YAML = (
    _REPO_ROOT / "src/fret/config/worlds/dubins_race_obstacles.yml"
)
_DEFAULT_MJCF = _REPO_ROOT / "src/fret/mjcf/dubins_race.xml"

# Native OBJ extents (meters) after ``import_aws_warehouse_assets.py``.
_CLUTTER_EXTENTS: dict[str, tuple[float, float, float]] = {
    "clutter_a": (2.717, 2.51, 1.339),
    "clutter_c": (2.167, 1.582, 1.8),
    "clutter_d": (1.831, 1.034, 1.778),
}

_STRUCTURES_BEGIN = "    <!-- Warehouse structure block (auto-generated) -->"
_STRUCTURES_END = "    <!-- End warehouse structure block -->"


def _tier_mesh(height: float) -> str:
    if height <= 2.2:
        return "clutter_a"
    if height <= 3.0:
        return "clutter_c"
    return "clutter_d"


def _tier_material(mesh: str) -> str:
    return {
        "clutter_a": "clutter_cardboard",
        "clutter_c": "clutter_crate",
        "clutter_d": "clutter_pallet",
    }[mesh]


def _mesh_scale(mesh: str, hx: float, hy: float, height: float) -> tuple[float, float, float]:
    ex, ey, ez = _CLUTTER_EXTENTS[mesh]
    return (2.0 * hx / ex, 2.0 * hy / ey, height / ez)


def _load_structures(yaml_path: Path) -> list[dict[str, float]]:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    structures = list(data.get("structures", []))
    for alcove in data.get("dead_ends", []):
        structures.extend(alcove.get("parts", []))
    return structures


def generate_structure_geoms(structures: list[dict[str, float]]) -> str:
    """Return MJCF geoms for analytic collision + clutter mesh visuals."""
    lines: list[str] = [_STRUCTURES_BEGIN]
    for idx, spec in enumerate(structures):
        x = float(spec["x"])
        y = float(spec["y"])
        hx = float(spec["hx"])
        hy = float(spec["hy"])
        height = float(spec["height"])
        half_z = height / 2.0
        mesh = _tier_mesh(height)
        material = _tier_material(mesh)
        sx, sy, sz = _mesh_scale(mesh, hx, hy, height)
        name = f"str_{idx:03d}"
        lines.append(
            f'    <geom name="{name}_col" class="structure_col" '
            f'pos="{x} {y} {half_z:.2f}" size="{hx} {hy} {half_z:.2f}"/>'
        )
        corner_x = x - hx
        corner_y = y - hy
        lines.append(
            f'    <geom name="{name}_vis" type="mesh" mesh="{mesh}" '
            f'pos="{corner_x:.3f} {corner_y:.3f} 0" '
            f'size="{sx:.4f} {sy:.4f} {sz:.4f}" material="{material}" '
            f'contype="0" conaffinity="0"/>'
        )
    lines.append(_STRUCTURES_END)
    return "\n".join(lines)


def patch_mjcf(mjcf_path: Path, structure_block: str) -> str:
    """Replace the auto-generated structure block inside dubins_race.xml."""
    text = mjcf_path.read_text(encoding="utf-8")
    pattern = re.compile(
        re.escape(_STRUCTURES_BEGIN) + r".*?" + re.escape(_STRUCTURES_END),
        re.DOTALL,
    )
    if not pattern.search(text):
        raise ValueError(
            f"Structure block markers not found in {mjcf_path}. "
            "Add the begin/end comments before running this script."
        )
    return pattern.sub(structure_block, text)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", type=Path, default=_DEFAULT_YAML)
    parser.add_argument("--mjcf", type=Path, default=_DEFAULT_MJCF)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print generated block without writing MJCF",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    structures = _load_structures(args.yaml.resolve())
    block = generate_structure_geoms(structures)
    if args.dry_run:
        print(block)
        print(f"# structures: {len(structures)}", file=sys.stderr)
        return 0

    updated = patch_mjcf(args.mjcf.resolve(), block)
    args.mjcf.write_text(updated, encoding="utf-8")
    print(f"Wrote {len(structures)} structures into {args.mjcf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
