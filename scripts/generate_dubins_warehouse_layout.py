#!/usr/bin/env python3
"""Generate compact Dubins race obstacle YAML for real TurtleBot3 speeds.

Produces a 10 m × 10 m lab floor with narrow but navigable corridors for
ROBOTIS TurtleBot3 Burger (planning radius 0.12 m + clearance 0.18 m).

Example::

    python3 scripts/generate_dubins_warehouse_layout.py
    python3 scripts/generate_dubins_warehouse_layout.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_OUT = _REPO_ROOT / "src/fret/config/worlds/dubins_race_obstacles.yml"

_WORKSPACE_MAX_M = 10.0
_BOUNDS_PAD_M = 0.85
_BOUNDS_MAX_M = _WORKSPACE_MAX_M - 0.15


def _dist_to_diagonal(x: float, y: float) -> float:
    return abs(y - x) / math.sqrt(2.0)


def _normal(side: float) -> tuple[float, float]:
    inv_sqrt2 = 0.70710678
    return -inv_sqrt2 * side, inv_sqrt2 * side


def _add(
    structures: list[dict[str, float]],
    x: float,
    y: float,
    hx: float,
    hy: float,
    height: float,
    *,
    min_diagonal_offset: float = 0.0,
    avoid_pads: bool = True,
) -> None:
    if (
        min_diagonal_offset > 0.0
        and _dist_to_diagonal(x, y) < min_diagonal_offset
    ):
        return
    if avoid_pads:
        # Keep spawn / goal disks clear (centre + ~0.9 m).
        if math.hypot(x - 1.2, y - 1.2) < 1.1:
            return
        if math.hypot(x - 8.8, y - 8.8) < 1.1:
            return
    if (
        x - hx < _BOUNDS_PAD_M
        or x + hx > _BOUNDS_MAX_M
        or y - hy < _BOUNDS_PAD_M
        or y + hy > _BOUNDS_MAX_M
    ):
        return
    structures.append(
        {
            "x": round(x, 2),
            "y": round(y, 2),
            "hx": round(hx, 2),
            "hy": round(hy, 2),
            "height": round(height, 2),
        }
    )


def generate_structures() -> list[dict[str, float]]:
    """Staggered diagonal chokes + flank racks with ~0.8 m corridors."""
    structures: list[dict[str, float]] = []

    # Cheek barriers: offset from the diagonal so a ~0.8–1.0 m gap remains.
    # Alternating sides force a weave without sealing the route.
    cheeks: tuple[tuple[float, float, float, float, float, float], ...] = (
        # station, hx, hy, height, side, lateral_from_diagonal
        (3.0, 0.55, 0.22, 1.15, 1.0, 0.70),
        (4.2, 0.22, 0.55, 1.20, -1.0, 0.70),
        (5.4, 0.55, 0.22, 1.25, 1.0, 0.72),
        (6.6, 0.22, 0.55, 1.20, -1.0, 0.72),
        (7.6, 0.50, 0.22, 1.15, 1.0, 0.68),
    )
    for station, hx, hy, height, side, lateral in cheeks:
        nx, ny = _normal(side)
        _add(
            structures,
            station + nx * lateral,
            station + ny * lateral,
            hx,
            hy,
            height,
        )

    # Flank storage — further off-diagonal to narrow side pockets.
    for i, station in enumerate((2.8, 4.0, 5.2, 6.4, 7.4)):
        side = 1.0 if i % 2 == 0 else -1.0
        offset = 1.35 + (i % 2) * 0.15
        nx, ny = _normal(side)
        _add(
            structures,
            station + nx * offset,
            station + ny * offset,
            0.45,
            0.16,
            1.1 + 0.05 * (i % 3),
        )
        _add(
            structures,
            station - nx * (offset + 0.25),
            station - ny * (offset + 0.25),
            0.40,
            0.16,
            1.05,
        )

    # Small crates in the pockets (not on the race line).
    crates: tuple[tuple[float, float, float, float, float], ...] = (
        (2.5, 4.8, 0.22, 0.20, 0.9),
        (4.8, 2.6, 0.20, 0.22, 0.95),
        (5.0, 7.2, 0.22, 0.20, 1.0),
        (7.2, 5.0, 0.20, 0.22, 0.95),
        (3.6, 6.8, 0.18, 0.18, 0.85),
        (6.8, 3.6, 0.18, 0.18, 0.85),
    )
    for x, y, hx, hy, height in crates:
        _add(structures, x, y, hx, hy, height, min_diagonal_offset=1.0)

    return structures


def build_world(structures: list[dict[str, float]]) -> dict[str, object]:
    return {
        "workspace_bounds": {
            "x": [0.0, _WORKSPACE_MAX_M],
            "y": [0.0, _WORKSPACE_MAX_M],
            "z": [0.0, 2.5],
        },
        "start_xy": [1.2, 1.2],
        "goal_xy": [8.8, 8.8],
        "agent_lateral_offset": 0.28,
        "vehicle": {
            "radius": 0.12,
            "clearance_margin": 0.18,
        },
        "planner": {
            "bounds": [[0.0, _WORKSPACE_MAX_M], [0.0, _WORKSPACE_MAX_M]],
            "rrt_max_sample_count": 5000,
            "sst_max_sample_count": 6000,
            "step_size": 0.16,
            "goal_tolerance": 0.28,
            "collision_check_count": 24,
            "goal_bias": 0.15,
            "witness_radius": 0.10,
            "enable_pruning": True,
        },
        "structures": structures,
        "dead_ends": [
            {
                "id": "alcove_w",
                "parts": [
                    {
                        "x": 1.6,
                        "y": 3.8,
                        "hx": 0.12,
                        "hy": 0.55,
                        "height": 1.05,
                    },
                    {
                        "x": 1.25,
                        "y": 4.25,
                        "hx": 0.35,
                        "hy": 0.12,
                        "height": 1.05,
                    },
                    {
                        "x": 1.25,
                        "y": 3.35,
                        "hx": 0.35,
                        "hy": 0.12,
                        "height": 1.05,
                    },
                ],
            },
            {
                "id": "alcove_e",
                "parts": [
                    {
                        "x": 8.4,
                        "y": 6.2,
                        "hx": 0.12,
                        "hy": 0.55,
                        "height": 1.05,
                    },
                    {
                        "x": 8.75,
                        "y": 6.65,
                        "hx": 0.35,
                        "hy": 0.12,
                        "height": 1.05,
                    },
                    {
                        "x": 8.75,
                        "y": 5.75,
                        "hx": 0.35,
                        "hy": 0.12,
                        "height": 1.05,
                    },
                ],
            },
        ],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=_DEFAULT_OUT)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print YAML to stdout instead of writing the file",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    structures = generate_structures()
    world = build_world(structures)
    header = (
        "# Dubins race lab layout (SC-v11) — real TurtleBot3 Burger scale.\n"
        "#\n"
        "# 10 m × 10 m floor with narrow corridors sized for TB3\n"
        "# (radius 0.12 m + clearance 0.18 m). Regenerate with:\n"
        "#   python3 scripts/generate_dubins_warehouse_layout.py\n"
        "#   python3 scripts/generate_dubins_race_mjcf.py\n"
        "#\n"
        "# Structure format: {x, y, hx, hy, height} — centre + half-extents (m).\n\n"
    )
    body = yaml.dump(
        world,
        default_flow_style=False,
        sort_keys=False,
        allow_unicode=True,
    )
    text = header + body
    if args.dry_run:
        print(text)
        print(f"# base structures: {len(structures)}", file=sys.stderr)
        return 0

    args.out.write_text(text, encoding="utf-8")
    print(f"Wrote {len(structures)} base structures to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
