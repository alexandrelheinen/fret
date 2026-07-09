#!/usr/bin/env python3
"""Generate warehouse-style Dubins race obstacle YAML.

Produces a lower-density industrial layout: rack rows and pallet clusters
along the A→B corridor with a clear central aisle (≥ 3.5 m from structure
faces to the diagonal race line).

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
_DEFAULT_OUT = (
    _REPO_ROOT / "src/fret/config/worlds/dubins_race_obstacles.yml"
)

# Minimum perpendicular distance from structure centre to diagonal y = x [m].
_MIN_DIAGONAL_OFFSET_M = 4.2


def _dist_to_diagonal(x: float, y: float) -> float:
    return abs(y - x) / math.sqrt(2.0)


def _add(
    structures: list[dict[str, float]],
    x: float,
    y: float,
    hx: float,
    hy: float,
    height: float,
) -> None:
    if _dist_to_diagonal(x, y) < _MIN_DIAGONAL_OFFSET_M:
        return
    if x - hx < 6.0 or x + hx > 78.0 or y - hy < 6.0 or y + hy > 78.0:
        return
    structures.append(
        {
            "x": round(x, 1),
            "y": round(y, 1),
            "hx": round(hx, 2),
            "hy": round(hy, 2),
            "height": round(height, 1),
        }
    )


def generate_structures() -> list[dict[str, float]]:
    """Build rack rows, pallet stations, and cross-aisle clutter."""
    structures: list[dict[str, float]] = []

    # Rack pairs flanking the diagonal corridor (warehouse aisles).
    for i, station in enumerate(range(14, 70, 7)):
        side = 1.0 if i % 2 == 0 else -1.0
        offset = 7.0 + (i % 3) * 1.2
        nx = -0.70710678 * side
        ny = 0.70710678 * side
        height = 2.4 + (i % 4) * 0.25
        rack_hx = 2.8 + (i % 2) * 0.4
        _add(
            structures,
            station + nx * offset,
            station + ny * offset,
            rack_hx,
            0.45,
            height,
        )
        _add(
            structures,
            station - nx * (offset + 1.5),
            station - ny * (offset + 1.5),
            rack_hx,
            0.45,
            height + 0.2,
        )

    # Pallet / crate clusters between rack rows.
    for i, station in enumerate(range(18, 66, 9)):
        side = -1.0 if i % 2 == 0 else 1.0
        nx = -0.70710678 * side
        ny = 0.70710678 * side
        offset = 5.2
        height = 1.9 + (i % 3) * 0.35
        _add(
            structures,
            station + nx * offset,
            station + ny * offset,
            0.95,
            0.85,
            height,
        )

    # Peripheral storage blocks (force detours without sealing the map).
    peripheral = (
        (22.0, 52.0, 1.2, 1.0, 2.2),
        (52.0, 22.0, 1.0, 1.2, 2.1),
        (30.0, 58.0, 1.1, 0.9, 2.0),
        (58.0, 30.0, 0.9, 1.1, 2.3),
        (64.0, 44.0, 1.0, 0.95, 2.1),
        (44.0, 64.0, 0.95, 1.0, 2.4),
    )
    for x, y, hx, hy, height in peripheral:
        _add(structures, x, y, hx, hy, height)

    return structures


def build_world(structures: list[dict[str, float]]) -> dict[str, object]:
    return {
        "workspace_bounds": {
            "x": [0.0, 80.0],
            "y": [0.0, 80.0],
            "z": [0.0, 4.0],
        },
        "start_xy": [6.0, 6.0],
        "goal_xy": [74.0, 74.0],
        "agent_lateral_offset": 0.8,
        "vehicle": {
            "radius": 0.42,
            "clearance_margin": 0.50,
        },
        "planner": {
            "bounds": [[0.0, 80.0], [0.0, 80.0]],
            "rrt_max_sample_count": 6000,
            "sst_max_sample_count": 6000,
            "step_size": 0.45,
            "goal_tolerance": 0.65,
            "collision_check_count": 24,
            "goal_bias": 0.10,
            "witness_radius": 0.25,
            "enable_pruning": True,
        },
        "structures": structures,
        "dead_ends": [
            {
                "id": "alcove_sw",
                "parts": [
                    {"x": 24.0, "y": 16.0, "hx": 0.35, "hy": 2.20, "height": 2.6},
                    {"x": 22.4, "y": 18.0, "hx": 1.60, "hy": 0.35, "height": 2.6},
                    {"x": 22.4, "y": 14.0, "hx": 1.60, "hy": 0.35, "height": 2.6},
                ],
            },
            {
                "id": "alcove_mid",
                "parts": [
                    {"x": 42.0, "y": 40.5, "hx": 2.80, "hy": 0.35, "height": 2.8},
                    {"x": 44.5, "y": 38.8, "hx": 0.35, "hy": 1.50, "height": 2.8},
                    {"x": 39.5, "y": 38.8, "hx": 0.35, "hy": 1.50, "height": 2.8},
                ],
            },
            {
                "id": "alcove_ne",
                "parts": [
                    {"x": 58.0, "y": 54.0, "hx": 0.35, "hy": 2.40, "height": 2.4},
                    {"x": 56.2, "y": 56.0, "hx": 1.80, "hy": 0.35, "height": 2.4},
                    {"x": 56.2, "y": 52.0, "hx": 1.80, "hy": 0.35, "height": 2.4},
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
        "# Dubins race warehouse layout (SC-v11 / T11-02).\n"
        "#\n"
        "# 80 m × 80 m industrial floor with rack rows, pallet clusters, and\n"
        "# U-shaped dead-end alcoves.  Regenerate with:\n"
        "#   python3 scripts/generate_dubins_warehouse_layout.py\n"
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
    sys.exit(main())
