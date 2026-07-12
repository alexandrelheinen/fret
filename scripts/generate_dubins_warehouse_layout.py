#!/usr/bin/env python3
"""Generate warehouse-style Dubins race obstacle YAML.

Produces an industrial layout where rack rows and pallet clusters flank the
A→B race line, with deliberate diagonal barriers that block the trivial
straight route while leaving multiple viable detour corridors.

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

# Keep peripheral clutter off the spawn/goal pads.
_BOUNDS_PAD_M = 6.0
_BOUNDS_MAX_M = 78.0

# Flank racks sit this far from the diagonal (perpendicular) [m].
_FLANK_OFFSET_M = 3.4


def _dist_to_diagonal(x: float, y: float) -> float:
    return abs(y - x) / math.sqrt(2.0)


def _on_diagonal(station: float) -> tuple[float, float]:
    """Return a point on the race diagonal y = x."""
    return station, station


def _normal(side: float) -> tuple[float, float]:
    """Unit normal to the diagonal; ``side`` flips north/south of the line."""
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
) -> None:
    if min_diagonal_offset > 0.0 and _dist_to_diagonal(x, y) < min_diagonal_offset:
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
            "x": round(x, 1),
            "y": round(y, 1),
            "hx": round(hx, 2),
            "hy": round(hy, 2),
            "height": round(height, 1),
        }
    )


def _add_flank_rack(
    structures: list[dict[str, float]],
    station: float,
    side: float,
    offset: float,
    rack_hx: float,
    height: float,
) -> None:
    nx, ny = _normal(side)
    cx = station + nx * offset
    cy = station + ny * offset
    _add(structures, cx, cy, rack_hx, 0.45, height)


def _add_diagonal_barrier(
    structures: list[dict[str, float]],
    station: float,
    hx: float,
    hy: float,
    height: float,
    *,
    side: float = 0.0,
    lateral: float = 0.0,
) -> None:
    """Place a barrier that crosses or grazes the race diagonal."""
    cx, cy = _on_diagonal(station)
    if side != 0.0:
        nx, ny = _normal(side)
        cx += nx * lateral
        cy += ny * lateral
    _add(structures, cx, cy, hx, hy, height)


def generate_structures() -> list[dict[str, float]]:
    """Build rack rows, pallet stations, and cross-aisle clutter."""
    structures: list[dict[str, float]] = []

    # Diagonal barriers with staggered gaps — block the trivial straight route
    # while leaving both a northern (y > x) and southern (y < x) corridor.
    diagonal_barriers: tuple[tuple[float, float, float, float, float, float, float], ...] = (
        # station, hx, hy, height, side, lateral
        (18.0, 2.40, 2.40, 2.6, 0.0, 0.0),  # early full block → detour north or south
        (28.0, 2.80, 1.20, 2.8, 1.0, 1.6),  # north cheek; gap to the south
        (36.0, 1.20, 2.80, 2.7, -1.0, 1.6),  # south cheek; gap to the north
        (46.0, 2.60, 2.60, 2.9, 0.0, 0.0),  # mid choke — forces side choice
        (54.0, 2.80, 1.20, 2.8, -1.0, 1.6),  # south cheek
        (62.0, 1.20, 2.80, 2.7, 1.0, 1.6),  # north cheek
        (70.0, 2.20, 2.20, 2.5, 0.0, 0.0),  # late block before goal pad
    )
    for station, hx, hy, height, side, lateral in diagonal_barriers:
        _add_diagonal_barrier(
            structures,
            station,
            hx,
            hy,
            height,
            side=side,
            lateral=lateral,
        )

    # Rack pairs flanking the diagonal — close enough to narrow corridors but
    # not a duplicate of the central barriers.
    for i, station in enumerate(range(14, 70, 7)):
        side = 1.0 if i % 2 == 0 else -1.0
        offset = _FLANK_OFFSET_M + (i % 3) * 0.8
        height = 2.4 + (i % 4) * 0.25
        rack_hx = 2.6 + (i % 2) * 0.4
        _add_flank_rack(structures, station, side, offset, rack_hx, height)
        _add_flank_rack(
            structures,
            station,
            -side,
            offset + 1.2,
            rack_hx,
            height + 0.2,
        )

    # Pallet / crate clusters between rack rows, biased toward alternate routes.
    pallet_stations: tuple[tuple[float, float, float, float], ...] = (
        (22.0, 1.0, 5.0, 1.9),  # station, side, offset, height
        (30.0, -1.0, 4.6, 2.2),
        (40.0, 1.0, 5.4, 2.5),
        (48.0, -1.0, 4.8, 2.0),
        (58.0, 1.0, 5.2, 2.3),
        (66.0, -1.0, 4.4, 2.6),
    )
    for station, side, offset, height in pallet_stations:
        nx, ny = _normal(side)
        _add(
            structures,
            station + nx * offset,
            station + ny * offset,
            0.95,
            0.85,
            height,
        )

    # Peripheral storage blocks — pin corners without sealing the map.
    peripheral = (
        (20.0, 50.0, 1.2, 1.0, 2.2),
        (50.0, 20.0, 1.0, 1.2, 2.1),
        (28.0, 60.0, 1.1, 0.9, 2.0),
        (60.0, 28.0, 0.9, 1.1, 2.3),
        (66.0, 46.0, 1.0, 0.95, 2.1),
        (46.0, 66.0, 0.95, 1.0, 2.4),
    )
    for x, y, hx, hy, height in peripheral:
        _add(structures, x, y, hx, hy, height, min_diagonal_offset=2.5)

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
            "clearance_margin": 1.00,
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
                    {"x": 20.0, "y": 12.0, "hx": 0.35, "hy": 2.20, "height": 2.6},
                    {"x": 18.4, "y": 14.0, "hx": 1.60, "hy": 0.35, "height": 2.6},
                    {"x": 18.4, "y": 10.0, "hx": 1.60, "hy": 0.35, "height": 2.6},
                ],
            },
            {
                "id": "alcove_mid",
                "parts": [
                    {"x": 44.0, "y": 36.0, "hx": 2.80, "hy": 0.35, "height": 2.8},
                    {"x": 46.5, "y": 34.2, "hx": 0.35, "hy": 1.50, "height": 2.8},
                    {"x": 41.5, "y": 34.2, "hx": 0.35, "hy": 1.50, "height": 2.8},
                ],
            },
            {
                "id": "alcove_ne",
                "parts": [
                    {"x": 60.0, "y": 66.0, "hx": 0.35, "hy": 2.40, "height": 2.4},
                    {"x": 58.2, "y": 68.0, "hx": 1.80, "hy": 0.35, "height": 2.4},
                    {"x": 58.2, "y": 64.0, "hx": 1.80, "hy": 0.35, "height": 2.4},
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
