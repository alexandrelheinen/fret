#!/usr/bin/env python3
"""Generate a navigable warehouse maze YAML for the Dubins TB3 race.

Corridor face-to-face gaps satisfy::

    gap >= robot_width + 4 * clearance_margin
    gap >= 3 * robot_diameter

with ``robot_width = 2 * half_width`` (0.18 m) and ``clearance_margin = 0.18 m``.
Implemented corridor target: **1.14 m** (minimum + 0.24 m PP slack).

Layout: staggered shelf cheeks along the start→goal diagonal (weave) plus a
on-diagonal clutter block so the grey dummy's straight path tracker collides,
while RRT*/SST keep a clear lane.

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
_START = (1.2, 1.2)
_GOAL = (8.8, 8.8)

_ROBOT_HALF_WIDTH_M = 0.09
_ROBOT_WIDTH_M = 2.0 * _ROBOT_HALF_WIDTH_M
_ROBOT_RADIUS_M = 0.12
# Raised 0.18 -> 0.30: see clearance_margin history in
# dubins_race_obstacles.yml — the smaller margin left a 40% multi-seed
# physics-mode collision rate. Regenerating this layout with the new
# margin still leaves the same tight cheek/on-diagonal-foil geometry
# untouched; widening specific gaps needs its own multi-seed validation
# pass, not just bumping this constant.
_CLEARANCE_MARGIN_M = 0.30
_MIN_GAP_MARGIN_M = _ROBOT_WIDTH_M + 4.0 * _CLEARANCE_MARGIN_M
_MIN_GAP_SIZE_M = 3.0 * (2.0 * _ROBOT_RADIUS_M)
_CORRIDOR_GAP_M = max(_MIN_GAP_MARGIN_M, _MIN_GAP_SIZE_M) + 0.24

_WALL_THICK_M = 0.30
_SHELF_HEIGHT_M = 1.30


def _dist_to_diagonal(x: float, y: float) -> float:
    return abs(y - x) / math.sqrt(2.0)


def _normal(side: float) -> tuple[float, float]:
    inv_sqrt2 = 0.70710678
    return -inv_sqrt2 * side, inv_sqrt2 * side


def _pad_clear(x: float, y: float, hx: float, hy: float) -> bool:
    for px, py in (_START, _GOAL):
        closest_x = min(max(px, x - hx), x + hx)
        closest_y = min(max(py, y - hy), y + hy)
        if math.hypot(closest_x - px, closest_y - py) < 1.15:
            return False
    return True


def _add(
    structures: list[dict[str, object]],
    x: float,
    y: float,
    hx: float,
    hy: float,
    height: float,
    *,
    visual: str,
    min_diagonal_offset: float = 0.0,
) -> None:
    if hx <= 0.0 or hy <= 0.0:
        return
    if (
        min_diagonal_offset > 0.0
        and _dist_to_diagonal(x, y) < min_diagonal_offset
    ):
        return
    if (
        x - hx < 0.45
        or x + hx > _WORKSPACE_MAX_M - 0.45
        or y - hy < 0.45
        or y + hy > _WORKSPACE_MAX_M - 0.45
    ):
        return
    if not _pad_clear(x, y, hx, hy):
        return
    structures.append(
        {
            "x": round(x, 3),
            "y": round(y, 3),
            "hx": round(hx, 3),
            "hy": round(hy, 3),
            "height": round(height, 3),
            "visual": visual,
        }
    )


def generate_structures() -> list[dict[str, object]]:
    """Staggered shelf cheeks with a guaranteed diagonal corridor width."""
    structures: list[dict[str, object]] = []
    # Face-to-face gap across the diagonal ≈ 2 * (lateral - thick/2).
    # Solve for lateral so gap == _CORRIDOR_GAP_M.
    half_thick = 0.5 * _WALL_THICK_M
    lateral = 0.5 * _CORRIDOR_GAP_M + half_thick

    cheeks: tuple[tuple[float, float, float, float, float], ...] = (
        # station along diagonal, hx, hy, height, side(+1/-1)
        (3.0, 0.55, half_thick, _SHELF_HEIGHT_M, 1.0),
        (4.2, half_thick, 0.55, _SHELF_HEIGHT_M, -1.0),
        (5.4, 0.55, half_thick, _SHELF_HEIGHT_M, 1.0),
        (6.6, half_thick, 0.55, _SHELF_HEIGHT_M, -1.0),
        (7.6, 0.50, half_thick, _SHELF_HEIGHT_M, 1.0),
    )
    for station, hx, hy, height, side in cheeks:
        nx, ny = _normal(side)
        _add(
            structures,
            station + nx * lateral,
            station + ny * lateral,
            hx,
            hy,
            height,
            visual="shelf",
        )

    # Outer flank shelves (further off-diagonal) — enrich the scene.
    for i, station in enumerate((3.2, 5.0, 6.8)):
        side = 1.0 if i % 2 == 0 else -1.0
        offset = lateral + 1.35
        nx, ny = _normal(side)
        _add(
            structures,
            station + nx * offset,
            station + ny * offset,
            0.45,
            0.16,
            1.15,
            visual="shelf_e",
        )

    # On-diagonal foil for the straight-line dummy (small, centered).
    _add(structures, 5.0, 5.0, 0.28, 0.28, 1.05, visual="clutter_c")

    # Props well off the race lane.
    props: tuple[tuple[float, float, float, float, float, str], ...] = (
        (2.0, 4.8, 0.18, 0.18, 0.55, "bucket"),
        (4.8, 2.0, 0.20, 0.20, 0.70, "trash"),
        (2.0, 7.5, 0.30, 0.18, 0.75, "desk"),
        (7.5, 2.0, 0.22, 0.30, 0.90, "pallet"),
        (3.2, 8.5, 0.14, 0.14, 1.10, "lamp"),
        (8.5, 3.2, 0.14, 0.14, 1.10, "lamp"),
        (8.2, 6.5, 0.18, 0.18, 0.55, "bucket"),
        (6.5, 8.2, 0.20, 0.20, 0.70, "trash"),
    )
    for x, y, hx, hy, height, visual in props:
        _add(
            structures,
            x,
            y,
            hx,
            hy,
            height,
            visual=visual,
            min_diagonal_offset=1.15,
        )

    return structures


def diagonal_corridor_gap_m(structures: list[dict[str, object]]) -> float:
    """Estimate face-to-face gap across the main diagonal from cheek shelves."""
    left = [
        _dist_to_diagonal(float(s["x"]), float(s["y"]))
        - min(float(s["hx"]), float(s["hy"]))
        for s in structures
        if str(s.get("visual", "")).startswith("shelf")
        and (float(s["y"]) - float(s["x"])) > 0.05
    ]
    right = [
        _dist_to_diagonal(float(s["x"]), float(s["y"]))
        - min(float(s["hx"]), float(s["hy"]))
        for s in structures
        if str(s.get("visual", "")).startswith("shelf")
        and (float(s["x"]) - float(s["y"])) > 0.05
    ]
    if not left or not right:
        return float("nan")
    return float(min(left) + min(right))


def build_world(structures: list[dict[str, object]]) -> dict[str, object]:
    return {
        "workspace_bounds": {
            "x": [0.0, _WORKSPACE_MAX_M],
            "y": [0.0, _WORKSPACE_MAX_M],
            "z": [0.0, 2.5],
        },
        "start_xy": list(_START),
        "goal_xy": list(_GOAL),
        # Keep RRT*/SST shells clear of the dummy on the diagonal (TB3 ≈ 0.18 m wide).
        "agent_lateral_offset": 0.55,
        "vehicle": {
            "radius": _ROBOT_RADIUS_M,
            "clearance_margin": _CLEARANCE_MARGIN_M,
            "width": _ROBOT_WIDTH_M,
            "min_corridor_gap": _CORRIDOR_GAP_M,
        },
        "planner": {
            "bounds": [[0.0, _WORKSPACE_MAX_M], [0.0, _WORKSPACE_MAX_M]],
            "rrt_max_sample_count": 7000,
            "sst_max_sample_count": 8000,
            "step_size": 0.20,
            "goal_tolerance": 0.30,
            "collision_check_count": 48,
            "goal_bias": 0.20,
            "witness_radius": 0.12,
            "enable_pruning": True,
        },
        "structures": structures,
        "dead_ends": [],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=_DEFAULT_OUT)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    structures = generate_structures()
    world = build_world(structures)
    gap = diagonal_corridor_gap_m(structures)
    text = yaml.safe_dump(world, sort_keys=False)
    header = (
        "# Dubins race warehouse maze (SC-v11) — real TurtleBot3 Burger.\n"
        "#\n"
        f"# Corridor gap target: {_CORRIDOR_GAP_M:.2f} m "
        f"(>= width {_ROBOT_WIDTH_M:.2f} + 4*clearance "
        f"{_CLEARANCE_MARGIN_M:.2f}, plus PP slack).\n"
        f"# Estimated diagonal face-to-face gap: "
        f"{gap if math.isfinite(gap) else float('nan'):.3f} m.\n"
        "# Regenerate with:\n"
        "#   python3 scripts/generate_dubins_warehouse_layout.py\n"
        "#   python3 scripts/generate_dubins_race_mjcf.py\n"
        "#\n"
    )
    payload = header + text
    if args.dry_run:
        print(payload)
        print(f"# structures: {len(structures)}", file=sys.stderr)
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(payload, encoding="utf-8")
    print(
        f"Wrote {len(structures)} structures to {args.out} "
        f"(diagonal gap {gap:.3f} m, target {_CORRIDOR_GAP_M:.2f} m)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
