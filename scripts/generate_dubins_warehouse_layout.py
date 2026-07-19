#!/usr/bin/env python3
"""Generate a multi-corridor warehouse maze YAML for the Dubins TB3 race.

Design goals (SC-v11):
  * ~four wide start→goal routes with early forks (not one forced weave)
  * aisle face-to-face gaps >= robot_width + 4*clearance (+ PP slack)
  * diagonal foils so the grey dummy's straight tracker still collides
  * varied AWS shelf / prop visuals without packing the free lanes

Corridor skeleton (center bands, ~1.26 m clear before inflation)::

    vertical aisles   x ≈ 2.0, 5.0, 8.0
    horizontal aisles y ≈ 2.0, 5.0, 8.0

Four solid shelf islands sit in the cells between aisles and block
through-cell shortcuts, so planners must pick aisle routes:

  1. south → east   (y≈2 then x≈8)
  2. west → north   (x≈2 then y≈8)
  3. south → mid → north  (y≈2, x≈5, y≈8)
  4. west → mid → east    (x≈2, y≈5, x≈8)

Example::

    python3 scripts/generate_dubins_warehouse_layout.py
    python3 scripts/generate_dubins_warehouse_layout.py --dry-run
    python3 scripts/generate_dubins_race_mjcf.py
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
_CLEARANCE_MARGIN_M = 0.30
_MIN_GAP_MARGIN_M = _ROBOT_WIDTH_M + 4.0 * _CLEARANCE_MARGIN_M
_MIN_GAP_SIZE_M = 3.0 * (2.0 * _ROBOT_RADIUS_M)
# Wider than the mathematical minimum so alternate routes feel open.
_CORRIDOR_GAP_M = max(_MIN_GAP_MARGIN_M, _MIN_GAP_SIZE_M) + 0.36  # ≈ 1.26 m

_SHELF_HEIGHT_M = 1.30
# Aisle pitch 3.0 m; island half-extent leaves a wide clear lane after
# occupancy inflation (vehicle_radius + clearance_margin = 0.42 m).
_ISLAND_HALF_M = 0.68  # face-to-face aisle gap ≈ 3.0 - 1.36 = 1.64 m


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
) -> None:
    if hx <= 0.0 or hy <= 0.0:
        return
    if (
        x - hx < 0.35
        or x + hx > _WORKSPACE_MAX_M - 0.35
        or y - hy < 0.35
        or y + hy > _WORKSPACE_MAX_M - 0.35
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
    """City-block islands with four wide aisle routes start→goal."""
    structures: list[dict[str, object]] = []

    # -----------------------------------------------------------------
    # Four shelf islands in the cells between the aisle grid.
    # Aisle free bands (approx): x,y ∈ [1.37–2.63], [4.37–5.63], [7.37–8.63]
    # Island centres ≈ (3.5, 3.5), (6.5, 3.5), (3.5, 6.5), (6.5, 6.5)
    # SW / NE islands sit on y=x and foil the grey dummy's straight chord.
    # -----------------------------------------------------------------
    islands: tuple[tuple[float, float, str, str, str], ...] = (
        # (cx, cy, main, accent_a, accent_b)
        (3.50, 3.50, "shelf", "clutter_a", "bucket"),  # SW — diagonal foil
        (6.50, 3.50, "shelf_e", "clutter_d", "trash"),  # SE
        (3.50, 6.50, "shelf_e", "clutter_c", "desk"),  # NW
        (6.50, 6.50, "shelf", "clutter_c", "pallet"),  # NE — diagonal foil
    )
    half = _ISLAND_HALF_M
    for cx, cy, main, accent_a, accent_b in islands:
        # Main shelf mass fills the cell so planners cannot cut through it.
        _add(structures, cx, cy, half, half * 0.55, _SHELF_HEIGHT_M, visual=main)
        # Cross-bar makes a compact "+" footprint (still one blocked cell).
        _add(
            structures,
            cx,
            cy,
            half * 0.45,
            half,
            _SHELF_HEIGHT_M,
            visual=main,
        )
        # Small accent props on the island corners for visual variety —
        # kept inside the island AABB so aisles stay clear.
        _add(
            structures,
            cx + half * 0.55,
            cy + half * 0.55,
            0.18,
            0.18,
            0.95,
            visual=accent_a,
        )
        _add(
            structures,
            cx - half * 0.55,
            cy - half * 0.55,
            0.14,
            0.14,
            0.70,
            visual=accent_b,
        )

    # Extra on-diagonal foil near the mid-cross. Sized so y=x is blocked
    # for the grey dummy while the orthogonal aisle centerlines (x=5, y=5)
    # stay free after inflation (clearance 0.42 m).
    _add(structures, 4.35, 4.35, 0.12, 0.12, 1.00, visual="clutter_a")
    _add(structures, 5.65, 5.65, 0.12, 0.12, 1.00, visual="clutter_d")

    # Corner / flank props — outside the three aisle bands for atmosphere.
    props: tuple[tuple[float, float, float, float, float, str], ...] = (
        (1.0, 4.0, 0.22, 0.18, 0.75, "desk"),  # west wall niche
        (4.0, 1.0, 0.18, 0.22, 0.70, "trash"),  # south wall niche
        (1.0, 9.0, 0.28, 0.18, 0.80, "pallet"),
        (9.0, 1.0, 0.18, 0.28, 0.90, "pallet"),
        (2.4, 9.2, 0.12, 0.12, 1.10, "lamp"),
        (9.2, 2.4, 0.12, 0.12, 1.10, "lamp"),
        (9.1, 6.5, 0.16, 0.16, 0.55, "bucket"),
        (6.5, 9.1, 0.18, 0.18, 0.70, "trash"),
        (9.0, 9.0, 0.22, 0.22, 0.85, "clutter_c"),
        (0.9, 6.5, 0.16, 0.22, 0.65, "desk"),
        (6.5, 0.9, 0.22, 0.16, 0.65, "bucket"),
    )
    for x, y, hx, hy, height, visual in props:
        _add(structures, x, y, hx, hy, height, visual=visual)

    return structures


def estimate_aisle_gap_m() -> float:
    """Return the designed clear aisle width (geometric, pre-inflation)."""
    # Pitch between island centres is 3.0 m; each island half-extent is
    # _ISLAND_HALF_M along the axis that faces the aisle.
    return float(3.0 - 2.0 * _ISLAND_HALF_M)


def build_world(structures: list[dict[str, object]]) -> dict[str, object]:
    return {
        "workspace_bounds": {
            "x": [0.0, _WORKSPACE_MAX_M],
            "y": [0.0, _WORKSPACE_MAX_M],
            "z": [0.0, 2.5],
        },
        "start_xy": list(_START),
        "goal_xy": list(_GOAL),
        "agent_lateral_offset": 0.55,
        "vehicle": {
            "radius": _ROBOT_RADIUS_M,
            "clearance_margin": _CLEARANCE_MARGIN_M,
            "width": _ROBOT_WIDTH_M,
            "min_corridor_gap": round(estimate_aisle_gap_m(), 3),
        },
        "planner": {
            "bounds": [[0.0, _WORKSPACE_MAX_M], [0.0, _WORKSPACE_MAX_M]],
            "rrt_max_sample_count": 7000,
            "sst_max_sample_count": 8000,
            "step_size": 0.25,
            "goal_tolerance": 0.30,
            "collision_check_count": 40,
            "goal_bias": 0.18,
            "witness_radius": 0.15,
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
    gap = estimate_aisle_gap_m()
    text = yaml.safe_dump(world, sort_keys=False)
    header = (
        "# Dubins race warehouse maze (SC-v11) — real TurtleBot3 Burger.\n"
        "#\n"
        "# Multi-corridor city-block layout: ~4 wide start→goal routes with\n"
        "# early forks (south→east, west→north, mid-aisle hybrids). Aisle\n"
        f"# clear gap (island face-to-face): {gap:.2f} m "
        f"(>= width {_ROBOT_WIDTH_M:.2f} + 4*clearance "
        f"{_CLEARANCE_MARGIN_M:.2f}). SW/NE islands foil the grey dummy.\n"
        "# Regenerate with:\n"
        "#   python3 scripts/generate_dubins_warehouse_layout.py\n"
        "#   python3 scripts/generate_dubins_race_mjcf.py\n"
        "#\n"
        "# vehicle.clearance_margin 0.18 -> 0.30: prior physics-mode tuning\n"
        "# validated against multi-seed collision rates; see git history.\n"
    )
    payload = header + text
    if args.dry_run:
        print(payload)
        print(
            f"# structures: {len(structures)} aisle_gap={gap:.3f}m",
            file=sys.stderr,
        )
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(payload, encoding="utf-8")
    print(
        f"Wrote {len(structures)} structures to {args.out} "
        f"(aisle gap {gap:.3f} m)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
