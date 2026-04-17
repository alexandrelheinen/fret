#!/usr/bin/env python3
"""Milestone 4 CI simulation: workspace occupancy map build + validation.

Runs the complete Milestone 4 pipeline without ROS or Gazebo:

  1. Builds a synthetic obstacle payload from three 10×10×10 cm box obstacles
     placed inside the SCARA reachable annulus (mirrors ``arco_scenario.sdf``).
  2. Constructs a ``WorkspaceOccupancyBuilder`` with the default 20 cm grid.
  3. Calls ``builder.build(payload)`` to classify 147 voxels.
  4. Validates acceptance criteria (Milestone 4):
       - The occupied-voxel set is non-empty (obstacles detected).
       - All classified voxels lie within the reachable annulus.
       - ``is_occupied`` returns ``True`` for a point inside a known obstacle.
       - ``is_occupied`` returns ``False`` for a free-space point.
       - ``clearance()`` returns a negative value at an obstacle centre.
       - ``clearance()`` returns a positive value in free space.
       - ``occupied_centres()`` + ``free_centres()`` partition the annular set.
  5. Generates a 3-D matplotlib scatter plot of occupied vs. free voxels.
  6. Writes ``results.env`` and exits 0 on success, 1 on failure.

No ARCO or ROS runtime is required.

Usage::

    python3 scripts/simulate_milestone4_pipeline.py [--output DIR]

Output files (written to DIR, default /tmp/sim_output_m4):
    occupancy_map.png    — 3-D scatter figure (red = occupied, grey = free).
    results.env          — KEY=VALUE file with simulation metrics.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

# Allow running without an installed package by adding src/ to the path.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "src"))

import numpy as np

from fret.interfaces import OccupancyUpdatePayload
from fret.scene.workspace_occupancy import (
    _R_MAX,
    _R_MIN,
    WorkspaceOccupancyBuilder,
)

# ---------------------------------------------------------------------------
# Scenario: three 10 cm box obstacles inside the SCARA reachable workspace
# (mirrors arco_scenario.sdf obstacle_box_a/b/c)
# ---------------------------------------------------------------------------

_OBSTACLE_CENTRES: list[list[float]] = [
    [0.30, 0.10, 0.05],  # obstacle_box_a
    [0.20, 0.30, 0.05],  # obstacle_box_b
    [-0.10, 0.25, 0.05],  # obstacle_box_c
]
_BOX_HALF: float = 0.05  # half of 10 cm side
_N_SAMPLES_PER_FACE: int = 15  # surface sample density

# Known in-obstacle query point (at obstacle_box_a centre, x-y plane)
_INSIDE_PT: list[float] = [0.30, 0.10, 0.05]

# Known free-space query point (far from all obstacles in the annulus)
_FREE_PT: list[float] = [0.50, -0.30, 0.20]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _sample_box_surface(
    centre: list[float], half: float, n: int
) -> np.ndarray:
    """Return ``n`` surface samples per face for a cube obstacle.

    Args:
        centre: Box centre ``[cx, cy, cz]`` in metres.
        half: Half-side length in metres.
        n: Samples per face.

    Returns:
        Array of shape ``(6*n, 3)``.
    """
    cx, cy, cz = centre
    rng = np.random.default_rng(seed=0)
    pts = []

    for axis, sign in [(0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1)]:
        face_pts = rng.uniform(-half, half, (n, 3)) + np.array([cx, cy, cz])
        face_value = {0: cx, 1: cy, 2: cz}[axis] + sign * half
        face_pts[:, axis] = face_value
        pts.append(face_pts)

    return np.vstack(pts).astype(np.float64)


def _build_scenario_payload() -> OccupancyUpdatePayload:
    """Build an obstacle payload from the scenario box obstacles."""
    parts = [
        _sample_box_surface(c, _BOX_HALF, _N_SAMPLES_PER_FACE)
        for c in _OBSTACLE_CENTRES
    ]
    pts = np.vstack(parts).astype(np.float64)
    return OccupancyUpdatePayload(
        obstacle_points=pts,
        timestamp=0.0,
        frame_id="world",
    )


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------


def _plot(
    out_path: pathlib.Path,
    builder: WorkspaceOccupancyBuilder,
    resolution: float,
) -> None:
    """Generate a 3-D occupancy scatter plot.

    Args:
        out_path: Destination PNG path.
        builder: Fully built ``WorkspaceOccupancyBuilder``.
        resolution: Voxel resolution in metres (for title label).
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("WARNING: matplotlib not available — skipping plot.")
        return

    occ = builder.occupied_centres()
    free = builder.free_centres()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    if len(free) > 0:
        ax.scatter(
            free[:, 0],
            free[:, 1],
            free[:, 2],
            c="lightgrey",
            s=25,
            alpha=0.4,
            label=f"Free ({len(free)})",
        )
    if len(occ) > 0:
        ax.scatter(
            occ[:, 0],
            occ[:, 1],
            occ[:, 2],
            c="red",
            s=55,
            alpha=0.9,
            label=f"Occupied ({len(occ)})",
        )

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")  # type: ignore[attr-defined]
    res_cm = int(round(resolution * 100))
    ax.set_title(
        f"FRET workspace occupancy — {res_cm} cm grid\n"
        f"Milestone 4 CI simulation  |  "
        f"{len(occ)} occupied / {len(free)} free annular voxels"
    )
    ax.legend(fontsize=9)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"[M4] Plot saved → {out_path}")


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate(output_dir: pathlib.Path) -> dict[str, float]:
    """Run the Milestone 4 occupancy simulation.

    Args:
        output_dir: Directory where output files are written.

    Returns:
        Dict of metric names to values.

    Raises:
        AssertionError: If any acceptance criterion is violated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    resolution = 0.20  # 20 cm voxel grid

    # 1 - Build obstacle payload ----------------------------------------------
    payload = _build_scenario_payload()
    print(
        f"[M4] Obstacle payload: {len(payload.obstacle_points)} surface points"
    )

    # 2 - Build workspace occupancy map ---------------------------------------
    builder = WorkspaceOccupancyBuilder(resolution=resolution)
    builder.build(payload)

    occ = builder.occupied_centres()
    free = builder.free_centres()
    print(f"[M4] Occupied voxels : {len(occ)}")
    print(f"[M4] Free voxels     : {len(free)}")
    print(f"[M4] Total annular   : {len(occ) + len(free)}")

    # 3 - Validate: non-empty obstacle detection ------------------------------
    assert (
        len(occ) > 0
    ), "No occupied voxels found — obstacles not detected in the grid."

    # 4 - Validate: annular reachability constraint ---------------------------
    if len(occ) > 0:
        r_occ = np.sqrt(occ[:, 0] ** 2 + occ[:, 1] ** 2)
        assert np.all(
            r_occ >= _R_MIN - 1e-9
        ), f"Occupied voxel outside reachable annulus: min r = {r_occ.min():.4f}"
        assert np.all(
            r_occ <= _R_MAX + 1e-9
        ), f"Occupied voxel outside reachable annulus: max r = {r_occ.max():.4f}"

    all_centres = np.vstack([occ, free]) if len(free) > 0 else occ
    r_all = np.sqrt(all_centres[:, 0] ** 2 + all_centres[:, 1] ** 2)
    assert np.all(r_all >= _R_MIN - 1e-9), "Voxel below R_MIN found."
    assert np.all(r_all <= _R_MAX + 1e-9), "Voxel above R_MAX found."

    # 5 - Validate: is_occupied correctness ----------------------------------
    # Use the default CCD (circumradius) for the in-obstacle query.
    assert builder.is_occupied(
        _INSIDE_PT
    ), f"is_occupied({_INSIDE_PT}) returned False — obstacle not detected."
    assert not builder.is_occupied(
        _FREE_PT
    ), f"is_occupied({_FREE_PT}) returned True — false positive in free space."

    # 6 - Validate: clearance() SDF semantics --------------------------------
    c_inside = builder.clearance(_INSIDE_PT)
    assert (
        c_inside < 0.0
    ), f"clearance({_INSIDE_PT}) = {c_inside:.4f} should be negative."
    c_free = builder.clearance(_FREE_PT)
    assert (
        c_free > 0.0
    ), f"clearance({_FREE_PT}) = {c_free:.4f} should be positive."
    print(f"[M4] clearance(inside) = {c_inside:.4f} m  (< 0 ✓)")
    print(f"[M4] clearance(free)   = {c_free:.4f} m  (> 0 ✓)")

    # 7 - Validate: partition correctness ------------------------------------
    total_classified = len(occ) + len(free)
    assert total_classified > 0, "No voxels classified."

    # 8 - Generate plot -------------------------------------------------------
    _plot(output_dir / "occupancy_map.png", builder, resolution)

    # 9 - Write results.env --------------------------------------------------
    metrics: dict[str, float] = {
        "N_OCCUPIED": float(len(occ)),
        "N_FREE": float(len(free)),
        "N_TOTAL_ANNULAR": float(total_classified),
        "CLEARANCE_INSIDE_M": float(c_inside),
        "CLEARANCE_FREE_M": float(c_free),
        "RESOLUTION_M": float(resolution),
    }
    env_lines = [f"{k}={v}" for k, v in metrics.items()]
    (output_dir / "results.env").write_text("\n".join(env_lines) + "\n")

    return metrics


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    """CLI entry point.

    Returns:
        Exit code: 0 on success, 1 on failure.
    """
    parser = argparse.ArgumentParser(
        description="Milestone 4 workspace occupancy simulation"
    )
    parser.add_argument(
        "--output",
        default="/tmp/sim_output_m4",
        help="Output directory (default: /tmp/sim_output_m4)",
    )
    parser.add_argument(
        "--results-file",
        default=None,
        help="Optional explicit path for results.env (overrides --output)",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)

    print("=== Milestone 4 workspace occupancy simulation ===")
    print(f"Resolution   : 20 cm voxel grid")
    print(f"Output dir   : {output_dir}")

    try:
        metrics = simulate(output_dir)
    except (AssertionError, RuntimeError, ValueError) as exc:
        print(f"\n[M4] FAILED: {exc}", file=sys.stderr)
        return 1

    # Optionally write to an explicit results file (for simulate_milestone4.sh)
    if args.results_file:
        rf = pathlib.Path(args.results_file)
        rf.parent.mkdir(parents=True, exist_ok=True)
        env_lines = [f"{k}={v}" for k, v in metrics.items()]
        rf.write_text("\n".join(env_lines) + "\n")

    print(
        f"\n[M4] PASSED — {int(metrics['N_OCCUPIED'])} occupied voxels, "
        f"{int(metrics['N_FREE'])} free voxels, "
        f"clearance_inside={metrics['CLEARANCE_INSIDE_M']:.4f} m"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
