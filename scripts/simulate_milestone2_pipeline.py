#!/usr/bin/env python3
"""Milestone 2 CI simulation: ARCO planning pipeline in pure Python.

Runs the full Milestone 2 planning pipeline without ROS or Gazebo:

  1. Initialises a SCARA Kinematics engine.
  2. Builds an empty-world OccupancyAdapter (no obstacles).
  3. Creates a PlannerNode and submits a PlanningRequest (static_reach
     scenario: rest pose → goal configuration).
  4. Validates the PlanningResult: SUCCESS, ≥ 2 waypoints, all within
     joint limits.
  5. Runs TrajectoryGenerator.process() and checks ``len(traj.points) ≥ 2``.
  6. Generates a three-panel diagnostic plot:
       Panel 1: Cartesian (x, y, z) trajectory — FK of each waypoint.
       Panel 2: C-space (q1°, q2°, q3 m) — raw waypoint sequence.
       Panel 3: Joint variables over path index — q1, q2, q3.
  7. Writes ``results.env`` and exits 0 on success, 1 on failure.

No ARCO or ROS runtime is required — a pure-Python fallback planner and
trajectory generator are used when ARCO is not installed.

Usage::

    python3 scripts/simulate_milestone2_pipeline.py [--output DIR]

Output files (written to DIR, default /tmp/sim_output_m2):
    planning_plots.png   — 3-panel diagnostic figure.
    results.env          — KEY=VALUE file with simulation metrics.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

# Allow running without an installed package by adding src/ to the path.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "src"))

import numpy as np

from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.planner_node import PlannerNode
from fret.planning.trajectory_generator import TrajectoryGenerator
from fret.scene.occupancy_adapter import OccupancyAdapter

# ---------------------------------------------------------------------------
# Scenario parameters (SC-01: Static Reach, empty world)
# ---------------------------------------------------------------------------

_START_CONFIG = np.array([0.0, 0.0, 0.0])  # rest pose
_GOAL_CONFIG = np.array([0.5, -0.3, 0.05])  # reachable goal
_PLANNING_TIMEOUT = 30.0  # [s]
_SCENARIO_ID = "static_reach_ci"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _fk_ee_pos(kin: Kinematics, q: np.ndarray) -> np.ndarray:
    """Return the 3-D EE position for configuration q."""
    return kin.forward_kinematics(q)[:3, 3]


def _within_limits(kin: Kinematics, path: list[np.ndarray]) -> bool:
    """Return True if every waypoint in path is within joint limits."""
    limits = kin.joint_limits  # (DOF, 2)
    for q in path:
        for i, (lo, hi) in enumerate(limits):
            if not (lo - 1e-9 <= float(q[i]) <= hi + 1e-9):
                return False
    return True


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate(output_dir: pathlib.Path) -> dict[str, float]:
    """Run the Milestone 2 planning simulation.

    Args:
        output_dir: Directory where output files are written.

    Returns:
        Dict of metric names to values.

    Raises:
        AssertionError: If any acceptance criterion is violated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1 — Kinematics ---------------------------------------------------------
    kin = Kinematics("scara")

    # 2 — Empty-world OccupancyAdapter ---------------------------------------
    empty_payload = OccupancyUpdatePayload(
        obstacle_points=np.zeros((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )
    adapter = OccupancyAdapter()
    adapter.update(empty_payload)

    # 3 — Plan ---------------------------------------------------------------
    planner = PlannerNode(model="scara", occupancy_adapter=adapter)
    request = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id=_SCENARIO_ID,
    )

    t0 = time.monotonic()
    result = planner.plan(request)
    planning_duration = time.monotonic() - t0

    # 4 — Validate PlanningResult --------------------------------------------
    assert result.status == PlanningStatus.SUCCESS, (
        f"PlannerNode returned {result.status!r} "
        f"(error_code={result.error_code!r})"
    )
    assert (
        len(result.path) >= 2
    ), f"Expected ≥ 2 waypoints, got {len(result.path)}"
    assert _within_limits(
        kin, result.path
    ), "One or more waypoints violate joint limits"

    # 5 — Post-process -------------------------------------------------------
    traj_gen = TrajectoryGenerator(kin)
    traj = traj_gen.process(result.path)
    assert (
        len(traj.points) >= 2
    ), f"TrajectoryGenerator returned {len(traj.points)} points, expected ≥ 2"

    # 6 — Compute metrics ----------------------------------------------------
    waypoints = result.path
    n_wps = len(waypoints)
    ee_positions = np.array([_fk_ee_pos(kin, q) for q in waypoints])
    goal_ee = _fk_ee_pos(kin, _GOAL_CONFIG)
    start_ee = _fk_ee_pos(kin, _START_CONFIG)
    final_ee_error_m = float(np.linalg.norm(ee_positions[-1] - goal_ee))
    path_length = float(
        sum(
            np.linalg.norm(waypoints[i + 1] - waypoints[i])
            for i in range(n_wps - 1)
        )
    )

    metrics: dict[str, float] = {
        "PLANNING_DURATION_S": planning_duration,
        "N_WAYPOINTS": float(n_wps),
        "N_TRAJ_POINTS": float(len(traj.points)),
        "FINAL_EE_ERROR_M": final_ee_error_m,
        "FINAL_EE_ERROR_MM": final_ee_error_m * 1000.0,
        "PATH_LENGTH_RAD": path_length,
    }

    # 7 — Generate plots -----------------------------------------------------
    _plot(
        output_dir / "planning_plots.png",
        waypoints=waypoints,
        ee_positions=ee_positions,
        traj=traj,
        goal_ee=goal_ee,
        start_ee=start_ee,
    )

    # 8 — Write results.env --------------------------------------------------
    env_lines = [f"{k}={v}" for k, v in metrics.items()]
    (output_dir / "results.env").write_text("\n".join(env_lines) + "\n")

    return metrics


def _plot(
    out_path: pathlib.Path,
    *,
    waypoints: list[np.ndarray],
    ee_positions: np.ndarray,
    traj: object,
    goal_ee: np.ndarray,
    start_ee: np.ndarray,
) -> None:
    """Generate a 3-panel planning diagnostic figure."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not available — skipping plot generation.")
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle(
        "Milestone 2 — ARCO Planning Pipeline (Static Reach)", fontsize=13
    )

    idx = np.arange(len(waypoints))

    # Panel 1: Cartesian trajectory
    ax = axes[0]
    ax.plot(
        ee_positions[:, 0], ee_positions[:, 1], "b.-", label="EE path (xy)"
    )
    ax.plot(start_ee[0], start_ee[1], "go", markersize=10, label="Start EE")
    ax.plot(goal_ee[0], goal_ee[1], "r*", markersize=12, label="Goal EE")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Cartesian EE Path (top view)")
    ax.legend(fontsize=7)
    ax.grid(True)
    ax.set_aspect("equal")

    # Panel 2: C-space (q1°, q2°, q3 m)
    ax = axes[1]
    q_arr = np.array(waypoints)
    ax.plot(idx, np.degrees(q_arr[:, 0]), "r.-", label="q1 [°]")
    ax.plot(idx, np.degrees(q_arr[:, 1]), "g.-", label="q2 [°]")
    ax.set_xlabel("Waypoint index")
    ax.set_ylabel("Angle [°]")
    ax2 = ax.twinx()
    ax2.plot(idx, q_arr[:, 2] * 100, "b.-", label="q3 [cm]")
    ax2.set_ylabel("Extension [cm]", color="b")
    ax.set_title("C-Space (joint angles + extension)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7)
    ax.grid(True)

    # Panel 3: EE z over path
    ax = axes[2]
    ax.plot(idx, ee_positions[:, 0], "r.-", label="EE x [m]")
    ax.plot(idx, ee_positions[:, 1], "g.-", label="EE y [m]")
    ax.plot(idx, ee_positions[:, 2], "b.-", label="EE z [m]")
    ax.axhline(
        goal_ee[0], color="r", linestyle="--", alpha=0.4, label="Goal x"
    )
    ax.axhline(
        goal_ee[1], color="g", linestyle="--", alpha=0.4, label="Goal y"
    )
    ax.axhline(
        goal_ee[2], color="b", linestyle="--", alpha=0.4, label="Goal z"
    )
    ax.set_xlabel("Waypoint index")
    ax.set_ylabel("Position [m]")
    ax.set_title("EE Position over Path")
    ax.legend(fontsize=7)
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(str(out_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved to {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Milestone 2 planning simulation"
    )
    parser.add_argument(
        "--output",
        default="/tmp/sim_output_m2",
        help="Output directory for plots and results (default: /tmp/sim_output_m2)",
    )
    parser.add_argument(
        "--results-file",
        default=None,
        help="Optional explicit path for results.env (overrides --output location)",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)

    print("=== Milestone 2 planning pipeline simulation ===")
    print(f"Start config : {_START_CONFIG.tolist()}")
    print(f"Goal  config : {_GOAL_CONFIG.tolist()}")

    try:
        metrics = simulate(output_dir)
    except AssertionError as exc:
        print(f"\n❌  SIMULATION FAILED: {exc}")
        return 1

    print("\n--- Results ---")
    for k, v in metrics.items():
        print(f"  {k} = {v:.4f}")

    n_wps = int(metrics["N_WAYPOINTS"])
    n_pts = int(metrics["N_TRAJ_POINTS"])
    ee_err_mm = metrics["FINAL_EE_ERROR_MM"]
    duration_s = metrics["PLANNING_DURATION_S"]

    print(
        f"\n✅  Simulation PASSED — "
        f"{n_wps} waypoints, {n_pts} trajectory points, "
        f"EE error = {ee_err_mm:.2f} mm, "
        f"planning time = {duration_s:.3f} s"
    )

    # Write results.env to explicit path if given
    if args.results_file:
        env_lines = [f"{k}={v}" for k, v in metrics.items()]
        pathlib.Path(args.results_file).write_text("\n".join(env_lines) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
