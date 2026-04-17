#!/usr/bin/env python3
"""Milestone 3 CI simulation: full end-to-end planning + tracking pipeline.

Runs the complete Milestone 3 pipeline without ROS or Gazebo:

  1. Initialises a SCARA Kinematics engine.
  2. Builds an empty-world OccupancyAdapter (no obstacles).
  3. Creates a PlannerNode and submits a PlanningRequest (static_reach
     scenario: rest pose → goal configuration).
  4. Validates the PlanningResult: SUCCESS, ≥ 2 waypoints.
  5. Densifies the planned path to ``_N_STEPS`` waypoints to simulate a
     20-second trajectory at 50 Hz (1000 steps).
  6. Runs the ControllerNode Jacobian tracking loop at 50 Hz:
       - Records reference and executed EE positions.
       - Records joint velocity commands.
       - Checks for fault triggering.
  7. Validates acceptance criteria (Milestone 3, FR-CTL-02):
       - Max EE tracking error ≤ 5 mm.
       - No fault triggered (controller stays in TRACKING state).
       - Final EE position within 20 mm of the goal.
  8. Generates a three-panel diagnostic plot:
       Panel 1: Cartesian (x, y) EE path — reference vs. executed.
       Panel 2: EE tracking error over time.
       Panel 3: Joint variables over time (q1, q2, q3).
  9. Writes ``results.env`` and exits 0 on success, 1 on failure.

No ARCO or ROS runtime is required — the pure-Python fallback planner and
Jacobian controller are used throughout.

Usage::

    python3 scripts/simulate_milestone3_pipeline.py [--output DIR]

Output files (written to DIR, default /tmp/sim_output_m3):
    tracking_plots.png   - 3-panel diagnostic figure.
    results.env          - KEY=VALUE file with simulation metrics.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

# Allow running without an installed package by adding src/ to the path.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "src"))

import numpy as np

from fret.control.controller_node import ControllerNode, _NodeState
from fret.control.kinematics import Kinematics
from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.planner_node import PlannerNode
from fret.scene.occupancy_adapter import OccupancyAdapter

# ---------------------------------------------------------------------------
# Scenario parameters (SC-01: Static Reach, empty world)
# ---------------------------------------------------------------------------

_START_CONFIG = np.array([0.0, 0.0, 0.0])  # rest pose
_GOAL_CONFIG = np.array([0.5, -0.3, 0.05])  # reachable goal
_PLANNING_TIMEOUT = 30.0  # [s]
_SCENARIO_ID = "static_reach_ci"
_DURATION_S = 20.0  # trajectory duration [s]
_RATE_HZ = 50.0  # controller rate [Hz]
_N_STEPS = int(_DURATION_S * _RATE_HZ)  # 1000 steps
_DT = 1.0 / _RATE_HZ
_MAX_EE_ERROR_M = 0.005  # 5 mm (FR-CTL-02)
_GOAL_CONVERGENCE_M = 0.020  # 20 mm final goal tolerance


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _fk_ee_pos(kin: Kinematics, q: np.ndarray) -> np.ndarray:
    """Return the 3-D EE position for configuration q.

    Args:
        kin: Kinematics engine.
        q: Joint configuration, shape ``(DOF,)``.

    Returns:
        3-D EE position array.
    """
    return kin.forward_kinematics(q)[:3, 3]


def _densify_path(path: list[np.ndarray], n_steps: int) -> list[np.ndarray]:
    """Linearly interpolate a joint-space path to n_steps waypoints.

    Converts a sparse planned path (e.g., 2 waypoints) into a dense
    trajectory suitable for Jacobian controller tracking at 50 Hz over
    the simulation duration.

    Args:
        path: List of joint configurations (at least 2 elements).
        n_steps: Number of output waypoints.

    Returns:
        List of n_steps joint configurations.
    """
    waypoints: list[np.ndarray] = []
    for i in range(n_steps):
        alpha = i / (n_steps - 1)
        q = path[0] + alpha * (path[-1] - path[0])
        waypoints.append(q)
    return waypoints


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate(output_dir: pathlib.Path) -> dict[str, float]:
    """Run the Milestone 3 end-to-end planning + tracking simulation.

    Args:
        output_dir: Directory where output files are written.

    Returns:
        Dict of metric names to values.

    Raises:
        AssertionError: If any acceptance criterion is violated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1 - Kinematics ---------------------------------------------------------
    kin = Kinematics("scara")

    # 2 - Empty-world OccupancyAdapter ---------------------------------------
    empty_payload = OccupancyUpdatePayload(
        obstacle_points=np.zeros((0, 3), dtype=np.float64),
        timestamp=0.0,
        frame_id="world",
    )
    adapter = OccupancyAdapter()
    adapter.update(empty_payload)

    # 3 - Plan ---------------------------------------------------------------
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

    # 4 - Validate PlanningResult --------------------------------------------
    assert result.status == PlanningStatus.SUCCESS, (
        f"PlannerNode returned {result.status!r} "
        f"(error_code={result.error_code!r})"
    )
    assert (
        len(result.path) >= 2
    ), f"Expected ≥ 2 waypoints, got {len(result.path)}"

    # 5 - Densify planned path to tracking trajectory ------------------------
    # Convert the sparse planner output to a dense 1000-point reference
    # trajectory that the Jacobian controller can track at 50 Hz over 20 s.
    waypoints = _densify_path(result.path, _N_STEPS)

    # 6 - Controller tracking simulation -------------------------------------
    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(waypoints)
    assert (
        ctrl._state == _NodeState.TRACKING
    ), "Controller did not transition to TRACKING after set_trajectory"

    q_cur = _START_CONFIG.copy()
    times = np.arange(_N_STEPS) * _DT
    q_ref_arr = np.array(waypoints, dtype=np.float64)  # (N, 3)
    q_exec_arr = np.zeros_like(q_ref_arr)  # (N, 3)
    x_ref_arr = np.zeros((_N_STEPS, 3), dtype=np.float64)
    x_exec_arr = np.zeros((_N_STEPS, 3), dtype=np.float64)
    ee_errors = np.zeros(_N_STEPS, dtype=np.float64)
    fault_triggered = False
    fault_step = _N_STEPS

    for i, q_ref in enumerate(waypoints):
        x_ref = _fk_ee_pos(kin, q_ref)
        x_cur = _fk_ee_pos(kin, q_cur)
        x_ref_arr[i] = x_ref
        x_exec_arr[i] = x_cur
        q_exec_arr[i] = q_cur.copy()
        ee_errors[i] = float(np.linalg.norm(x_ref - x_cur))

        ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + ctrl._get_current_command() * _DT

        if ctrl._state == _NodeState.HALTED:
            fault_triggered = True
            fault_step = i
            break

    # 7 - Validate acceptance criteria ---------------------------------------
    assert not fault_triggered, (
        f"Fault triggered at step {fault_step} "
        f"(EE error = {ee_errors[fault_step] * 1000:.2f} mm)"
    )

    max_ee_error_m = float(np.max(ee_errors))
    assert max_ee_error_m <= _MAX_EE_ERROR_M, (
        f"Max EE error {max_ee_error_m * 1000:.2f} mm "
        f"exceeds 5 mm limit (FR-CTL-02)"
    )

    goal_ee = _fk_ee_pos(kin, _GOAL_CONFIG)
    start_ee = _fk_ee_pos(kin, _START_CONFIG)
    final_ee_error_m = float(np.linalg.norm(x_exec_arr[-1] - goal_ee))
    assert final_ee_error_m <= _GOAL_CONVERGENCE_M, (
        f"Final EE error {final_ee_error_m * 1000:.2f} mm "
        f"exceeds {_GOAL_CONVERGENCE_M * 1000:.0f} mm goal convergence limit"
    )

    # 8 - Compute metrics ----------------------------------------------------
    rms_ee_error_m = float(np.sqrt(np.mean(ee_errors**2)))
    metrics: dict[str, float] = {
        "PLANNING_DURATION_S": planning_duration,
        "N_WAYPOINTS": float(len(result.path)),
        "N_TRAJ_STEPS": float(_N_STEPS),
        "TRAJ_DURATION_S": _DURATION_S,
        "MAX_EE_ERROR_MM": max_ee_error_m * 1000.0,
        "RMS_EE_ERROR_MM": rms_ee_error_m * 1000.0,
        "FINAL_EE_ERROR_MM": final_ee_error_m * 1000.0,
        "FAULT_TRIGGERED": 0.0,
    }

    # 9 - Generate plots -----------------------------------------------------
    _plot(
        output_dir / "tracking_plots.png",
        times=times,
        q_ref_arr=q_ref_arr,
        q_exec_arr=q_exec_arr,
        x_ref_arr=x_ref_arr,
        x_exec_arr=x_exec_arr,
        ee_errors=ee_errors,
        goal_ee=goal_ee,
        start_ee=start_ee,
    )

    # 10 - Write results.env -------------------------------------------------
    env_lines = [f"{k}={v}" for k, v in metrics.items()]
    (output_dir / "results.env").write_text("\n".join(env_lines) + "\n")

    return metrics


def _plot(
    out_path: pathlib.Path,
    *,
    times: np.ndarray,
    q_ref_arr: np.ndarray,
    q_exec_arr: np.ndarray,
    x_ref_arr: np.ndarray,
    x_exec_arr: np.ndarray,
    ee_errors: np.ndarray,
    goal_ee: np.ndarray,
    start_ee: np.ndarray,
) -> None:
    """Generate a 3-panel planning + tracking diagnostic figure.

    Args:
        out_path: Destination PNG file path.
        times: Time array for tracking steps, shape ``(N,)``.
        q_ref_arr: Reference joint configurations, shape ``(N, 3)``.
        q_exec_arr: Executed joint configurations, shape ``(N, 3)``.
        x_ref_arr: Reference EE Cartesian positions, shape ``(N, 3)``.
        x_exec_arr: Executed EE Cartesian positions, shape ``(N, 3)``.
        ee_errors: EE tracking error per step, shape ``(N,)``.
        goal_ee: Goal EE position, shape ``(3,)``.
        start_ee: Start EE position, shape ``(3,)``.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not available - skipping plot generation.")
        return

    max_err_mm = float(np.max(ee_errors) * 1000)
    rms_err_mm = float(np.sqrt(np.mean(ee_errors**2)) * 1000)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(
        f"Milestone 3 — Full Pipeline: Planning + Tracking  |  "
        f"max EE error = {max_err_mm:.2f} mm  |  "
        f"RMS EE error = {rms_err_mm:.2f} mm",
        fontsize=12,
        fontweight="bold",
    )

    # Panel 1: Cartesian EE path (top view) — reference vs executed
    ax = axes[0]
    ax.plot(
        x_ref_arr[:, 0],
        x_ref_arr[:, 1],
        "b-",
        linewidth=2,
        label="Reference (planned)",
        alpha=0.7,
    )
    ax.plot(
        x_exec_arr[:, 0],
        x_exec_arr[:, 1],
        "r--",
        linewidth=1.5,
        label="Executed (tracked)",
        alpha=0.9,
    )
    ax.plot(
        start_ee[0],
        start_ee[1],
        "go",
        markersize=10,
        label="Start EE",
        zorder=5,
    )
    ax.plot(
        goal_ee[0], goal_ee[1], "r*", markersize=12, label="Goal EE", zorder=5
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Cartesian EE Path (top view)\nReference vs. Executed")
    ax.legend(fontsize=7)
    ax.grid(True)
    ax.set_aspect("equal")

    # Panel 2: EE tracking error over time
    ax = axes[1]
    ax.plot(
        times, ee_errors * 1000, "r-", linewidth=1.5, label="EE error [mm]"
    )
    ax.axhline(
        5.0, color="k", linestyle="--", linewidth=1.2, label="5 mm limit"
    )
    ax.fill_between(times, ee_errors * 1000, alpha=0.2, color="red")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("EE error [mm]")
    ax.set_title(f"EE Tracking Error over Time\nmax = {max_err_mm:.2f} mm")
    ax.legend(fontsize=8)
    ax.grid(True)
    ax.set_ylim(bottom=0)

    # Panel 3: Joint variables over time
    ax = axes[2]
    joint_labels = ["q1 [°]", "q2 [°]", "q3 [m]"]
    colors_ref = ["#1f77b4", "#2ca02c", "#ff7f0e"]
    colors_exec = ["#aec7e8", "#98df8a", "#ffbb78"]

    for j in range(3):
        y_ref = np.degrees(q_ref_arr[:, j]) if j < 2 else q_ref_arr[:, j]
        y_exec = np.degrees(q_exec_arr[:, j]) if j < 2 else q_exec_arr[:, j]
        ax.plot(
            times,
            y_ref,
            color=colors_ref[j],
            linewidth=2,
            label=f"ref {joint_labels[j]}",
        )
        ax.plot(
            times,
            y_exec,
            color=colors_exec[j],
            linewidth=1.5,
            linestyle="--",
            label=f"exec {joint_labels[j]}",
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Joint value")
    ax.set_title("Joint Variables over Time\nReference vs. Executed")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(out_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved to {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    """CLI entry point.

    Returns:
        Exit code: 0 on success, 1 on failure.
    """
    parser = argparse.ArgumentParser(
        description="Milestone 3 end-to-end planning + tracking simulation"
    )
    parser.add_argument(
        "--output",
        default="/tmp/sim_output_m3",
        help="Output directory for plots and results (default: /tmp/sim_output_m3)",
    )
    parser.add_argument(
        "--results-file",
        default=None,
        help=(
            "Optional explicit path for results.env "
            "(overrides --output location)"
        ),
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)

    print("=== Milestone 3 end-to-end simulation (planning + tracking) ===")
    print(f"Start config  : {_START_CONFIG.tolist()}")
    print(f"Goal  config  : {_GOAL_CONFIG.tolist()}")
    print(
        f"Sim duration  : {_DURATION_S:.0f} s  ({_N_STEPS} steps @ {_RATE_HZ:.0f} Hz)"
    )

    try:
        metrics = simulate(output_dir)
    except AssertionError as exc:
        print(f"\n❌  SIMULATION FAILED: {exc}")
        return 1

    print("\n--- Results ---")
    for k, v in metrics.items():
        print(f"  {k} = {v:.4f}")

    max_err_mm = metrics["MAX_EE_ERROR_MM"]
    rms_err_mm = metrics["RMS_EE_ERROR_MM"]
    final_err_mm = metrics["FINAL_EE_ERROR_MM"]
    planning_s = metrics["PLANNING_DURATION_S"]
    n_wps = int(metrics["N_WAYPOINTS"])

    print(
        f"\n✅  Simulation PASSED — "
        f"{n_wps} planned waypoints, "
        f"max EE error = {max_err_mm:.2f} mm, "
        f"RMS = {rms_err_mm:.2f} mm, "
        f"final error = {final_err_mm:.2f} mm, "
        f"planning = {planning_s:.3f} s"
    )

    # Write results.env to explicit path if given
    if args.results_file:
        env_lines = [f"{k}={v}" for k, v in metrics.items()]
        pathlib.Path(args.results_file).write_text("\n".join(env_lines) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
