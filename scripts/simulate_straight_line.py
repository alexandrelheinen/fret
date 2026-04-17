#!/usr/bin/env python3
"""Milestone 1 CI simulation: straight-line SCARA tracking.

Simulates the Jacobian-based controller tracking a Cartesian straight-line
trajectory in pure Python (no ROS, no Gazebo required) for 3 seconds at
50 Hz.  Records the reference and executed trajectories in both Cartesian
space and joint space, then generates diagnostic plots.

Output files (written to the current working directory):
    trajectory_plots.png   — 3-panel figure with all requested plots.

Usage::

    python3 scripts/simulate_straight_line.py [--output DIR]

The image must show:
  - Very close straight line in Cartesian space (EE tracks the reference).
  - Clear mismatch between reference and executed in C-space (nonlinear).
  - Joint variables over time: q1 and q2 nonlinear, q3 constant.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

# Allow running without an installed package by adding src/ to the path
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "src"))

import numpy as np

from fret.control.controller_node import ControllerNode
from fret.control.kinematics import Kinematics
from fret.ros.straight_line_injector import generate_trajectory


# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

N_WAYPOINTS = 150
DURATION = 3.0  # [s]
DT = 1.0 / 50.0  # 50 Hz


def simulate(
    n_waypoints: int = N_WAYPOINTS,
    duration: float = DURATION,
    initial_offset_rad: float = 0.0,
) -> dict[str, np.ndarray]:
    """Run the closed-loop Jacobian controller simulation.

    The controller starts at the exact first reference waypoint.  The
    tracking mismatch is entirely due to the finite proportional gain
    (Kp = 20): the controller can't reach each new waypoint perfectly
    within one 20 ms step, so the executed trajectory lags behind the
    reference.  The lag is small in Cartesian space (< 5 mm, FR-CTL-02)
    but clearly nonlinear in C-space because the IK solution varies
    nonlinearly along the Cartesian straight line.

    Args:
        n_waypoints: Number of trajectory waypoints.
        duration: Trajectory duration in seconds.
        initial_offset_rad: Unused; kept for API compatibility.

    Returns:
        Dictionary with keys:
            ``times``       — (N,) time array [s]
            ``q_ref``       — (N, 3) reference joint configurations
            ``q_exec``      — (N, 3) executed joint configurations
            ``x_ref``       — (N, 3) reference Cartesian EE positions [m]
            ``x_exec``      — (N, 3) executed Cartesian EE positions [m]
            ``ee_error_m``  — (N,) EE position error in metres
    """
    kin = Kinematics("scara")
    configs, timestamps = generate_trajectory(n_waypoints=n_waypoints, duration=duration)
    n = len(configs)

    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(configs)

    # Start exactly at the first reference waypoint — mismatch comes from
    # the finite Kp (tracking lag), not from an artificial initial offset
    q_cur = configs[0].copy()

    times = np.array(timestamps, dtype=np.float64)
    q_ref_arr = np.array(configs, dtype=np.float64)            # (N, 3)
    q_exec_arr = np.zeros_like(q_ref_arr)                      # (N, 3)
    x_ref_arr = np.zeros((n, 3), dtype=np.float64)
    x_exec_arr = np.zeros((n, 3), dtype=np.float64)
    ee_error = np.zeros(n, dtype=np.float64)

    for i, q_ref in enumerate(configs):
        x_ref_arr[i] = kin.forward_kinematics(q_ref)[:3, 3]
        x_exec_arr[i] = kin.forward_kinematics(q_cur)[:3, 3]
        ee_error[i] = float(np.linalg.norm(x_ref_arr[i] - x_exec_arr[i]))

        q_exec_arr[i] = q_cur.copy()

        if ctrl._state.value == 2:  # HALTED
            break

        q_dot = ctrl.compute_jacobian_command(kin, q_cur)
        q_cur = q_cur + q_dot * DT

    return {
        "times": times,
        "q_ref": q_ref_arr,
        "q_exec": q_exec_arr,
        "x_ref": x_ref_arr,
        "x_exec": x_exec_arr,
        "ee_error_m": ee_error,
    }


def make_plots(data: dict[str, np.ndarray], output_path: pathlib.Path) -> None:
    """Generate and save the three-panel trajectory diagnostic figure.

    Panels:
        1. Cartesian space: (x, y, z) reference + executed EE trajectories.
        2. C-space:          (q1, q2, q3) reference + executed joint trajectories.
        3. Joint variables over time: q1(t), q2(t), q3(t) reference + executed.

    Args:
        data: Simulation output dictionary (from ``simulate``).
        output_path: Destination PNG file path.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")  # headless rendering
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as exc:
        print(f"ERROR: matplotlib is required: {exc}", file=sys.stderr)
        sys.exit(1)

    times = data["times"]
    q_ref = data["q_ref"]
    q_exec = data["q_exec"]
    x_ref = data["x_ref"]
    x_exec = data["x_exec"]
    ee_error = data["ee_error_m"]

    max_err_mm = float(np.max(ee_error) * 1000)
    rms_err_mm = float(np.sqrt(np.mean(ee_error**2)) * 1000)

    fig = plt.figure(figsize=(18, 6))
    fig.suptitle(
        f"Milestone 1 — SCARA Straight-Line Tracking  |  "
        f"max EE error = {max_err_mm:.2f} mm  |  "
        f"RMS EE error = {rms_err_mm:.2f} mm",
        fontsize=13,
        fontweight="bold",
    )

    # ------------------------------------------------------------------
    # Panel 1: Cartesian (x, y, z) trajectory
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(1, 3, 1, projection="3d")
    ax1.plot(
        x_ref[:, 0], x_ref[:, 1], x_ref[:, 2],
        "b-", linewidth=2, label="Reference",
    )
    ax1.plot(
        x_exec[:, 0], x_exec[:, 1], x_exec[:, 2],
        "r--", linewidth=1.5, label="Executed",
    )
    ax1.scatter(x_ref[0, 0], x_ref[0, 1], x_ref[0, 2], color="blue", s=50, zorder=5)
    ax1.scatter(x_ref[-1, 0], x_ref[-1, 1], x_ref[-1, 2], color="blue", marker="^", s=60, zorder=5)
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_zlabel("z [m]")  # type: ignore[attr-defined]
    ax1.set_title("Cartesian space (x, y, z)\n≈ straight line", fontsize=11)
    ax1.legend(loc="upper left", fontsize=9)

    # ------------------------------------------------------------------
    # Panel 2: C-space (q1, q2, q3=z) trajectory
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(1, 3, 2, projection="3d")
    ax2.plot(
        np.degrees(q_ref[:, 0]),
        np.degrees(q_ref[:, 1]),
        q_ref[:, 2],
        "b-", linewidth=2, label="Reference",
    )
    ax2.plot(
        np.degrees(q_exec[:, 0]),
        np.degrees(q_exec[:, 1]),
        q_exec[:, 2],
        "r--", linewidth=1.5, label="Executed",
    )
    ax2.scatter(
        np.degrees(q_ref[0, 0]), np.degrees(q_ref[0, 1]), q_ref[0, 2],
        color="blue", s=50, zorder=5,
    )
    ax2.scatter(
        np.degrees(q_ref[-1, 0]), np.degrees(q_ref[-1, 1]), q_ref[-1, 2],
        color="blue", marker="^", s=60, zorder=5,
    )
    ax2.set_xlabel("q1 [°]")
    ax2.set_ylabel("q2 [°]")
    ax2.set_zlabel("q3 [m]")  # type: ignore[attr-defined]
    ax2.set_title("C-space (q1, q2, q3)\nnonlinear ≠ straight line", fontsize=11)
    ax2.legend(loc="upper left", fontsize=9)

    # ------------------------------------------------------------------
    # Panel 3: Joint variables over time
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(1, 3, 3)
    joint_labels = ["q1 [°]", "q2 [°]", "q3 [m]"]
    colors_ref = ["#1f77b4", "#2ca02c", "#ff7f0e"]
    colors_exec = ["#aec7e8", "#98df8a", "#ffbb78"]

    for j in range(3):
        y_ref = np.degrees(q_ref[:, j]) if j < 2 else q_ref[:, j]
        y_exec = np.degrees(q_exec[:, j]) if j < 2 else q_exec[:, j]
        ax3.plot(times, y_ref, color=colors_ref[j], linewidth=2,
                 label=f"ref {joint_labels[j]}")
        ax3.plot(times, y_exec, color=colors_exec[j], linewidth=1.5,
                 linestyle="--", label=f"exec {joint_labels[j]}")

    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Joint value")
    ax3.set_title("Joint variables over time", fontsize=11)
    ax3.legend(loc="upper left", fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved: {output_path}")


def print_summary(data: dict[str, np.ndarray]) -> None:
    """Print a text summary of simulation results to stdout.

    Args:
        data: Simulation output dictionary.
    """
    ee = data["ee_error_m"]
    q_ref = data["q_ref"]
    print("=" * 60)
    print("Milestone 1 — Straight-Line Tracking Simulation Summary")
    print("=" * 60)
    print(f"  Duration:              {data['times'][-1]:.2f} s")
    print(f"  Waypoints:             {len(data['times'])}")
    print(f"  Max EE error:          {np.max(ee)*1000:.2f} mm  (limit: 5 mm)")
    print(f"  RMS EE error:          {np.sqrt(np.mean(ee**2))*1000:.2f} mm")
    print(f"  q2 peak (nonlinearity) {np.degrees(np.max(np.abs(q_ref[:, 1]))):.1f} °")
    print(f"  PASS (EE ≤ 5 mm):      {'YES' if np.max(ee) <= 0.005 else 'NO'}")
    print("=" * 60)


def main() -> None:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Simulate Milestone 1 straight-line SCARA tracking and generate plots.",
    )
    parser.add_argument(
        "--output",
        default=".",
        help="Directory to write trajectory_plots.png (default: current directory)",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.05,
        help="Initial q1 offset [rad] to generate visible tracking error (default: 0.05)",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / "trajectory_plots.png"

    print("Running Milestone 1 simulation...")
    data = simulate(
        n_waypoints=N_WAYPOINTS,
        duration=DURATION,
        initial_offset_rad=args.offset,
    )
    print_summary(data)
    make_plots(data, output_file)

    max_err_m = float(np.max(data["ee_error_m"]))
    if max_err_m > 0.005:
        print(
            f"WARNING: max EE error {max_err_m*1000:.2f} mm exceeds 5 mm threshold!",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
