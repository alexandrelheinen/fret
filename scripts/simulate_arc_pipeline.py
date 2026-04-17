#!/usr/bin/env python3
"""Arc scenario CI simulation: pure-Python arc trajectory + tracking pipeline.

Runs the complete arc scenario (SC-05) without ROS or Gazebo:

  1. Generates a circular arc trajectory using ``generate_arc_trajectory``
     (analytical IK, SCARA kinematics).
  2. Densifies the arc waypoints to ``_N_STEPS`` steps (50 Hz over the
     configured arc duration).
  3. Runs the ControllerNode Jacobian tracking loop:
       - Records reference and executed EE positions.
       - Detects fault triggering.
  4. Validates acceptance criteria:
       - Max EE tracking error ≤ 5 mm (FR-CTL-02).
       - No fault triggered.
  5. Generates a three-panel diagnostic figure (matplotlib, Agg backend):
       Panel 1: Cartesian (x, y) arc path — reference vs. executed.
                Shows the visible arc shape in the horizontal plane.
       Panel 2: EE tracking error over time.
       Panel 3: Joint variables (q1, q2, q3) over time — reference vs. executed.
  6. Writes ``results.env`` and exits 0 on success, 1 on failure.

No ARCO or ROS runtime is required.

Usage::

    python3 scripts/simulate_arc_pipeline.py [--output DIR]

Output files (written to DIR, default /tmp/sim_output_arc):
    arc_plots.png      - 3-panel diagnostic figure.
    results.env        - KEY=VALUE file with simulation metrics.
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
from fret.ros.arc_injector import generate_arc_trajectory

# ---------------------------------------------------------------------------
# Scenario parameters (SC-05: Arc, from arc.yml)
# ---------------------------------------------------------------------------

_ARC_CENTER_X = 0.30  # m
_ARC_CENTER_Y = 0.00  # m
_ARC_RADIUS = 0.15  # m
_ARC_START_DEG = -45.0  # degrees
_ARC_END_DEG = 45.0  # degrees
_Z_HEIGHT = 0.138  # m (constant EE height)
_DURATION_S = 4.0  # [s]
_RATE_HZ = 50.0  # controller rate [Hz]
_N_STEPS = int(_DURATION_S * _RATE_HZ)  # 200 steps
_DT = 1.0 / _RATE_HZ
_MAX_EE_ERROR_M = 0.005  # 5 mm (FR-CTL-02)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _fk_ee_pos(kin: Kinematics, q: np.ndarray) -> np.ndarray:
    """Return the 3-D EE position for configuration q.

    Args:
        kin: Kinematics engine.
        q: Joint configuration, shape ``(DOF,)``.

    Returns:
        3-D EE position array, shape ``(3,)``.
    """
    return kin.forward_kinematics(q)[:3, 3]


def _densify_arc(
    joint_configs: list[np.ndarray],
    n_steps: int,
) -> list[np.ndarray]:
    """Linearly interpolate a joint-space arc to exactly n_steps waypoints.

    Args:
        joint_configs: Ordered list of joint configurations (≥ 2 elements).
        n_steps: Target number of waypoints.

    Returns:
        List of n_steps joint configurations.
    """
    n_in = len(joint_configs)
    result: list[np.ndarray] = []
    for i in range(n_steps):
        # Map i uniformly into [0, n_in - 1]
        alpha = i / (n_steps - 1)
        idx_f = alpha * (n_in - 1)
        lo = int(idx_f)
        hi = min(lo + 1, n_in - 1)
        t = idx_f - lo
        q = joint_configs[lo] * (1.0 - t) + joint_configs[hi] * t
        result.append(q)
    return result


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate(output_dir: pathlib.Path) -> dict[str, float]:
    """Run the arc scenario end-to-end simulation.

    Args:
        output_dir: Directory where output files are written.

    Returns:
        Dict of metric names to float values.

    Raises:
        AssertionError: If any acceptance criterion is violated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    kin = Kinematics("scara")

    # 1 – Generate arc trajectory (no ROS required) --------------------------
    t0 = time.monotonic()
    joint_configs, timestamps = generate_arc_trajectory(
        center_x=_ARC_CENTER_X,
        center_y=_ARC_CENTER_Y,
        radius=_ARC_RADIUS,
        start_deg=_ARC_START_DEG,
        end_deg=_ARC_END_DEG,
        z_height=_Z_HEIGHT,
        n_waypoints=_N_STEPS,
        duration=_DURATION_S,
    )
    gen_duration = time.monotonic() - t0

    assert (
        len(joint_configs) >= 2
    ), f"Arc trajectory must have ≥ 2 waypoints, got {len(joint_configs)}"

    # 2 – Densify (arc_injector already returns N_STEPS; this is a no-op) ---
    waypoints = _densify_arc(joint_configs, _N_STEPS)

    # 3 – Controller tracking simulation -------------------------------------
    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(waypoints)
    assert (
        ctrl._state == _NodeState.TRACKING
    ), "Controller did not transition to TRACKING after set_trajectory"

    q_cur = waypoints[0].copy()
    times = np.array(timestamps, dtype=np.float64)
    q_ref_arr = np.array(waypoints, dtype=np.float64)  # (N, 3)
    q_exec_arr = np.zeros_like(q_ref_arr)
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

    # 4 – Validate acceptance criteria ---------------------------------------
    assert not fault_triggered, (
        f"Fault triggered at step {fault_step} "
        f"(EE error = {ee_errors[fault_step] * 1000:.2f} mm)"
    )

    max_ee_error_m = float(np.max(ee_errors))
    assert max_ee_error_m <= _MAX_EE_ERROR_M, (
        f"Max EE error {max_ee_error_m * 1000:.2f} mm "
        f"exceeds 5 mm limit (FR-CTL-02)"
    )

    # 5 – Compute metrics ----------------------------------------------------
    rms_ee_error_m = float(np.sqrt(np.mean(ee_errors**2)))
    start_ee = x_ref_arr[0]
    end_ee = x_ref_arr[-1]

    metrics: dict[str, float] = {
        "GEN_DURATION_S": gen_duration,
        "N_WAYPOINTS": float(len(joint_configs)),
        "TRAJ_DURATION_S": _DURATION_S,
        "MAX_EE_ERROR_MM": max_ee_error_m * 1000.0,
        "RMS_EE_ERROR_MM": rms_ee_error_m * 1000.0,
        "FAULT_TRIGGERED": 0.0,
    }

    # 6 – Generate plots -----------------------------------------------------
    _plot(
        output_dir / "arc_plots.png",
        times=times,
        q_ref_arr=q_ref_arr,
        q_exec_arr=q_exec_arr,
        x_ref_arr=x_ref_arr,
        x_exec_arr=x_exec_arr,
        ee_errors=ee_errors,
        start_ee=start_ee,
        end_ee=end_ee,
    )

    # 7 – Write results.env --------------------------------------------------
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
    start_ee: np.ndarray,
    end_ee: np.ndarray,
) -> None:
    """Generate a 3-panel arc trajectory diagnostic figure (matplotlib Agg).

    The figure shows the visible arc shape in Cartesian space (Panel 1),
    the EE tracking error over time (Panel 2), and joint variables over
    time — reference vs. executed (Panel 3).

    Args:
        out_path: Destination PNG file path.
        times: Timestamp array, shape ``(N,)``.
        q_ref_arr: Reference joint configs, shape ``(N, 3)``.
        q_exec_arr: Executed joint configs, shape ``(N, 3)``.
        x_ref_arr: Reference EE Cartesian positions, shape ``(N, 3)``.
        x_exec_arr: Executed EE Cartesian positions, shape ``(N, 3)``.
        ee_errors: EE tracking error per step, shape ``(N,)``.
        start_ee: Arc start EE position, shape ``(3,)``.
        end_ee: Arc end EE position, shape ``(3,)``.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not available — skipping plot generation.")
        return

    max_err_mm = float(np.max(ee_errors) * 1000)
    rms_err_mm = float(np.sqrt(np.mean(ee_errors**2)) * 1000)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(
        f"Arc Scenario (SC-05) — Trajectory + Tracking  |  "
        f"radius={_ARC_RADIUS:.2f} m  "
        f"[{_ARC_START_DEG:.0f}° → {_ARC_END_DEG:.0f}°]  |  "
        f"max EE error = {max_err_mm:.2f} mm  |  "
        f"RMS = {rms_err_mm:.2f} mm",
        fontsize=11,
        fontweight="bold",
    )

    # ------------------------------------------------------------------
    # Panel 1: Cartesian EE arc (top view, XY plane)
    # ------------------------------------------------------------------
    ax = axes[0]

    # Draw the ideal arc as a thin grey reference
    import math

    ideal_angles = np.linspace(
        math.radians(_ARC_START_DEG),
        math.radians(_ARC_END_DEG),
        200,
    )
    arc_x = _ARC_CENTER_X + _ARC_RADIUS * np.cos(ideal_angles)
    arc_y = _ARC_CENTER_Y + _ARC_RADIUS * np.sin(ideal_angles)
    ax.plot(arc_x, arc_y, "grey", linewidth=1, alpha=0.5, label="Ideal arc")

    ax.plot(
        x_ref_arr[:, 0],
        x_ref_arr[:, 1],
        "b-",
        linewidth=2,
        label="Reference (IK)",
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
        label="Arc start",
        zorder=5,
    )
    ax.plot(
        end_ee[0],
        end_ee[1],
        "r*",
        markersize=12,
        label="Arc end",
        zorder=5,
    )
    # Mark arc centre
    ax.plot(
        _ARC_CENTER_X,
        _ARC_CENTER_Y,
        "k+",
        markersize=10,
        label=f"Centre ({_ARC_CENTER_X:.2f}, {_ARC_CENTER_Y:.2f}) m",
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        f"Cartesian EE Arc (top view)\n"
        f"r = {_ARC_RADIUS:.2f} m, z = {_Z_HEIGHT:.3f} m"
    )
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.4)
    ax.set_aspect("equal")

    # ------------------------------------------------------------------
    # Panel 2: EE tracking error over time
    # ------------------------------------------------------------------
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
    ax.grid(True, alpha=0.4)
    ax.set_ylim(bottom=0)

    # ------------------------------------------------------------------
    # Panel 3: Joint variables over time
    # ------------------------------------------------------------------
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
        description="Arc scenario (SC-05) trajectory + tracking simulation"
    )
    parser.add_argument(
        "--output",
        default="/tmp/sim_output_arc",
        help="Output directory for plots and results (default: /tmp/sim_output_arc)",
    )
    parser.add_argument(
        "--results-file",
        default=None,
        help="Optional explicit path for results.env (overrides --output location)",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)

    print("=== Arc Scenario (SC-05) simulation ===")
    print(f"Arc centre : ({_ARC_CENTER_X:.2f}, {_ARC_CENTER_Y:.2f}) m")
    print(
        f"Arc radius : {_ARC_RADIUS:.2f} m  "
        f"[{_ARC_START_DEG:.0f}° → {_ARC_END_DEG:.0f}°]"
    )
    print(f"z height   : {_Z_HEIGHT:.3f} m")
    print(
        f"Duration   : {_DURATION_S:.0f} s  ({_N_STEPS} steps @ {_RATE_HZ:.0f} Hz)"
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

    print(
        f"\n✅  Simulation PASSED — "
        f"{int(metrics['N_WAYPOINTS'])} waypoints, "
        f"max EE error = {max_err_mm:.2f} mm, "
        f"RMS = {rms_err_mm:.2f} mm"
    )

    if args.results_file:
        env_lines = [f"{k}={v}" for k, v in metrics.items()]
        pathlib.Path(args.results_file).write_text("\n".join(env_lines) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
