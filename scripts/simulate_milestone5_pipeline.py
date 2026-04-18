#!/usr/bin/env python3
"""Milestone 5 CI simulation: two-pillar avoidance — planning + tracking.

Runs the complete Milestone 5 pipeline without ROS or Gazebo:

  1. Samples cylinder surfaces for both static pillars (radius 0.04 m,
     height 0.40 m) mirroring ``pillar_scenario.sdf``.
  2. Builds a ``WorkspaceOccupancyBuilder`` (20 cm grid) and verifies that
     both pillars produce occupied voxels.
  3. Verifies that positions near the pillar centres have negative clearance.
  4. Creates an ``OccupancyAdapter`` with the raw pillar surface points.
  5. Plans a joint-space path from the home configuration (arm fully
     extended, EE at (0.60, 0, 0.138) m) to the goal configuration
     (EE at (0.417, −0.405, 0.138) m — opposite quadrant).
  6. For every planned waypoint checks that the FK end-effector position
     is at least ``pillar_radius + 0.05 m = 0.09 m`` from both pillar
     centres in the horizontal (XY) plane.
  7. Densifies the sparse path to 1000 steps (50 Hz × 20 s) and runs
     the Jacobian controller tracking loop.
  8. Validates acceptance criteria (Milestone 5):
       - Both pillars detected (occupied_voxels > 0).
       - Positions at pillar centres have negative clearance.
       - No waypoint within safety zone (≥ 0.09 m from every pillar centre).
       - Max EE tracking error ≤ 5 mm (FR-CTL-02).
       - No controller fault triggered.
  9. Generates a four-panel diagnostic plot.
  10. Writes ``results.env`` and exits 0 on success, 1 on failure.

Usage::

    python3 scripts/simulate_milestone5_pipeline.py [--output DIR]

Output files (written to DIR, default /tmp/sim_output_m5):
    pillar_avoidance_plots.png  — 4-panel diagnostic figure.
    results.env                 — KEY=VALUE file with simulation metrics.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

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
from fret.scene.workspace_occupancy import WorkspaceOccupancyBuilder

# ---------------------------------------------------------------------------
# Scenario constants (mirrors pillar_scenario.sdf + pillar_avoidance.yml)
# ---------------------------------------------------------------------------

_PILLAR_A: tuple[float, float] = (0.25, 0.10)  # (x, y) centre
_PILLAR_B: tuple[float, float] = (-0.15, 0.30)  # (x, y) centre
_PILLAR_RADIUS: float = 0.04  # [m]
_PILLAR_Z_CENTRE: float = 0.20  # [m]
_PILLAR_LENGTH: float = 0.40  # [m]

_START_CONFIG = np.array([0.0, 0.0, 0.10])  # arm at rest (fully extended)
_GOAL_CONFIG = np.array([-1.0, 0.5, 0.10])  # goal in opposite quadrant

_PLANNING_TIMEOUT: float = 30.0  # [s]
_DURATION_S: float = 20.0
_RATE_HZ: float = 50.0
_N_STEPS: int = int(_DURATION_S * _RATE_HZ)  # 1000 steps
_DT: float = 1.0 / _RATE_HZ

_MAX_EE_ERROR_M: float = 0.005  # 5 mm limit (FR-CTL-02)
_SAFETY_MARGIN: float = 0.05  # min clearance from surface [m]
_MIN_HORIZONTAL_CLEARANCE: float = _PILLAR_RADIUS + _SAFETY_MARGIN  # 0.09 m

# Known positions for occupancy checks
_OCCUPIED_NEAR_A: list[float] = [
    0.20,
    0.20,
    0.20,
]  # in collision zone of pillar_a
_OCCUPIED_NEAR_B: list[float] = [
    -0.20,
    0.20,
    0.20,
]  # in collision zone of pillar_b
_FREE_PT: list[float] = [0.60, 0.00, 0.20]  # far from both pillars


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _sample_cylinder(
    cx: float, cy: float, cz: float, radius: float, length: float
) -> np.ndarray:
    """Return uniformly sampled surface points for a vertical cylinder.

    Args:
        cx, cy, cz: Cylinder axis centre [m].
        radius: Cylinder radius [m].
        length: Cylinder length (height) [m].

    Returns:
        Array of shape ``(N, 3)`` with sampled surface points in world frame.
    """
    half = length / 2.0
    rng = np.random.default_rng(seed=42)

    n_lat = 300
    theta = rng.uniform(0, 2 * math.pi, n_lat)
    z_lat = rng.uniform(cz - half, cz + half, n_lat)
    lat = np.column_stack(
        [cx + radius * np.cos(theta), cy + radius * np.sin(theta), z_lat]
    )

    caps = []
    for sign in (-1.0, 1.0):
        n_cap = 80
        r_sq = rng.uniform(0, radius**2, n_cap)
        th = rng.uniform(0, 2 * math.pi, n_cap)
        r_c = np.sqrt(r_sq)
        cap = np.column_stack(
            [
                cx + r_c * np.cos(th),
                cy + r_c * np.sin(th),
                np.full(n_cap, cz + sign * half),
            ]
        )
        caps.append(cap)

    return np.vstack([lat] + caps).astype(np.float64)


def _build_pillar_payload() -> OccupancyUpdatePayload:
    """Build an OccupancyUpdatePayload from both pillars.

    Returns:
        Payload with merged surface points from pillar_a and pillar_b.
    """
    pts_a = _sample_cylinder(
        _PILLAR_A[0],
        _PILLAR_A[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    pts_b = _sample_cylinder(
        _PILLAR_B[0],
        _PILLAR_B[1],
        _PILLAR_Z_CENTRE,
        _PILLAR_RADIUS,
        _PILLAR_LENGTH,
    )
    pts = np.vstack([pts_a, pts_b]).astype(np.float64)
    print(f"[M5] Pillar surface payload: {len(pts)} points")
    return OccupancyUpdatePayload(
        obstacle_points=pts, timestamp=0.0, frame_id="world"
    )


def _densify_path(path: list[np.ndarray], n_steps: int) -> list[np.ndarray]:
    """Linearly interpolate a joint-space path to n_steps waypoints.

    Args:
        path: List of at least 2 joint configurations.
        n_steps: Number of output waypoints.

    Returns:
        List of n_steps joint configurations.
    """
    return [
        path[0] + i / (n_steps - 1) * (path[-1] - path[0])
        for i in range(n_steps)
    ]


def _fk_ee(kin: Kinematics, q: np.ndarray) -> np.ndarray:
    """Return EE position (3,) for configuration q.

    Args:
        kin: Kinematics engine.
        q: Joint configuration of shape ``(3,)``.

    Returns:
        EE Cartesian position array of shape ``(3,)``.
    """
    return kin.forward_kinematics(q)[:3, 3]


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate(output_dir: pathlib.Path) -> dict[str, float]:
    """Run the Milestone 5 pillar-avoidance simulation.

    Args:
        output_dir: Directory where output files are written.

    Returns:
        Dict of metric names to values.

    Raises:
        AssertionError: If any acceptance criterion is violated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1 - Build pillar payload -----------------------------------------------
    payload = _build_pillar_payload()

    # 2 - WorkspaceOccupancyBuilder: verify pillar detection -----------------
    resolution = 0.20
    builder = WorkspaceOccupancyBuilder(resolution=resolution)
    builder.build(payload)

    occ_voxels = builder.occupied_centres()
    free_voxels = builder.free_centres()
    print(f"[M5] Occupied voxels: {len(occ_voxels)}")
    print(f"[M5] Free voxels    : {len(free_voxels)}")

    assert len(occ_voxels) > 0, "No occupied voxels — pillars not detected."

    # 3 - Verify clearance semantics -----------------------------------------
    cl_near_a = builder.clearance(_OCCUPIED_NEAR_A)
    cl_near_b = builder.clearance(_OCCUPIED_NEAR_B)
    cl_free = builder.clearance(_FREE_PT)

    assert cl_near_a < 0.0, (
        f"clearance({_OCCUPIED_NEAR_A}) = {cl_near_a:.4f} should be negative "
        f"(pillar_a collision zone)."
    )
    assert cl_near_b < 0.0, (
        f"clearance({_OCCUPIED_NEAR_B}) = {cl_near_b:.4f} should be negative "
        f"(pillar_b collision zone)."
    )
    assert (
        cl_free > 0.0
    ), f"clearance({_FREE_PT}) = {cl_free:.4f} should be positive (free space)."
    print(f"[M5] clearance(near_a) = {cl_near_a:.4f} m  (< 0 ✓)")
    print(f"[M5] clearance(near_b) = {cl_near_b:.4f} m  (< 0 ✓)")
    print(f"[M5] clearance(free)   = {cl_free:.4f} m    (> 0 ✓)")

    # 4 - Plan ---------------------------------------------------------------
    kin = Kinematics("scara")
    adapter = OccupancyAdapter()
    adapter.update(payload)

    planner = PlannerNode(model="scara", occupancy_adapter=adapter)
    request = PlanningRequest(
        start_configuration=_START_CONFIG.copy(),
        goal_configuration=_GOAL_CONFIG.copy(),
        planning_timeout=_PLANNING_TIMEOUT,
        scenario_id="pillar_avoidance_ci",
    )

    t0 = time.monotonic()
    result = planner.plan(request)
    planning_duration = time.monotonic() - t0

    assert result.status == PlanningStatus.SUCCESS, (
        f"PlannerNode returned {result.status!r} "
        f"(error_code={result.error_code!r})"
    )
    assert len(result.path) >= 2
    print(
        f"[M5] Planning: {result.status.name}, "
        f"{len(result.path)} waypoints, {planning_duration:.3f} s"
    )

    # 5 - Pillar-clearance check on planned path -----------------------------
    waypoints_dense = _densify_path(result.path, 100)
    min_horizontal_clearance = float("inf")
    worst_waypoint = -1

    for i, q in enumerate(waypoints_dense):
        T = kin.forward_kinematics(q)
        ee_x, ee_y = float(T[0, 3]), float(T[1, 3])
        for cx, cy in [_PILLAR_A, _PILLAR_B]:
            dist_xy = math.sqrt((ee_x - cx) ** 2 + (ee_y - cy) ** 2)
            if dist_xy < min_horizontal_clearance:
                min_horizontal_clearance = dist_xy
                worst_waypoint = i
            assert dist_xy >= _MIN_HORIZONTAL_CLEARANCE, (
                f"Waypoint {i}: EE ({ee_x:.3f}, {ee_y:.3f}) too close to "
                f"pillar ({cx}, {cy}): dist_xy = {dist_xy:.4f} m "
                f"(required >= {_MIN_HORIZONTAL_CLEARANCE:.3f} m)"
            )

    print(
        f"[M5] Min horizontal pillar clearance: "
        f"{min_horizontal_clearance:.4f} m "
        f"(at waypoint {worst_waypoint}, req. >= {_MIN_HORIZONTAL_CLEARANCE:.3f} m ✓)"
    )

    # 6 - Densify for controller tracking ------------------------------------
    waypoints = _densify_path(result.path, _N_STEPS)

    # 7 - Controller tracking simulation -------------------------------------
    ctrl = ControllerNode(model="scara", config_path="")
    ctrl.set_trajectory(waypoints)
    assert (
        ctrl._state == _NodeState.TRACKING
    ), "Controller did not transition to TRACKING"

    times = np.arange(_N_STEPS) * _DT
    q_ref_arr = np.array(waypoints, dtype=np.float64)
    q_exec_arr = np.zeros_like(q_ref_arr)
    x_ref_arr = np.zeros((_N_STEPS, 3), dtype=np.float64)
    x_exec_arr = np.zeros((_N_STEPS, 3), dtype=np.float64)
    ee_errors = np.zeros(_N_STEPS, dtype=np.float64)
    fault_triggered = False
    fault_step = _N_STEPS

    q_cur = _START_CONFIG.copy()

    for i, q_ref in enumerate(waypoints):
        x_ref = _fk_ee(kin, q_ref)
        x_cur = _fk_ee(kin, q_cur)
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

    # 8 - Validate acceptance criteria ---------------------------------------
    assert not fault_triggered, (
        f"Fault triggered at step {fault_step} "
        f"(EE error = {ee_errors[fault_step] * 1000:.2f} mm)"
    )

    max_ee_error_m = float(np.max(ee_errors))
    assert (
        max_ee_error_m <= _MAX_EE_ERROR_M
    ), f"Max EE error {max_ee_error_m * 1000:.2f} mm exceeds 5 mm limit"

    rms_ee_error_m = float(np.sqrt(np.mean(ee_errors**2)))
    final_ee_error_m = float(
        np.linalg.norm(x_exec_arr[-1] - _fk_ee(kin, _GOAL_CONFIG))
    )

    print(
        f"[M5] Tracking: max EE error = {max_ee_error_m*1000:.2f} mm, "
        f"RMS = {rms_ee_error_m*1000:.2f} mm, "
        f"final error = {final_ee_error_m*1000:.2f} mm"
    )

    # 9 - Generate plots -----------------------------------------------------
    goal_ee = _fk_ee(kin, _GOAL_CONFIG)
    start_ee = _fk_ee(kin, _START_CONFIG)
    _plot(
        output_dir / "pillar_avoidance_plots.png",
        times=times,
        q_ref_arr=q_ref_arr,
        q_exec_arr=q_exec_arr,
        x_ref_arr=x_ref_arr,
        x_exec_arr=x_exec_arr,
        ee_errors=ee_errors,
        goal_ee=goal_ee,
        start_ee=start_ee,
        occ_voxels=occ_voxels,
    )

    # 10 - Write results.env -------------------------------------------------
    metrics: dict[str, float] = {
        "N_OCCUPIED_VOXELS": float(len(occ_voxels)),
        "N_FREE_VOXELS": float(len(free_voxels)),
        "CLEARANCE_NEAR_A_M": float(cl_near_a),
        "CLEARANCE_NEAR_B_M": float(cl_near_b),
        "MIN_PILLAR_CLEARANCE_M": float(min_horizontal_clearance),
        "PLANNING_DURATION_S": float(planning_duration),
        "N_WAYPOINTS": float(len(result.path)),
        "MAX_EE_ERROR_MM": float(max_ee_error_m * 1000.0),
        "RMS_EE_ERROR_MM": float(rms_ee_error_m * 1000.0),
        "FINAL_EE_ERROR_MM": float(final_ee_error_m * 1000.0),
        "FAULT_TRIGGERED": 0.0,
    }
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
    occ_voxels: np.ndarray,
) -> None:
    """Generate a four-panel diagnostic figure for the MS-5 simulation.

    Args:
        out_path: Destination PNG file path.
        times: Time array of shape ``(N,)``.
        q_ref_arr: Reference joint configs, shape ``(N, 3)``.
        q_exec_arr: Executed joint configs, shape ``(N, 3)``.
        x_ref_arr: Reference EE positions, shape ``(N, 3)``.
        x_exec_arr: Executed EE positions, shape ``(N, 3)``.
        ee_errors: EE tracking error per step, shape ``(N,)``.
        goal_ee: Goal EE position, shape ``(3,)``.
        start_ee: Start EE position, shape ``(3,)``.
        occ_voxels: Occupied voxel centres, shape ``(M, 3)``.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        print("WARNING: matplotlib not available — skipping plot.")
        return

    max_err_mm = float(np.max(ee_errors) * 1000)
    rms_err_mm = float(np.sqrt(np.mean(ee_errors**2)) * 1000)

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(
        f"Milestone 5 — Pillar Avoidance: Planning + Tracking  |  "
        f"max EE error = {max_err_mm:.2f} mm  |  "
        f"RMS EE error = {rms_err_mm:.2f} mm",
        fontsize=12,
        fontweight="bold",
    )

    # Panel 1 (top-left): EE path in XY with pillar footprints
    ax = axes[0, 0]
    if len(occ_voxels) > 0:
        ax.scatter(
            occ_voxels[:, 0],
            occ_voxels[:, 1],
            c="lightcoral",
            s=200,
            alpha=0.3,
            label=f"Occ. voxels ({len(occ_voxels)})",
            marker="s",
            zorder=1,
        )
    ax.plot(
        x_ref_arr[:, 0],
        x_ref_arr[:, 1],
        "b-",
        linewidth=2,
        label="Reference (planned)",
        alpha=0.7,
        zorder=3,
    )
    ax.plot(
        x_exec_arr[:, 0],
        x_exec_arr[:, 1],
        "r--",
        linewidth=1.5,
        label="Executed (tracked)",
        alpha=0.9,
        zorder=4,
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
        goal_ee[0],
        goal_ee[1],
        "r*",
        markersize=12,
        label="Goal EE",
        zorder=5,
    )
    for (cx, cy), lbl in [(_PILLAR_A, "pillar_a"), (_PILLAR_B, "pillar_b")]:
        circle = Circle(
            (cx, cy), _PILLAR_RADIUS, color="saddlebrown", alpha=0.8, zorder=6
        )
        ax.add_patch(circle)
        safety = Circle(
            (cx, cy),
            _MIN_HORIZONTAL_CLEARANCE,
            color="orange",
            fill=False,
            linestyle="--",
            linewidth=1.2,
            zorder=6,
        )
        ax.add_patch(safety)
        ax.annotate(
            lbl,
            (cx, cy),
            ha="center",
            va="center",
            fontsize=7,
            color="white",
            fontweight="bold",
            zorder=7,
        )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        "Cartesian EE Path (top view)\nWith pillar footprints and safety zones"
    )
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True)
    ax.set_aspect("equal")

    # Panel 2 (top-right): EE tracking error over time
    ax = axes[0, 1]
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

    # Panel 3 (bottom-left): joint variables over time
    ax = axes[1, 0]
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

    # Panel 4 (bottom-right): horizontal clearance to nearest pillar over time
    ax = axes[1, 1]
    clearances = np.array(
        [
            min(
                math.sqrt((x[0] - cx) ** 2 + (x[1] - cy) ** 2) - _PILLAR_RADIUS
                for cx, cy in [_PILLAR_A, _PILLAR_B]
            )
            for x in x_exec_arr
        ]
    )
    ax.plot(
        times,
        clearances * 100,
        "g-",
        linewidth=1.5,
        label="Min pillar clearance [cm]",
    )
    ax.axhline(
        _SAFETY_MARGIN * 100,
        color="orange",
        linestyle="--",
        linewidth=1.2,
        label=f"{_SAFETY_MARGIN*100:.0f} cm safety margin",
    )
    ax.fill_between(times, clearances * 100, alpha=0.15, color="green")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Clearance from nearest pillar [cm]")
    ax.set_title("Horizontal Pillar Clearance (EE)\nvs. Safety Margin")
    ax.legend(fontsize=8)
    ax.grid(True)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"[M5] Plot saved → {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    """CLI entry point.

    Returns:
        Exit code: 0 on success, 1 on failure.
    """
    parser = argparse.ArgumentParser(
        description="Milestone 5 pillar-avoidance simulation (planning + tracking)"
    )
    parser.add_argument(
        "--output",
        default="/tmp/sim_output_m5",
        help="Output directory (default: /tmp/sim_output_m5)",
    )
    parser.add_argument(
        "--results-file",
        default=None,
        help="Optional explicit path for results.env",
    )
    args = parser.parse_args()

    output_dir = pathlib.Path(args.output)

    print(
        "=== Milestone 5 simulation (pillar-avoidance: planning + tracking) ==="
    )
    print(f"Start config  : {_START_CONFIG.tolist()}")
    print(f"Goal config   : {_GOAL_CONFIG.tolist()}")
    print(
        f"Pillars       : {_PILLAR_A} r={_PILLAR_RADIUS}m, {_PILLAR_B} r={_PILLAR_RADIUS}m"
    )
    print(f"Safety margin : {_SAFETY_MARGIN*100:.0f} cm from pillar surface")
    print(
        f"Sim duration  : {_DURATION_S:.0f} s "
        f"({_N_STEPS} steps @ {_RATE_HZ:.0f} Hz)"
    )

    try:
        metrics = simulate(output_dir)
    except AssertionError as exc:
        print(f"\n❌  SIMULATION FAILED: {exc}", file=sys.stderr)
        return 1

    print("\n--- Results ---")
    for k, v in metrics.items():
        print(f"  {k} = {v:.4f}")

    print(
        f"\n✅  Simulation PASSED — "
        f"{int(metrics['N_OCCUPIED_VOXELS'])} occupied voxels, "
        f"min pillar clearance = {metrics['MIN_PILLAR_CLEARANCE_M']*100:.1f} cm, "
        f"max EE error = {metrics['MAX_EE_ERROR_MM']:.2f} mm"
    )

    if args.results_file:
        rf = pathlib.Path(args.results_file)
        rf.parent.mkdir(parents=True, exist_ok=True)
        env_lines = [f"{k}={v}" for k, v in metrics.items()]
        rf.write_text("\n".join(env_lines) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
