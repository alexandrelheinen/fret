#!/usr/bin/env python3
"""Generate trajectory plots for every benchmark scenario.

Runs the RRT-Connect planner on each scenario defined in
``src/fret/config/benchmark.yaml`` and produces one PNG per scenario
showing the planned path in the 2-D joint-space proxy (q0 × q1), the
obstacle cloud, start and goal configurations.

Output PNG files are written to ``--output-dir`` (default:
``/tmp/trajectory_plots``).  The script exits with code 0 on success and
code 1 on any hard error.

**Usage**::

    python3 scripts/plot_trajectories.py
    python3 scripts/plot_trajectories.py --output-dir /tmp/my_plots

No ROS 2 runtime is required.  ``matplotlib`` must be installed
(``pip install matplotlib``).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Make the fret package importable from the repository root.
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

import matplotlib  # type: ignore[import]
import yaml  # type: ignore[import]

matplotlib.use("Agg")  # non-interactive backend – no display required
import matplotlib.patches as mpatches  # type: ignore[import]
import matplotlib.pyplot as plt  # type: ignore[import]

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.planner_adapter import PlannerAdapter

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_CONFIG_PATH = os.path.join(
    _REPO_ROOT, "src", "fret", "config", "benchmark.yaml"
)

# ---------------------------------------------------------------------------
# Joint limits (fallback; config values take precedence)
# ---------------------------------------------------------------------------
_DEFAULT_JOINT_LIMITS: List[Tuple[float, float]] = [
    (-math.pi * 132 / 180, math.pi * 132 / 180),
    (-math.pi * 150 / 180, math.pi * 150 / 180),
    (0.0, 0.2),
    (-math.pi, math.pi),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_config(path: str) -> Dict[str, Any]:
    """Load and return the ``benchmark`` section of the YAML config."""
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    return raw["benchmark"]


def _build_joint_limits(
    cfg: Dict[str, Any],
) -> List[Tuple[float, float]]:
    """Extract joint limits from config or fall back to defaults."""
    raw = cfg.get("joint_limits")
    if raw:
        return [(float(lo), float(hi)) for lo, hi in raw]
    return _DEFAULT_JOINT_LIMITS


def _make_validator(
    occupancy: OccupancyAdapter,
) -> Any:
    """Return a state validator for the first two joints as x/y proxy."""

    def validator(q: List[float]) -> bool:
        return occupancy.is_free([q[0], q[1], 0.0])

    return validator


def _plan_once(
    obstacles: List[List[float]],
    inflation_radius: float,
    start: List[float],
    goal: List[float],
    joint_limits: List[Tuple[float, float]],
    planner_config: Dict[str, Any],
    timeout: float,
) -> Dict[str, Any]:
    """Run the planner once and return the raw result dict."""
    occupancy = OccupancyAdapter(inflation_radius=inflation_radius)
    occupancy.update(obstacles)
    validator = _make_validator(occupancy)
    adapter = PlannerAdapter(
        occupancy_adapter=occupancy,
        joint_limits=joint_limits,
        state_validator=validator,
    )
    request = {
        "request_id": str(uuid.uuid4()),
        "start_joint_positions": start,
        "goal_joint_positions": goal,
        "joint_count": len(joint_limits),
        "occupancy_stamp": time.time(),
        "timeout": timeout,
        "planner_config": planner_config,
        "reference_frame": "world",
    }
    return adapter.plan(request)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def _plot_scenario(
    name: str,
    obstacles: List[List[float]],
    inflation_radius: float,
    start: List[float],
    goal: List[float],
    joint_limits: List[Tuple[float, float]],
    path: Optional[List[List[float]]],
    status: str,
    solve_time: float,
    output_path: str,
) -> None:
    """Render a single scenario plot and save it as a PNG.

    The plot shows the 2-D joint-space proxy (q0 on x-axis, q1 on y-axis):
    - grey shaded inflation circles around each obstacle point
    - obstacle points (red dots)
    - planned path (blue line with circle markers)
    - start (green star) and goal (orange star)

    Args:
        name: Scenario name (used as figure title).
        obstacles: Obstacle cloud points ``[[x, y, z], …]``.
        inflation_radius: Obstacle inflation radius (metres/radians).
        start: Start joint configuration.
        goal: Goal joint configuration.
        joint_limits: Per-joint ``(lower, upper)`` bounds.
        path: Planned path waypoints, or ``None`` when planning failed.
        status: Planner status string (``"success"`` or ``"failure"``).
        solve_time: Planner wall-clock time in seconds.
        output_path: Absolute path for the output PNG file.
    """
    fig, ax = plt.subplots(figsize=(7, 6))

    # Axis limits from joint bounds (q0, q1)
    q0_lo, q0_hi = joint_limits[0]
    q1_lo, q1_hi = joint_limits[1]
    ax.set_xlim(q0_lo, q0_hi)
    ax.set_ylim(q1_lo, q1_hi)

    # Obstacle cloud
    if obstacles:
        obs_x = [pt[0] for pt in obstacles]
        obs_y = [pt[1] for pt in obstacles]
        # Draw inflation radius circles (translucent)
        for cx, cy in zip(obs_x, obs_y):
            circle = plt.Circle(
                (cx, cy),
                inflation_radius,
                color="tomato",
                alpha=0.25,
                linewidth=0,
            )
            ax.add_patch(circle)
        ax.scatter(
            obs_x,
            obs_y,
            c="red",
            s=25,
            zorder=3,
            label="Obstacles",
        )

    # Planned path
    if path and status == "success":
        px = [wp[0] for wp in path]
        py = [wp[1] for wp in path]
        ax.plot(
            px,
            py,
            "b-o",
            markersize=3,
            linewidth=1.5,
            zorder=4,
            label=f"Path ({len(path)} waypoints)",
        )

    # Start / goal markers
    ax.scatter(
        [start[0]],
        [start[1]],
        c="green",
        s=180,
        marker="*",
        zorder=5,
        label="Start",
    )
    ax.scatter(
        [goal[0]],
        [goal[1]],
        c="darkorange",
        s=180,
        marker="*",
        zorder=5,
        label="Goal",
    )

    # Labels
    color = "green" if status == "success" else "red"
    ax.set_title(
        f"Scenario: {name}  |  {status.upper()}  ({solve_time:.3f} s)",
        fontsize=11,
        color=color,
        fontweight="bold",
    )
    ax.set_xlabel("q₀ – joint_arm_0 [rad]", fontsize=9)
    ax.set_ylabel("q₁ – joint_arm_1 [rad]", fontsize=9)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.6)

    fig.tight_layout()
    fig.savefig(output_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def main() -> int:
    """Generate one trajectory PNG per benchmark scenario.

    Returns:
        0 on success, 1 on any hard error.
    """
    parser = argparse.ArgumentParser(
        description="Generate trajectory plots for ARCO benchmark scenarios."
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/trajectory_plots",
        help="Directory for output PNG files (created if absent).",
    )
    parser.add_argument(
        "--config",
        default=_CONFIG_PATH,
        help="Path to benchmark.yaml (default: src/fret/config/benchmark.yaml).",
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    cfg = _load_config(args.config)
    joint_limits = _build_joint_limits(cfg)
    scenarios_cfg: Dict[str, Any] = cfg.get("scenarios", {})

    print("=" * 60)
    print("ARCO-FRET Trajectory Plotter")
    print(f"Config    : {args.config}")
    print(f"Output dir: {args.output_dir}")
    print("=" * 60)

    generated: List[str] = []

    for name, scenario_cfg in scenarios_cfg.items():
        print(f"\n[{name}] Planning …", flush=True)
        obstacles: List[List[float]] = scenario_cfg.get("obstacles", [])
        inflation_radius: float = float(
            scenario_cfg.get("inflation_radius", 0.05)
        )
        start: List[float] = [float(v) for v in scenario_cfg["start"]]
        goal: List[float] = [float(v) for v in scenario_cfg["goal"]]
        timeout: float = float(scenario_cfg.get("timeout", 10.0))
        planner_config: Dict[str, Any] = scenario_cfg.get("planner_config", {})

        try:
            result = _plan_once(
                obstacles=obstacles,
                inflation_radius=inflation_radius,
                start=start,
                goal=goal,
                joint_limits=joint_limits,
                planner_config=planner_config,
                timeout=timeout,
            )
        except (ValueError, RuntimeError, KeyError) as exc:
            import traceback

            print(f"  ERROR: planner raised an exception: {exc}")
            traceback.print_exc()
            return 1

        path: Optional[List[List[float]]] = result.get("path")
        status: str = result.get("status", "failure")
        solve_time: float = float(result.get("solve_time", 0.0))
        print(f"  status={status}  solve_time={solve_time:.3f} s")

        out_file = os.path.join(args.output_dir, f"trajectory_{name}.png")
        _plot_scenario(
            name=name,
            obstacles=obstacles,
            inflation_radius=inflation_radius,
            start=start,
            goal=goal,
            joint_limits=joint_limits,
            path=path,
            status=status,
            solve_time=solve_time,
            output_path=out_file,
        )
        generated.append(out_file)

    print(f"\nDone — {len(generated)} plot(s) written to {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
