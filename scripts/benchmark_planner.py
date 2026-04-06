#!/usr/bin/env python3
"""Benchmark script for the ARCO planner adapter (Issue 05).

Measures RRT-Connect solve time across three reference scenarios:

- **easy**: open space, short path.
- **medium**: partial obstacle wall requiring detour.
- **hard**: narrow passage between two close obstacles.

Usage::

    python3 scripts/benchmark_planner.py

The script prints a summary table and exits with code 0 on success.

No ROS 2 runtime or external dependencies are required.
"""

from __future__ import annotations

import math
import os
import sys
import time
import uuid
from typing import Callable, List, Tuple

# ---------------------------------------------------------------------------
# Make the fret package importable from the repository root.
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.planner_adapter import PlannerAdapter

# ---------------------------------------------------------------------------
# SCARA-like joint limits (R-R-P-R)
# ---------------------------------------------------------------------------

JOINT_LIMITS: List[Tuple[float, float]] = [
    (-math.pi * 132 / 180, math.pi * 132 / 180),
    (-math.pi * 150 / 180, math.pi * 150 / 180),
    (0.0, 0.2),
    (-math.pi, math.pi),
]

# ---------------------------------------------------------------------------
# Scenario definitions
# ---------------------------------------------------------------------------


def _make_validator(
    occupancy: OccupancyAdapter,
) -> Callable[[List[float]], bool]:
    """Return a validator that checks the first two joints as an x/y proxy.

    This is a simplified proxy for a real forward-kinematics collision
    checker.  It maps (q0, q1) to a 2-D point and queries the occupancy
    adapter at z=0.
    """

    def validator(q: List[float]) -> bool:
        return occupancy.is_free([q[0], q[1], 0.0])

    return validator


def _scenario_easy() -> Tuple[dict, OccupancyAdapter]:
    """Easy: open space, short straight-line path."""
    occupancy = OccupancyAdapter(inflation_radius=0.05)
    occupancy.update([])  # no obstacles
    request = {
        "request_id": str(uuid.uuid4()),
        "start_joint_positions": [0.0, 0.0, 0.0, 0.0],
        "goal_joint_positions": [0.3, 0.2, 0.1, 0.1],
        "joint_count": 4,
        "occupancy_stamp": time.time(),
        "timeout": 10.0,
        "planner_config": {
            "algorithm": "rrt_connect",
            "rrt_connect": {"rng_seed": 42},
        },
        "reference_frame": "world",
    }
    return request, occupancy


def _scenario_medium() -> Tuple[dict, OccupancyAdapter]:
    """Medium: single obstacle column forces a detour."""
    occupancy = OccupancyAdapter(inflation_radius=0.05)
    # Column at (0.15, 0.1) in the 2-D proxy space.
    points = [
        [0.15, 0.1 + dy, 0.0]
        for dy in [
            -0.04,
            -0.03,
            -0.02,
            -0.01,
            0.0,
            0.01,
            0.02,
            0.03,
            0.04,
        ]
    ]
    occupancy.update(points)
    request = {
        "request_id": str(uuid.uuid4()),
        "start_joint_positions": [0.0, 0.0, 0.0, 0.0],
        "goal_joint_positions": [0.3, 0.2, 0.1, 0.0],
        "joint_count": 4,
        "occupancy_stamp": time.time(),
        "timeout": 10.0,
        "planner_config": {
            "algorithm": "rrt_connect",
            "rrt_connect": {"rng_seed": 42},
        },
        "reference_frame": "world",
    }
    return request, occupancy


def _scenario_hard() -> Tuple[dict, OccupancyAdapter]:
    """Hard: narrow passage between two obstacle walls."""
    occupancy = OccupancyAdapter(inflation_radius=0.04)
    points = []
    # Wall A: y > 0.08 at x ≈ 0.15
    for y in range(8, 20):
        points.append([0.15, y * 0.01, 0.0])
    # Wall B: y < -0.08 at x ≈ 0.15
    for y in range(-20, -8):
        points.append([0.15, y * 0.01, 0.0])
    occupancy.update(points)
    request = {
        "request_id": str(uuid.uuid4()),
        "start_joint_positions": [0.0, 0.0, 0.0, 0.0],
        "goal_joint_positions": [0.3, 0.0, 0.1, 0.0],
        "joint_count": 4,
        "occupancy_stamp": time.time(),
        "timeout": 30.0,
        "planner_config": {
            "algorithm": "rrt_connect",
            "rrt_connect": {
                "rng_seed": 42,
                "max_iterations": 50_000,
                "step_size": 0.03,
                "goal_bias": 0.15,
            },
        },
        "reference_frame": "world",
    }
    return request, occupancy


SCENARIOS = [
    ("easy", _scenario_easy),
    ("medium", _scenario_medium),
    ("hard", _scenario_hard),
]

# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------

REPEAT = 3  # runs per scenario for averaged timing


def _run_scenario(name: str, scenario_fn: Callable) -> None:
    """Run a single benchmark scenario and print results.

    Args:
        name: Human-readable scenario name.
        scenario_fn: Factory returning (request, occupancy).
    """
    times = []
    statuses = []
    waypoint_counts = []
    node_counts = []

    for run in range(REPEAT):
        request, occupancy = scenario_fn()
        validator = _make_validator(occupancy)
        adapter = PlannerAdapter(
            occupancy_adapter=occupancy,
            joint_limits=JOINT_LIMITS,
            state_validator=validator,
        )
        result = adapter.plan(request)
        times.append(result["solve_time"])
        statuses.append(result["status"])
        waypoint_counts.append(result["waypoint_count"])
        node_counts.append(result["node_count"])

    success_count = statuses.count("success")
    avg_time = sum(times) / len(times)
    min_time = min(times)
    max_time = max(times)
    avg_waypoints = (
        sum(waypoint_counts) / len(waypoint_counts) if success_count else 0
    )
    avg_nodes = sum(node_counts) / len(node_counts)

    print(
        f"[{name:6s}] "
        f"success={success_count}/{REPEAT}  "
        f"avg={avg_time*1000:.1f} ms  "
        f"min={min_time*1000:.1f} ms  "
        f"max={max_time*1000:.1f} ms  "
        f"waypoints={avg_waypoints:.1f}  "
        f"nodes={avg_nodes:.0f}"
    )


def main() -> None:
    """Run all benchmark scenarios and print a summary table."""
    print("=" * 72)
    print("ARCO Planner Adapter Benchmark — RRT-Connect")
    print(f"Repeats per scenario: {REPEAT}")
    print("=" * 72)

    for name, fn in SCENARIOS:
        _run_scenario(name, fn)

    print("=" * 72)
    print("Benchmark complete.")


if __name__ == "__main__":
    main()
