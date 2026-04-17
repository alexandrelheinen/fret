#!/usr/bin/env python3
"""Automated quality gate validation script for ARCO-FRET.

Loads the benchmark configuration from ``src/fret/config/benchmark.yaml``,
runs each scenario the configured number of times, computes metrics, evaluates
quality gates, and prints a pass/fail report.

**Metrics computed:**

- ``success_rate``   — fraction of runs that returned ``SUCCESS``.
- ``avg_latency_s`` — mean wall-clock planning time across all runs (seconds).
- ``path_length``   — mean joint-space arc length of successful paths.
- ``smoothness``    — mean total direction-change angle of successful paths.

**Exit codes:**

- ``0`` — all quality gates passed.
- ``1`` — one or more quality gates failed (or the script encountered an
  unexpected error).

**Usage**::

    python3 scripts/validate_quality_gates.py

No ROS 2 runtime or external dependencies are required beyond the packages
declared in ``pyproject.toml``.

See also:
    ``src/fret/config/benchmark.yaml``  — scenario definitions and thresholds.
"""

from __future__ import annotations

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

import numpy as np
import yaml  # type: ignore[import]

from fret.interfaces import (
    OccupancyUpdatePayload,
    PlanningRequest,
    PlanningStatus,
)
from fret.planning.planner_node import PlannerNode
from fret.scene.occupancy_adapter import OccupancyAdapter
from fret.validation import (
    GateResult,
    QualityGate,
    ScenarioReport,
    evaluate_gates,
    format_report,
    path_length,
    path_smoothness,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_CONFIG_PATH = os.path.join(
    _REPO_ROOT, "src", "fret", "config", "benchmark.yaml"
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_config(path: str) -> Dict[str, Any]:
    """Load and return the benchmark YAML configuration.

    Args:
        path: Absolute path to ``benchmark.yaml``.

    Returns:
        Parsed configuration dictionary rooted at the ``benchmark`` key.

    Raises:
        SystemExit: If the file cannot be read or parsed.
    """
    try:
        with open(path, encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
        return raw["benchmark"]
    except (OSError, KeyError, yaml.YAMLError) as exc:
        print(f"ERROR: Cannot load benchmark config from {path!r}: {exc}")
        sys.exit(1)


def _gates_from_config(
    gate_cfg: Dict[str, Any],
) -> List[QualityGate]:
    """Build :class:`QualityGate` objects from a scenario's gate config.

    Args:
        gate_cfg: Dict mapping metric names to gate specification dicts.

    Returns:
        Ordered list of :class:`QualityGate` objects.
    """
    gates: List[QualityGate] = []
    for metric_name, spec in gate_cfg.items():
        gates.append(
            QualityGate(
                name=metric_name,
                threshold=float(spec["threshold"]),
                operator=spec["operator"],
                units=spec.get("units", ""),
                description=spec.get("description", ""),
            )
        )
    return gates


# ---------------------------------------------------------------------------
# Per-scenario benchmark runner
# ---------------------------------------------------------------------------


def _run_scenario(
    scenario_id: str,
    scenario_cfg: Dict[str, Any],
    joint_limits: List[Tuple[float, float]],
    repeat: int,
) -> Dict[str, float]:
    """Run a single benchmark scenario and return aggregated metrics.

    Args:
        scenario_id: Scenario name forwarded to ``PlanningRequest`` (also
            used for diagnostic messages).
        scenario_cfg: Scenario sub-dict from ``benchmark.yaml``.
        joint_limits: Per-joint ``(lower, upper)`` bounds (unused by
            ``PlannerNode`` directly; kept for forward compatibility).
        repeat: Number of repeated runs.

    Returns:
        Dict with keys: ``success_rate``, ``avg_latency_s``,
        ``path_length``, ``smoothness``.
    """
    obstacles: List[List[float]] = scenario_cfg.get("obstacles", [])
    start: List[float] = [float(v) for v in scenario_cfg["start"]]
    goal: List[float] = [float(v) for v in scenario_cfg["goal"]]
    timeout: float = float(scenario_cfg.get("timeout", 10.0))

    latencies: List[float] = []
    path_lengths: List[float] = []
    smoothnesses: List[float] = []
    success_count = 0

    # Build obstacle point cloud (Nx3 float64) from list of [x,y,z] coords.
    # Each entry in obstacles must be a 3-element list/tuple [x, y, z].
    if obstacles:
        pts = np.array(obstacles, dtype=np.float64).reshape(-1, 3)
    else:
        pts = np.empty((0, 3), dtype=np.float64)

    payload = OccupancyUpdatePayload(
        obstacle_points=pts,
        timestamp=time.time(),
        frame_id="world",
    )

    for _ in range(repeat):
        occupancy_adapter = OccupancyAdapter()
        occupancy_adapter.update(payload)

        planner = PlannerNode(
            model="scara", occupancy_adapter=occupancy_adapter
        )

        request = PlanningRequest(
            start_configuration=np.array(start, dtype=np.float64),
            goal_configuration=np.array(goal, dtype=np.float64),
            planning_timeout=timeout,
            scenario_id=scenario_id,
        )

        result = planner.plan(request)
        latencies.append(result.planning_duration)

        if result.status == PlanningStatus.SUCCESS:
            success_count += 1
            path = result.path
            path_lengths.append(path_length(path))
            smoothnesses.append(path_smoothness(path))

    success_rate = success_count / repeat
    avg_latency = sum(latencies) / len(latencies)
    avg_path_length = (
        sum(path_lengths) / len(path_lengths) if path_lengths else math.nan
    )
    avg_smoothness = (
        sum(smoothnesses) / len(smoothnesses) if smoothnesses else math.nan
    )

    return {
        "success_rate": success_rate,
        "avg_latency_s": avg_latency,
        "path_length": avg_path_length,
        "smoothness": avg_smoothness,
    }


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def main() -> int:
    """Run quality gate validation across all benchmark scenarios.

    Returns:
        ``0`` if all gates pass, ``1`` otherwise.
    """
    cfg = _load_config(_CONFIG_PATH)
    repeat: int = int(cfg.get("repeat", 3))
    raw_limits = cfg.get("joint_limits", [])
    joint_limits: List[Tuple[float, float]] = [
        (float(lo), float(hi)) for lo, hi in raw_limits
    ]
    scenarios_cfg: Dict[str, Any] = cfg.get("scenarios", {})
    gates_cfg: Dict[str, Any] = cfg.get("quality_gates", {})

    print("=" * 72)
    print("ARCO-FRET Quality Gate Validation")
    print(f"Config : {_CONFIG_PATH}")
    print(f"Repeats: {repeat}")
    print("=" * 72)

    reports: List[ScenarioReport] = []

    for scenario_name, scenario_cfg in scenarios_cfg.items():
        print(f"\n[{scenario_name}] Running {repeat} repeat(s) …", flush=True)
        metrics = _run_scenario(
            scenario_id=scenario_name,
            scenario_cfg=scenario_cfg,
            joint_limits=joint_limits,
            repeat=repeat,
        )
        gates = _gates_from_config(gates_cfg.get(scenario_name, {}))
        results = evaluate_gates(metrics, gates)
        reports.append(ScenarioReport(scenario=scenario_name, results=results))
        for key, val in metrics.items():
            print(f"  {key}: {val:.4f}")

    print("\n" + format_report(reports))

    all_passed = all(r.passed for r in reports)
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
