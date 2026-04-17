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
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

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

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_CONFIG_PATH = os.path.join(
    _REPO_ROOT, "src", "fret", "config", "benchmark.yaml"
)

# ---------------------------------------------------------------------------
# Inline metric functions (no fret.validation dependency required)
# ---------------------------------------------------------------------------


def _path_length(path: List[Any]) -> float:
    """Compute joint-space arc length of a path.

    Args:
        path: List of joint configuration arrays, each shape ``(DOF,)``.

    Returns:
        Total arc length (sum of L2 norms of consecutive differences).
    """
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        total += float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
    return total


def _path_smoothness(path: List[Any]) -> float:
    """Compute total direction-change angle along a path.

    Args:
        path: List of joint configuration arrays.

    Returns:
        Sum of angles between consecutive segment directions (radians).
        Lower is smoother.
    """
    if len(path) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(path) - 1):
        d1 = np.asarray(path[i]) - np.asarray(path[i - 1])
        d2 = np.asarray(path[i + 1]) - np.asarray(path[i])
        n1, n2 = np.linalg.norm(d1), np.linalg.norm(d2)
        if n1 < 1e-12 or n2 < 1e-12:
            continue
        cos_a = float(np.clip(np.dot(d1, d2) / (n1 * n2), -1.0, 1.0))
        total += math.acos(cos_a)
    return total


# ---------------------------------------------------------------------------
# Inline quality-gate data structures
# ---------------------------------------------------------------------------


@dataclass
class QualityGate:
    """A single pass/fail threshold for a named metric."""

    name: str
    threshold: float
    operator: str  # ">=" | "<=" | ">" | "<" | "=="
    units: str = ""
    description: str = ""


@dataclass
class GateResult:
    """Result of evaluating one gate."""

    gate: QualityGate
    value: float
    passed: bool


@dataclass
class ScenarioReport:
    """Aggregated gate results for one scenario."""

    scenario: str
    results: List[GateResult]

    @property
    def passed(self) -> bool:
        return all(r.passed for r in self.results)


_OPS: Dict[str, Callable[[float, float], bool]] = {
    ">=": lambda v, t: v >= t,
    "<=": lambda v, t: v <= t,
    ">": lambda v, t: v > t,
    "<": lambda v, t: v < t,
    "==": lambda v, t: abs(v - t) < 1e-9,
}


def _evaluate_gates(
    metrics: Dict[str, float], gates: List[QualityGate]
) -> List[GateResult]:
    results = []
    for gate in gates:
        val = metrics.get(gate.name, math.nan)
        op_fn = _OPS.get(gate.operator, lambda v, t: False)
        passed = not math.isnan(val) and op_fn(val, gate.threshold)
        results.append(GateResult(gate=gate, value=val, passed=passed))
    return results


def _format_report(reports: List[ScenarioReport]) -> str:
    lines: List[str] = ["=" * 72, "Quality Gate Report", "=" * 72]
    all_passed = True
    for report in reports:
        status = "PASS" if report.passed else "FAIL"
        if not report.passed:
            all_passed = False
        lines.append(f"\n[{report.scenario}]  {status}")
        for r in report.results:
            mark = "✓" if r.passed else "✗"
            lines.append(
                f"  {mark} {r.gate.name:<22} "
                f"{r.value:>10.4f} {r.gate.units:<10} "
                f"(threshold {r.gate.operator} {r.gate.threshold})"
            )
    lines.append("\n" + "=" * 72)
    lines.append(
        "Overall: "
        + ("ALL GATES PASSED" if all_passed else "SOME GATES FAILED")
    )
    lines.append("=" * 72)
    return "\n".join(lines)


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
            path_lengths.append(_path_length(path))
            smoothnesses.append(_path_smoothness(path))

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
        results = _evaluate_gates(metrics, gates)
        reports.append(ScenarioReport(scenario=scenario_name, results=results))
        for key, val in metrics.items():
            print(f"  {key}: {val:.4f}")

    print("\n" + _format_report(reports))

    all_passed = all(r.passed for r in reports)
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
