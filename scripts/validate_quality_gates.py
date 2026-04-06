#!/usr/bin/env python3
"""Automated quality gate validation script for ARCO-FRET (Issue 09).

Loads the benchmark configuration from ``src/fret/config/benchmark.yaml``,
runs each scenario the configured number of times, computes all metrics,
evaluates the quality gates, and prints a pass/fail report.

**Metrics computed:**

- ``success_rate``    — fraction of runs that returned ``status=="success"``.
- ``avg_latency_s``  — mean wall-clock solve time across all runs (seconds).
- ``path_length``    — mean joint-space arc length of successful paths.
- ``smoothness``     — mean total direction-change angle of successful paths.
- ``min_clearance_m``— mean minimum obstacle clearance of successful paths.

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
    ``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md`` —
    full specification.
"""

from __future__ import annotations

import math
import os
import sys
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Make the fret package importable from the repository root.
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

import yaml  # type: ignore[import]

from fret.perception.occupancy_adapter import OccupancyAdapter
from fret.planning.planner_adapter import PlannerAdapter
from fret.validation.metrics import (
    min_obstacle_clearance,
    path_length,
    path_smoothness,
)
from fret.validation.quality_gates import (
    QualityGate,
    ScenarioReport,
    evaluate_gates,
    format_report,
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_CONFIG_PATH = os.path.join(
    _REPO_ROOT, "src", "fret", "config", "benchmark.yaml"
)

#: Joint-space degrees of freedom for the SCARA-like test robot (R-R-P-R).
#: Each entry is a ``(lower, upper)`` bound pair:
#:   - joint_arm_0    [rad]: revolute, ±132°
#:   - joint_arm_1    [rad]: revolute, ±150°
#:   - joint_extension [m]:  prismatic, 0–0.2 m
#:   - joint_tool_rotate [rad]: revolute, ±180°
#: Loaded from benchmark.yaml; this constant is the fallback default.
_DEFAULT_JOINT_LIMITS: List[Tuple[float, float]] = [
    (-math.pi * 132 / 180, math.pi * 132 / 180),  # joint_arm_0 [rad]
    (-math.pi * 150 / 180, math.pi * 150 / 180),  # joint_arm_1 [rad]
    (0.0, 0.2),  # joint_extension [m]
    (-math.pi, math.pi),  # joint_tool_rotate [rad]
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_config(path: str) -> Dict[str, Any]:
    """Load and return the benchmark YAML configuration.

    Args:
        path: Absolute path to the ``benchmark.yaml`` file.

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


def _build_joint_limits(
    cfg: Dict[str, Any],
) -> List[Tuple[float, float]]:
    """Extract joint limits from config or fall back to defaults.

    Args:
        cfg: Root ``benchmark`` config dict.

    Returns:
        List of ``(lower, upper)`` bound pairs.
    """
    raw = cfg.get("joint_limits")
    if raw:
        return [(float(lo), float(hi)) for lo, hi in raw]
    return _DEFAULT_JOINT_LIMITS


def _make_validator(
    occupancy: OccupancyAdapter,
) -> Callable[[List[float]], bool]:
    """Return a state validator that proxies (q0, q1) → occupancy query.

    Maps the first two joints to an x/y point and checks occupancy at
    z=0.  This is the same proxy used in ``scripts/benchmark_planner.py``.

    Args:
        occupancy: Populated :class:`OccupancyAdapter`.

    Returns:
        Callable ``(q: list[float]) -> bool``.
    """

    def validator(q: List[float]) -> bool:
        return occupancy.is_free([q[0], q[1], 0.0])

    return validator


def _make_clearance_fn(
    occupancy: OccupancyAdapter,
) -> Callable[[List[float]], float]:
    """Return a clearance function that proxies (q0, q1) → clearance.

    Args:
        occupancy: Populated :class:`OccupancyAdapter`.

    Returns:
        Callable ``(q: list[float]) -> float`` returning clearance in meters.
    """

    def clearance_fn(q: List[float]) -> float:
        return occupancy.clearance([q[0], q[1], 0.0])

    return clearance_fn


def _gates_from_config(
    scenario_name: str,
    gate_cfg: Dict[str, Any],
) -> List[QualityGate]:
    """Build :class:`QualityGate` objects from a scenario's gate config.

    Args:
        scenario_name: Human-readable name for error messages.
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
    name: str,
    scenario_cfg: Dict[str, Any],
    joint_limits: List[Tuple[float, float]],
    repeat: int,
) -> Dict[str, float]:
    """Run a single benchmark scenario and return aggregated metrics.

    Args:
        name: Scenario name (for diagnostic messages).
        scenario_cfg: Scenario sub-dict from ``benchmark.yaml``.
        joint_limits: Per-joint ``(lower, upper)`` bounds.
        repeat: Number of repeated runs.

    Returns:
        Dict with keys: ``success_rate``, ``avg_latency_s``,
        ``path_length``, ``smoothness``, ``min_clearance_m``.
    """
    obstacles: List[List[float]] = scenario_cfg.get("obstacles", [])
    inflation_radius: float = float(scenario_cfg.get("inflation_radius", 0.05))
    start: List[float] = [float(v) for v in scenario_cfg["start"]]
    goal: List[float] = [float(v) for v in scenario_cfg["goal"]]
    timeout: float = float(scenario_cfg.get("timeout", 10.0))
    planner_config: Dict[str, Any] = scenario_cfg.get("planner_config", {})

    latencies: List[float] = []
    path_lengths: List[float] = []
    smoothnesses: List[float] = []
    clearances: List[float] = []
    success_count = 0

    for _ in range(repeat):
        occupancy = OccupancyAdapter(inflation_radius=inflation_radius)
        occupancy.update(obstacles)

        validator = _make_validator(occupancy)
        clearance_fn = _make_clearance_fn(occupancy)

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

        result = adapter.plan(request)
        latencies.append(result["solve_time"])

        if result["status"] == "success":
            success_count += 1
            path = result["path"]
            path_lengths.append(path_length(path))
            smoothnesses.append(path_smoothness(path))
            clearances.append(min_obstacle_clearance(path, clearance_fn))

    success_rate = success_count / repeat
    avg_latency = sum(latencies) / len(latencies)

    avg_path_length = (
        sum(path_lengths) / len(path_lengths) if path_lengths else math.nan
    )
    avg_smoothness = (
        sum(smoothnesses) / len(smoothnesses) if smoothnesses else math.nan
    )
    avg_clearance = (
        sum(clearances) / len(clearances) if clearances else math.nan
    )

    return {
        "success_rate": success_rate,
        "avg_latency_s": avg_latency,
        "path_length": avg_path_length,
        "smoothness": avg_smoothness,
        "min_clearance_m": avg_clearance,
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
    joint_limits = _build_joint_limits(cfg)
    scenarios_cfg: Dict[str, Any] = cfg.get("scenarios", {})
    gates_cfg: Dict[str, Any] = cfg.get("quality_gates", {})

    print("=" * 72)
    print("ARCO-FRET Quality Gate Validation  (Issue 09)")
    print(f"Config : {_CONFIG_PATH}")
    print(f"Repeats: {repeat}")
    print("=" * 72)

    reports: List[ScenarioReport] = []

    for scenario_name, scenario_cfg in scenarios_cfg.items():
        print(f"\n[{scenario_name}] Running {repeat} repeat(s) …", flush=True)
        metrics = _run_scenario(
            name=scenario_name,
            scenario_cfg=scenario_cfg,
            joint_limits=joint_limits,
            repeat=repeat,
        )
        gates = _gates_from_config(
            scenario_name=scenario_name,
            gate_cfg=gates_cfg.get(scenario_name, {}),
        )
        results = evaluate_gates(metrics, gates)
        reports.append(ScenarioReport(scenario=scenario_name, results=results))

    print("\n" + format_report(reports))

    all_passed = all(r.passed for r in reports)
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
