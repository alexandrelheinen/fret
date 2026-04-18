"""Quality gate data structures and evaluation logic for FRET.

Provides :class:`QualityGate`, :class:`GateResult`, :class:`ScenarioReport`,
and helper functions :func:`evaluate_gates` and :func:`format_report` that
together implement a lightweight pass/fail gating system for benchmark metrics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable


@dataclass
class QualityGate:
    """A single pass/fail threshold for a named metric.

    Attributes:
        name: Metric name (must match a key in the metrics dict).
        threshold: Comparison threshold value.
        operator: Comparison operator string: ``">="``, ``"<="``, ``">"``,
            ``"<"``, ``"=="``.
        units: Human-readable unit string (e.g. ``"fraction"``, ``"s"``).
        description: Human-readable description of what is being gated.
    """

    name: str
    threshold: float
    operator: str
    units: str = ""
    description: str = ""


@dataclass
class GateResult:
    """Result of evaluating one quality gate.

    Attributes:
        gate: The gate that was evaluated.
        value: The measured metric value.
        passed: Whether the gate criterion was met.
    """

    gate: QualityGate
    value: float
    passed: bool


@dataclass
class ScenarioReport:
    """Aggregated gate results for one scenario.

    Attributes:
        scenario: Scenario name.
        results: List of individual gate results.
    """

    scenario: str
    results: list[GateResult]

    @property
    def passed(self) -> bool:
        """True if all gates in this scenario passed."""
        return all(r.passed for r in self.results)


_OPS: dict[str, Callable[[float, float], bool]] = {
    ">=": lambda v, t: v >= t,
    "<=": lambda v, t: v <= t,
    ">": lambda v, t: v > t,
    "<": lambda v, t: v < t,
    "==": lambda v, t: abs(v - t) < 1e-9,
}


def evaluate_gates(
    metrics: dict[str, float],
    gates: list[QualityGate],
) -> list[GateResult]:
    """Evaluate a list of quality gates against measured metrics.

    Args:
        metrics: Dict mapping metric names to measured float values.
        gates: List of quality gates to evaluate.

    Returns:
        List of :class:`GateResult` objects in the same order as *gates*.
        Gates for missing metrics have ``value=nan`` and ``passed=False``.
    """
    results: list[GateResult] = []
    for gate in gates:
        val = metrics.get(gate.name, math.nan)
        op_fn: Callable[[float, float], bool] = _OPS.get(
            gate.operator, lambda v, t: False
        )
        passed = not math.isnan(val) and op_fn(val, gate.threshold)
        results.append(GateResult(gate=gate, value=val, passed=passed))
    return results


def format_report(reports: list[ScenarioReport]) -> str:
    """Format a list of :class:`ScenarioReport` objects as a human-readable table.

    Args:
        reports: Scenario reports to format.

    Returns:
        A multi-line string with a pass/fail table.
    """
    lines: list[str] = ["=" * 72, "Quality Gate Report", "=" * 72]
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
