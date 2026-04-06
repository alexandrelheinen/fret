"""Quality-gate definitions, evaluator, and report formatter.

Implements the pass/fail threshold framework defined in
``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md``.

A *quality gate* is a named metric with a numeric threshold and a
comparison operator (``">="`` or ``"<="``).  A gate **passes** when the
observed metric value satisfies the comparison.

**Usage example**::

    from fret.validation.quality_gates import QualityGate, evaluate_gates

    gates = [
        QualityGate("success_rate", threshold=1.0, operator=">=",
                    units="fraction"),
        QualityGate("avg_latency_s", threshold=0.2, operator="<=",
                    units="s"),
    ]
    metrics = {"success_rate": 1.0, "avg_latency_s": 0.045}
    results = evaluate_gates(metrics, gates)
    print(format_report([ScenarioReport("easy", results)]))

See also:
    ``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md`` —
    full specification and threshold rationale.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class QualityGate:
    """A single quality gate: metric name, threshold, and comparison.

    Attributes:
        name: Metric key as returned by the benchmark runner.
        threshold: Numeric threshold value.
        operator: Comparison operator; either ``">="`` or ``"<="``.
        units: Human-readable unit string (e.g. ``"s"``, ``"fraction"``).
        description: Optional human-readable description of the gate.
    """

    name: str
    threshold: float
    operator: str
    units: str = ""
    description: str = ""

    def evaluate(self, value: float) -> bool:
        """Return ``True`` when *value* passes the gate.

        Args:
            value: Observed metric value.

        Returns:
            ``True`` if the value satisfies the threshold, ``False``
            otherwise.

        Raises:
            ValueError: If :attr:`operator` is not ``">="`` or ``"<="``.
        """
        if self.operator == ">=":
            return value >= self.threshold
        if self.operator == "<=":
            return value <= self.threshold
        raise ValueError(
            f"Unknown operator '{self.operator}'; expected '>=' or '<='"
        )


@dataclass
class GateResult:
    """Outcome of evaluating one quality gate against an observed value.

    Attributes:
        gate: The quality gate that was evaluated.
        value: The observed metric value.
        passed: ``True`` when the value satisfies the gate threshold.
    """

    gate: QualityGate
    value: float
    passed: bool


@dataclass
class ScenarioReport:
    """Quality gate results for a single benchmark scenario.

    Attributes:
        scenario: Scenario identifier (e.g. ``"easy"``, ``"hard"``).
        results: Ordered list of :class:`GateResult` objects.
    """

    scenario: str
    results: List[GateResult] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        """``True`` when every gate in the scenario passed."""
        return bool(self.results) and all(r.passed for r in self.results)

    @property
    def pass_count(self) -> int:
        """Number of gates that passed."""
        return sum(1 for r in self.results if r.passed)

    @property
    def fail_count(self) -> int:
        """Number of gates that failed."""
        return sum(1 for r in self.results if not r.passed)


# ---------------------------------------------------------------------------
# Gate evaluation
# ---------------------------------------------------------------------------


def evaluate_gates(
    metrics: Dict[str, float],
    gates: List[QualityGate],
) -> List[GateResult]:
    """Evaluate a list of quality gates against a metrics dictionary.

    Missing metrics are recorded as ``math.nan`` with ``passed=False``.

    Args:
        metrics: Mapping of metric name to observed float value.
        gates: Ordered list of :class:`QualityGate` objects to evaluate.

    Returns:
        List of :class:`GateResult` objects in the same order as ``gates``.
    """
    results: List[GateResult] = []
    for gate in gates:
        if gate.name in metrics:
            value = metrics[gate.name]
            passed = gate.evaluate(value)
        else:
            value = math.nan
            passed = False
        results.append(GateResult(gate=gate, value=value, passed=passed))
    return results


# ---------------------------------------------------------------------------
# Report formatter
# ---------------------------------------------------------------------------

_PASS_LABEL = "PASS"
_FAIL_LABEL = "FAIL"
_NA_LABEL = "N/A"


def format_report(
    reports: List[ScenarioReport],
    title: Optional[str] = None,
) -> str:
    """Format scenario reports as a human-readable pass/fail table.

    Args:
        reports: List of :class:`ScenarioReport` objects to include.
        title: Optional report title.  Defaults to
            ``"ARCO-FRET Quality Gate Report"``.

    Returns:
        Multi-line string suitable for printing to stdout or writing to a
        file.
    """
    if title is None:
        title = "ARCO-FRET Quality Gate Report"

    lines: List[str] = []
    sep = "=" * 72

    lines.append(sep)
    lines.append(title)
    lines.append(sep)

    for report in reports:
        overall = _PASS_LABEL if report.passed else _FAIL_LABEL
        lines.append(
            f"\nScenario: {report.scenario!r}  "
            f"[{overall}]  "
            f"({report.pass_count}/{len(report.results)} gates passed)"
        )
        lines.append("-" * 72)
        lines.append(
            f"  {'Gate':<28} {'Observed':>12} {'Threshold':>12} "
            f"{'Op':>4}  {'Result':>6}"
        )
        lines.append("  " + "-" * 68)

        for r in report.results:
            gate = r.gate
            if math.isnan(r.value):
                observed_str = _NA_LABEL
                result_str = _FAIL_LABEL
            else:
                observed_str = f"{r.value:.4g} {gate.units}".strip()
                result_str = _PASS_LABEL if r.passed else _FAIL_LABEL

            threshold_str = f"{gate.threshold:.4g} {gate.units}".strip()
            lines.append(
                f"  {gate.name:<28} {observed_str:>12} "
                f"{threshold_str:>12} {gate.operator:>4}  "
                f"{result_str:>6}"
            )

    lines.append("")
    lines.append(sep)

    total_pass = sum(r.pass_count for r in reports)
    total_gates = sum(len(r.results) for r in reports)
    all_passed = all(r.passed for r in reports)
    overall_label = _PASS_LABEL if all_passed else _FAIL_LABEL
    lines.append(
        f"Overall: [{overall_label}]  "
        f"{total_pass}/{total_gates} gates passed across "
        f"{len(reports)} scenario(s)"
    )
    lines.append(sep)

    return "\n".join(lines)
