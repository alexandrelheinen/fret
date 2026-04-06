"""Validation utilities for ARCO-FRET benchmarks and quality gates.

Provides metric computation functions and the quality-gate evaluation
framework used by the automated validation script
``scripts/validate_quality_gates.py``.

**Exported symbols:**

- :func:`~fret.validation.metrics.path_length`
- :func:`~fret.validation.metrics.path_smoothness`
- :func:`~fret.validation.metrics.min_obstacle_clearance`
- :func:`~fret.validation.metrics.tracking_rmse`
- :class:`~fret.validation.quality_gates.QualityGate`
- :class:`~fret.validation.quality_gates.GateResult`
- :class:`~fret.validation.quality_gates.ScenarioReport`
- :func:`~fret.validation.quality_gates.evaluate_gates`
- :func:`~fret.validation.quality_gates.format_report`

See also:
    ``docs/arco/issue-09-validation-benchmarks-and-quality-gates.md``
"""

from fret.validation.metrics import (
    min_obstacle_clearance,
    path_length,
    path_smoothness,
    tracking_rmse,
)
from fret.validation.quality_gates import (
    GateResult,
    QualityGate,
    ScenarioReport,
    evaluate_gates,
    format_report,
)

__all__ = [
    "path_length",
    "path_smoothness",
    "min_obstacle_clearance",
    "tracking_rmse",
    "QualityGate",
    "GateResult",
    "ScenarioReport",
    "evaluate_gates",
    "format_report",
]
