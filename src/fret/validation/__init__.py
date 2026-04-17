"""Validation and quality gate utilities for FRET.

Exports metric computation functions and quality gate evaluation tools.
"""

from __future__ import annotations

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
