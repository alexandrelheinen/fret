"""``fret-diag load`` — everything-on-Pi performance budget (FR-HW-73)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from fret.diag.core import CheckResult, CheckStatus
from fret.resources import resolve


def load_budget(path: Path | None = None) -> dict[str, Any]:
    if path is None:
        path = resolve("config", "hardware/budget.yml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"budget file must be a mapping: {path}")
    return data


def evaluate_budget(
    metrics: dict[str, float],
    budget: dict[str, Any] | None = None,
) -> list[CheckResult]:
    """Compare synthetic or measured metrics to the Performance budget table."""
    budget = budget or load_budget()
    results: list[CheckResult] = []

    def _check(
        name: str,
        value: float,
        limit: float,
        *,
        cmp: str = "le",
    ) -> None:
        ok = value <= limit if cmp == "le" else value < limit
        results.append(
            CheckResult(
                name,
                CheckStatus.PASS if ok else CheckStatus.FAIL,
                f"{name}={value} limit={limit}",
                {"value": value, "limit": limit},
            )
        )

    _check(
        "bridge_cycle_p99_ms",
        float(metrics["bridge_cycle_p99_ms"]),
        float(budget["bridge_cycle_p99_ms"]),
    )
    _check(
        "bridge_cycle_max_ms",
        float(metrics["bridge_cycle_max_ms"]),
        float(budget["bridge_cycle_max_ms"]),
    )
    missed = float(metrics.get("missed_cycles", 0))
    results.append(
        CheckResult(
            "missed_cycles",
            CheckStatus.PASS if missed <= 0 else CheckStatus.FAIL,
            f"missed_cycles={int(missed)}",
            {"value": missed},
        )
    )
    _check(
        "vision_p95_ms",
        float(metrics["vision_p95_ms"]),
        float(budget["vision_p95_ms"]),
    )
    _check(
        "cpu_percent",
        float(metrics["cpu_percent"]),
        float(budget["cpu_percent"]),
    )
    _check(
        "memory_gb",
        float(metrics["memory_gb"]),
        float(budget["memory_gb"]),
    )
    _check(
        "soc_temp_c",
        float(metrics["soc_temp_c"]),
        float(budget["soc_temp_c"]),
    )
    throttled = bool(metrics.get("throttled", False))
    undervoltage = bool(metrics.get("undervoltage", False))
    results.append(
        CheckResult(
            "throttled",
            CheckStatus.PASS if not throttled else CheckStatus.FAIL,
            f"throttled={throttled}",
        )
    )
    results.append(
        CheckResult(
            "undervoltage",
            CheckStatus.PASS if not undervoltage else CheckStatus.FAIL,
            f"undervoltage={undervoltage}",
        )
    )
    return results


def run_load_checks(
    *,
    duration_s: float = 1.0,
    metrics: dict[str, float] | None = None,
) -> list[CheckResult]:
    """Run budget evaluation.

    Without hardware, uses a synthetic passing profile unless ``metrics``
    is supplied. QEMU numbers are not representative of a real Pi 5.
    """
    _ = duration_s  # reserved for a future live stack harness
    if metrics is None:
        metrics = {
            "bridge_cycle_p99_ms": 5.0,
            "bridge_cycle_max_ms": 8.0,
            "missed_cycles": 0.0,
            "vision_p95_ms": 40.0,
            "cpu_percent": 55.0,
            "memory_gb": 2.5,
            "soc_temp_c": 62.0,
            "throttled": 0.0,
            "undervoltage": 0.0,
        }
    return evaluate_budget(metrics)
