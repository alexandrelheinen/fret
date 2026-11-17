"""FR-HW-73: budget evaluation pass and fail paths."""

from __future__ import annotations

from fret.diag.core import CheckStatus
from fret.diag.load import evaluate_budget, load_budget, run_load_checks


def test_budget_file_loads() -> None:
    budget = load_budget()
    assert budget["bridge_cycle_p99_ms"] == 10.0


def test_load_pass_synthetic() -> None:
    results = run_load_checks()
    assert all(r.status is CheckStatus.PASS for r in results)


def test_each_budget_fail_path() -> None:
    base = {
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
    cases = {
        "bridge_cycle_p99_ms": 50.0,
        "missed_cycles": 3.0,
        "vision_p95_ms": 200.0,
        "cpu_percent": 95.0,
        "memory_gb": 5.0,
        "soc_temp_c": 85.0,
        "throttled": 1.0,
        "undervoltage": 1.0,
    }
    for key, bad in cases.items():
        metrics = dict(base)
        metrics[key] = bad
        results = evaluate_budget(metrics)
        assert any(r.status is CheckStatus.FAIL for r in results), key
