"""Unit tests for fret.validation.quality_gates."""

from __future__ import annotations

import math

import pytest

from fret.validation.quality_gates import (
    GateResult,
    QualityGate,
    ScenarioReport,
    evaluate_gates,
    format_report,
)

# ---------------------------------------------------------------------------
# QualityGate construction
# ---------------------------------------------------------------------------


def test_gate_construction() -> None:
    gate = QualityGate(
        name="success_rate",
        threshold=0.9,
        operator=">=",
        units="fraction",
        description="Fraction of successful runs",
    )
    assert gate.name == "success_rate"
    assert gate.threshold == pytest.approx(0.9)
    assert gate.operator == ">="
    assert gate.units == "fraction"
    assert gate.description == "Fraction of successful runs"


def test_gate_defaults() -> None:
    gate = QualityGate(name="latency", threshold=5.0, operator="<=")
    assert gate.units == ""
    assert gate.description == ""


# ---------------------------------------------------------------------------
# evaluate_gates
# ---------------------------------------------------------------------------


def test_evaluate_pass_gte() -> None:
    gate = QualityGate(name="success_rate", threshold=0.90, operator=">=")
    results = evaluate_gates({"success_rate": 0.95}, [gate])
    assert len(results) == 1
    assert results[0].passed is True
    assert results[0].value == pytest.approx(0.95)


def test_evaluate_fail_gte() -> None:
    gate = QualityGate(name="success_rate", threshold=0.90, operator=">=")
    results = evaluate_gates({"success_rate": 0.85}, [gate])
    assert results[0].passed is False


def test_evaluate_pass_lte() -> None:
    gate = QualityGate(name="latency", threshold=10.0, operator="<=")
    results = evaluate_gates({"latency": 5.0}, [gate])
    assert results[0].passed is True


def test_evaluate_fail_lte() -> None:
    gate = QualityGate(name="latency", threshold=10.0, operator="<=")
    results = evaluate_gates({"latency": 15.0}, [gate])
    assert results[0].passed is False


def test_evaluate_missing_metric() -> None:
    gate = QualityGate(name="not_present", threshold=1.0, operator=">=")
    results = evaluate_gates({}, [gate])
    assert results[0].passed is False
    assert math.isnan(results[0].value)


def test_evaluate_multiple_gates() -> None:
    gates = [
        QualityGate(name="success_rate", threshold=0.9, operator=">="),
        QualityGate(name="latency", threshold=10.0, operator="<="),
    ]
    metrics = {"success_rate": 0.95, "latency": 15.0}
    results = evaluate_gates(metrics, gates)
    assert results[0].passed is True
    assert results[1].passed is False


def test_evaluate_empty_gates() -> None:
    results = evaluate_gates({"success_rate": 1.0}, [])
    assert results == []


def test_evaluate_exact_threshold_gte() -> None:
    gate = QualityGate(name="x", threshold=0.9, operator=">=")
    results = evaluate_gates({"x": 0.9}, [gate])
    assert results[0].passed is True


# ---------------------------------------------------------------------------
# GateResult
# ---------------------------------------------------------------------------


def test_gate_result_passed() -> None:
    gate = QualityGate(name="m", threshold=1.0, operator=">=")
    r = GateResult(gate=gate, value=2.0, passed=True)
    assert r.passed is True
    assert r.value == pytest.approx(2.0)
    assert r.gate is gate


def test_gate_result_failed() -> None:
    gate = QualityGate(name="m", threshold=1.0, operator=">=")
    r = GateResult(gate=gate, value=0.5, passed=False)
    assert r.passed is False


# ---------------------------------------------------------------------------
# ScenarioReport
# ---------------------------------------------------------------------------


def _make_gate(name: str = "m", threshold: float = 1.0) -> QualityGate:
    return QualityGate(name=name, threshold=threshold, operator=">=")


def test_scenario_report_all_passed() -> None:
    gate = _make_gate()
    results = [GateResult(gate=gate, value=2.0, passed=True)]
    report = ScenarioReport(scenario="s1", results=results)
    assert report.passed is True


def test_scenario_report_one_failed() -> None:
    gate = _make_gate()
    results = [
        GateResult(gate=gate, value=2.0, passed=True),
        GateResult(gate=gate, value=0.5, passed=False),
    ]
    report = ScenarioReport(scenario="s1", results=results)
    assert report.passed is False


def test_scenario_report_empty() -> None:
    report = ScenarioReport(scenario="empty", results=[])
    assert report.passed is True


# ---------------------------------------------------------------------------
# format_report
# ---------------------------------------------------------------------------


def _make_report(scenario: str, passed_flags: list[bool]) -> ScenarioReport:
    gate = QualityGate(name="metric", threshold=1.0, operator=">=", units="u")
    results = [
        GateResult(gate=gate, value=2.0 if p else 0.5, passed=p)
        for p in passed_flags
    ]
    return ScenarioReport(scenario=scenario, results=results)


def test_format_report_contains_scenario_name() -> None:
    report = _make_report("my_scenario", [True])
    text = format_report([report])
    assert "my_scenario" in text


def test_format_report_shows_pass() -> None:
    report = _make_report("s1", [True])
    text = format_report([report])
    assert "PASS" in text


def test_format_report_shows_fail() -> None:
    report = _make_report("s1", [False])
    text = format_report([report])
    assert "FAIL" in text


def test_format_report_multiple_scenarios() -> None:
    reports = [_make_report("s1", [True]), _make_report("s2", [False])]
    text = format_report(reports)
    assert "s1" in text
    assert "s2" in text


def test_format_report_empty() -> None:
    text = format_report([])
    assert isinstance(text, str)
    assert len(text) > 0


def test_format_overall_passed() -> None:
    reports = [_make_report("s1", [True]), _make_report("s2", [True])]
    text = format_report(reports)
    assert "ALL GATES PASSED" in text


def test_format_overall_failed() -> None:
    reports = [_make_report("s1", [True]), _make_report("s2", [False])]
    text = format_report(reports)
    assert "SOME GATES FAILED" in text
