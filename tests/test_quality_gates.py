"""Unit tests for fret.validation.quality_gates (Issue 09).

Validates:
- QualityGate.evaluate: ">=" and "<=" operators, threshold equality,
  unknown operator raises.
- evaluate_gates: all pass, all fail, missing metric, mixed results.
- ScenarioReport.passed, pass_count, fail_count properties.
- format_report: non-empty string, contains scenario name and pass/fail
  labels, overall summary, NaN handling for missing metrics.

All tests are deterministic and run in-process; no ROS runtime is required.
"""

import math
import os
import sys
import unittest

# ---------------------------------------------------------------------------
# Make the fret package importable without a full ROS build.
# ---------------------------------------------------------------------------
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(os.path.dirname(_TESTS_DIR), "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from fret.validation.quality_gates import (
    GateResult,
    QualityGate,
    ScenarioReport,
    evaluate_gates,
    format_report,
)

# ---------------------------------------------------------------------------
# QualityGate.evaluate
# ---------------------------------------------------------------------------


class TestQualityGateEvaluate(unittest.TestCase):
    """Tests for QualityGate.evaluate()."""

    def test_gte_passes_above_threshold(self) -> None:
        """'>=' passes when value > threshold."""
        gate = QualityGate("x", threshold=0.5, operator=">=")
        self.assertTrue(gate.evaluate(0.6))

    def test_gte_passes_at_threshold(self) -> None:
        """'>=' passes when value == threshold (inclusive)."""
        gate = QualityGate("x", threshold=0.5, operator=">=")
        self.assertTrue(gate.evaluate(0.5))

    def test_gte_fails_below_threshold(self) -> None:
        """'>=' fails when value < threshold."""
        gate = QualityGate("x", threshold=0.5, operator=">=")
        self.assertFalse(gate.evaluate(0.4))

    def test_lte_passes_below_threshold(self) -> None:
        """'<=' passes when value < threshold."""
        gate = QualityGate("y", threshold=1.0, operator="<=")
        self.assertTrue(gate.evaluate(0.9))

    def test_lte_passes_at_threshold(self) -> None:
        """'<=' passes when value == threshold (inclusive)."""
        gate = QualityGate("y", threshold=1.0, operator="<=")
        self.assertTrue(gate.evaluate(1.0))

    def test_lte_fails_above_threshold(self) -> None:
        """'<=' fails when value > threshold."""
        gate = QualityGate("y", threshold=1.0, operator="<=")
        self.assertFalse(gate.evaluate(1.1))

    def test_unknown_operator_raises(self) -> None:
        """An unrecognized operator must raise ValueError."""
        gate = QualityGate("z", threshold=0.0, operator="!=")
        with self.assertRaises(ValueError):
            gate.evaluate(0.0)

    def test_zero_threshold_gte(self) -> None:
        """Zero threshold with '>=' passes for non-negative values."""
        gate = QualityGate("clearance", threshold=0.0, operator=">=")
        self.assertTrue(gate.evaluate(0.0))
        self.assertFalse(gate.evaluate(-0.001))

    def test_units_and_description_stored(self) -> None:
        """units and description attributes are stored on the gate."""
        gate = QualityGate(
            "rate",
            threshold=1.0,
            operator=">=",
            units="fraction",
            description="Must always succeed",
        )
        self.assertEqual(gate.units, "fraction")
        self.assertEqual(gate.description, "Must always succeed")


# ---------------------------------------------------------------------------
# evaluate_gates
# ---------------------------------------------------------------------------


class TestEvaluateGates(unittest.TestCase):
    """Tests for evaluate_gates()."""

    def _make_gates(self) -> list:
        return [
            QualityGate("success_rate", threshold=1.0, operator=">="),
            QualityGate("avg_latency_s", threshold=0.2, operator="<="),
            QualityGate("min_clearance_m", threshold=0.0, operator=">="),
        ]

    def test_all_pass(self) -> None:
        """All gates pass when all metrics satisfy thresholds."""
        gates = self._make_gates()
        metrics = {
            "success_rate": 1.0,
            "avg_latency_s": 0.1,
            "min_clearance_m": 0.05,
        }
        results = evaluate_gates(metrics, gates)
        self.assertEqual(len(results), 3)
        self.assertTrue(all(r.passed for r in results))

    def test_all_fail(self) -> None:
        """All gates fail when no metric satisfies the threshold."""
        gates = self._make_gates()
        metrics = {
            "success_rate": 0.5,
            "avg_latency_s": 1.0,
            "min_clearance_m": -0.1,
        }
        results = evaluate_gates(metrics, gates)
        self.assertTrue(all(not r.passed for r in results))

    def test_missing_metric_fails(self) -> None:
        """A gate with a missing metric key is recorded as failed."""
        gates = [QualityGate("missing_key", threshold=0.5, operator=">=")]
        results = evaluate_gates({}, gates)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0].passed)
        self.assertTrue(math.isnan(results[0].value))

    def test_mixed_results(self) -> None:
        """First gate passes, second fails."""
        gates = [
            QualityGate("a", threshold=1.0, operator=">="),
            QualityGate("b", threshold=0.5, operator="<="),
        ]
        metrics = {"a": 2.0, "b": 0.9}
        results = evaluate_gates(metrics, gates)
        self.assertTrue(results[0].passed)
        self.assertFalse(results[1].passed)

    def test_result_values_stored(self) -> None:
        """Observed values are preserved on GateResult objects."""
        gates = [QualityGate("x", threshold=0.3, operator=">=")]
        metrics = {"x": 0.75}
        results = evaluate_gates(metrics, gates)
        self.assertAlmostEqual(results[0].value, 0.75)

    def test_empty_gates(self) -> None:
        """Empty gate list returns empty result list."""
        results = evaluate_gates({"x": 1.0}, [])
        self.assertEqual(results, [])

    def test_order_preserved(self) -> None:
        """Results are returned in the same order as the input gates."""
        names = ["c", "a", "b"]
        gates = [QualityGate(n, threshold=0.0, operator=">=") for n in names]
        metrics = {"a": 1.0, "b": 1.0, "c": 1.0}
        results = evaluate_gates(metrics, gates)
        self.assertEqual([r.gate.name for r in results], names)


# ---------------------------------------------------------------------------
# ScenarioReport
# ---------------------------------------------------------------------------


class TestScenarioReport(unittest.TestCase):
    """Tests for ScenarioReport properties."""

    def _make_report(self, passed_flags: list) -> ScenarioReport:
        gate = QualityGate("x", threshold=0.0, operator=">=")
        results = [
            GateResult(gate=gate, value=1.0 if p else -1.0, passed=p)
            for p in passed_flags
        ]
        return ScenarioReport(scenario="test", results=results)

    def test_all_pass_scenario(self) -> None:
        """passed is True when every gate result is True."""
        report = self._make_report([True, True, True])
        self.assertTrue(report.passed)

    def test_any_fail_scenario(self) -> None:
        """passed is False when any gate result is False."""
        report = self._make_report([True, False, True])
        self.assertFalse(report.passed)

    def test_empty_results_not_passed(self) -> None:
        """An empty results list is not considered passed."""
        report = ScenarioReport(scenario="empty", results=[])
        self.assertFalse(report.passed)

    def test_pass_count(self) -> None:
        """pass_count counts the number of passing gates."""
        report = self._make_report([True, True, False])
        self.assertEqual(report.pass_count, 2)

    def test_fail_count(self) -> None:
        """fail_count counts the number of failing gates."""
        report = self._make_report([True, False, False])
        self.assertEqual(report.fail_count, 2)

    def test_scenario_name_stored(self) -> None:
        """Scenario name is stored on the report."""
        report = ScenarioReport(scenario="easy", results=[])
        self.assertEqual(report.scenario, "easy")


# ---------------------------------------------------------------------------
# format_report
# ---------------------------------------------------------------------------


class TestFormatReport(unittest.TestCase):
    """Tests for format_report()."""

    def _make_passing_report(self, name: str = "easy") -> ScenarioReport:
        gate = QualityGate(
            "success_rate",
            threshold=1.0,
            operator=">=",
            units="fraction",
        )
        result = GateResult(gate=gate, value=1.0, passed=True)
        return ScenarioReport(scenario=name, results=[result])

    def _make_failing_report(self, name: str = "hard") -> ScenarioReport:
        gate = QualityGate(
            "avg_latency_s",
            threshold=0.1,
            operator="<=",
            units="s",
        )
        result = GateResult(gate=gate, value=5.0, passed=False)
        return ScenarioReport(scenario=name, results=[result])

    def test_returns_non_empty_string(self) -> None:
        """format_report returns a non-empty string."""
        report = self._make_passing_report()
        output = format_report([report])
        self.assertIsInstance(output, str)
        self.assertTrue(len(output) > 0)

    def test_contains_scenario_name(self) -> None:
        """Output contains the scenario name."""
        report = self._make_passing_report("my_scenario")
        output = format_report([report])
        self.assertIn("my_scenario", output)

    def test_pass_label_present_when_all_pass(self) -> None:
        """'PASS' label appears when all gates pass."""
        report = self._make_passing_report()
        output = format_report([report])
        self.assertIn("PASS", output)

    def test_fail_label_present_when_gate_fails(self) -> None:
        """'FAIL' label appears when a gate fails."""
        report = self._make_failing_report()
        output = format_report([report])
        self.assertIn("FAIL", output)

    def test_overall_pass_in_summary(self) -> None:
        """Overall summary shows PASS when all scenarios pass."""
        report = self._make_passing_report()
        output = format_report([report])
        # Overall line must contain PASS
        lines = output.splitlines()
        overall_lines = [l for l in lines if "Overall" in l]
        self.assertTrue(len(overall_lines) > 0)
        self.assertTrue(any("PASS" in l for l in overall_lines))

    def test_overall_fail_in_summary(self) -> None:
        """Overall summary shows FAIL when any scenario fails."""
        report = self._make_failing_report()
        output = format_report([report])
        lines = output.splitlines()
        overall_lines = [l for l in lines if "Overall" in l]
        self.assertTrue(any("FAIL" in l for l in overall_lines))

    def test_multiple_scenarios(self) -> None:
        """Multiple scenarios all appear in the output."""
        reports = [
            self._make_passing_report("easy"),
            self._make_failing_report("hard"),
        ]
        output = format_report(reports)
        self.assertIn("easy", output)
        self.assertIn("hard", output)

    def test_nan_value_handled(self) -> None:
        """NaN metric values (missing metric) appear as N/A."""
        gate = QualityGate("missing", threshold=0.5, operator=">=")
        result = GateResult(gate=gate, value=math.nan, passed=False)
        report = ScenarioReport(scenario="s", results=[result])
        output = format_report([report])
        self.assertIn("N/A", output)

    def test_custom_title(self) -> None:
        """A custom title appears in the output."""
        report = self._make_passing_report()
        output = format_report([report], title="My Custom Title")
        self.assertIn("My Custom Title", output)

    def test_default_title_present(self) -> None:
        """Default title is included when no custom title is given."""
        report = self._make_passing_report()
        output = format_report([report])
        self.assertIn("ARCO-FRET Quality Gate Report", output)

    def test_metric_units_in_output(self) -> None:
        """Gate units appear in the threshold column."""
        gate = QualityGate("x", threshold=0.5, operator=">=", units="rad")
        result = GateResult(gate=gate, value=1.0, passed=True)
        report = ScenarioReport(scenario="s", results=[result])
        output = format_report([report])
        self.assertIn("rad", output)

    def test_gate_count_in_summary(self) -> None:
        """Overall summary includes gate counts."""
        reports = [
            self._make_passing_report("easy"),
            self._make_failing_report("hard"),
        ]
        output = format_report(reports)
        # Should mention '2' scenarios
        self.assertIn("2", output)


if __name__ == "__main__":
    unittest.main()
