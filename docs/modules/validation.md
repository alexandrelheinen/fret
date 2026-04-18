# Validation Module

**Package:** `fret.validation`  
**Source:** `src/fret/validation/`  
**Tests:** `tests/test_metrics.py`, `tests/test_quality_gates.py`

---

## Responsibility

The validation module provides quantitative metrics and quality gates for evaluating
the performance of FRET simulation runs. It is a pure-Python, ROS-independent layer
used both in CI scripts and in post-processing analysis.

---

## Components

### `metrics.py` — Trajectory Metrics

| Function | Description |
|---|---|
| `path_length(path)` | Total Euclidean length of a joint-space path |
| `path_smoothness(path)` | Mean squared joint acceleration (lower = smoother) |
| `min_obstacle_clearance(path, occupancy)` | Minimum clearance to any obstacle across all waypoints |
| `tracking_rmse(reference, executed)` | RMS end-effector tracking error over a trajectory |

---

### `quality_gates.py` — Quality Gates

Defines pass/fail gates for simulation scenarios. Each gate evaluates a
`ScenarioReport` against a threshold and returns a `GateResult`.

| Class / Function | Description |
|---|---|
| `QualityGate` | Named gate with threshold and metric function |
| `GateResult` | Pass/fail result with actual vs. threshold values |
| `ScenarioReport` | Structured report from a completed simulation run |
| `evaluate_gates(report, gates)` | Apply all gates to a report |
| `format_report(results)` | Human-readable summary table |

---

## Configuration

Quality gate thresholds are in `src/fret/config/benchmark.yaml`:

```yaml
max_ee_error_mm: 5.0
min_clearance_m: 0.02
max_planning_s: 30.0
min_command_hz: 45.0
```

---

## CI Integration

The validation module is used by all CI simulation scripts:

```bash
python3 scripts/simulate_milestone3_pipeline.py --output /tmp/sim_ms3
# Writes results.env with MAX_EE_ERROR_MM, RMS_EE_ERROR_MM, FAULT_TRIGGERED, ...
```

The `validate_quality_gates.py` script runs all gates against a completed run's
`results.env` and exits non-zero if any gate fails.

---

## Tests

- `tests/test_metrics.py` — unit tests for all metric functions (path length,
  smoothness, clearance, RMSE)
- `tests/test_quality_gates.py` — unit tests for gate evaluation and report
  formatting
- **Total: 71 tests**

---

## Satisfies Requirements

| Requirement | Description |
|---|---|
| FR-CTL-02 | EE error ≤ 5 mm validated by `tracking_rmse` gate |
| FR-PLN-04 | Planning timeout ≤ 30 s validated by `max_planning_s` gate |
| FR-CTL-01 | Command rate ≥ 45 Hz validated by `min_command_hz` gate |
