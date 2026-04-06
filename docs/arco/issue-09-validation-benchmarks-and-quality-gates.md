# Issue 09: Validation Benchmarks and Quality Gates

## Goal

Define and automate milestone quality gates for planning correctness,
execution quality, and integration robustness.

## Problem Statement

A demo alone is not sufficient; we need objective acceptance metrics
aligned with spec-driven development.

## Functional Requirements

1. Define benchmark scenarios (easy, medium, hard obstacle layouts).
2. Define metrics:
   - planning success rate,
   - planning latency,
   - path length/smoothness,
   - minimum obstacle clearance,
   - tracking RMSE,
   - end-to-end completion time.
3. Define pass/fail thresholds per metric.
4. Add automated validation scripts and CI hooks where practical.

## Non-Functional Requirements

- Benchmarks must be reproducible from versioned configs.
- Metric extraction must be scriptable and deterministic.

## V-Cycle Tasks

### Descending Branch

#### Benchmark Protocol

- Each scenario is run `repeat` times (default 3) with a fixed
  `rng_seed` (default 42) for full reproducibility.
- The same seed is applied to every repeat of the same scenario.
- Scenario definitions (obstacle positions, inflation radius, start/goal
  configurations, timeout, and planner parameters) are versioned in
  `src/fret/config/benchmark.yaml`.

#### Metric Formulas and Sampling Windows

| Metric | Formula | Sampling window |
|--------|---------|-----------------|
| `success_rate` | `#successful / repeat` | all runs |
| `avg_latency_s` | mean `solve_time` across all runs | all runs |
| `path_length` | Σ ‖wₖ − wₖ₋₁‖₂ over consecutive waypoints | successful runs |
| `smoothness` | Σ arccos(ABˆ · BCˆ) at each waypoint triple | successful runs |
| `min_clearance_m` | min clearance(wₖ, occupancy) over all waypoints | successful runs |
| `tracking_rmse` | √(Σ(aₖⱼ − dₖⱼ)² / (N·D)) | future integration |

All metrics are averaged over successful runs (latency is averaged over
all runs, including failures).

#### Quality-Gate Thresholds

Thresholds are defined in `src/fret/config/benchmark.yaml` under
`benchmark.quality_gates` and reproduced below.

| Scenario | Gate | Threshold | Op |
|----------|------|-----------|----|
| easy | success_rate | 1.0 fraction | >= |
| easy | avg_latency_s | 0.2 s | <= |
| easy | path_length | 3.0 rad_m | <= |
| easy | smoothness | 5.0 rad | <= |
| easy | min_clearance_m | 0.0 m | >= |
| medium | success_rate | 1.0 fraction | >= |
| medium | avg_latency_s | 1.0 s | <= |
| medium | path_length | 5.0 rad_m | <= |
| medium | smoothness | 10.0 rad | <= |
| medium | min_clearance_m | 0.0 m | >= |
| hard | success_rate | 0.66 fraction | >= |
| hard | avg_latency_s | 15.0 s | <= |
| hard | path_length | 8.0 rad_m | <= |
| hard | smoothness | 20.0 rad | <= |
| hard | min_clearance_m | 0.0 m | >= |

### Ascending Branch

#### Unit Tests for Metric Computation

- `tests/test_metrics.py` — 38 tests covering `path_length`,
  `path_smoothness`, `min_obstacle_clearance`, `tracking_rmse`, and
  package import hygiene.
- `tests/test_quality_gates.py` — 33 tests covering `QualityGate`,
  `evaluate_gates`, `ScenarioReport`, and `format_report`.

#### Integration Runs

Run the full benchmark suite:

```bash
python3 scripts/validate_quality_gates.py
```

Exits with code 0 if all gates pass, 1 if any gate fails.  The CI
workflow (`.github/workflows/tests.yml`) runs this step automatically on
every pull request.

#### Summary Report

See `docs/arco/baseline-report.md` for the report template and the
filled baseline run.  Baseline result: **15/15 gates passed** across 3
scenarios.

## Deliverables

| Deliverable | Path |
|-------------|------|
| Benchmark configuration | `src/fret/config/benchmark.yaml` |
| Metric computation module | `src/fret/validation/metrics.py` |
| Quality-gate framework | `src/fret/validation/quality_gates.py` |
| Validation package | `src/fret/validation/__init__.py` |
| Automated validation script | `scripts/validate_quality_gates.py` |
| Unit tests (metrics) | `tests/test_metrics.py` |
| Unit tests (gates) | `tests/test_quality_gates.py` |
| Report template + baseline | `docs/arco/baseline-report.md` |
| CI hook | `.github/workflows/tests.yml` step "Quality gate validation" |

## Acceptance Criteria

- [x] Metrics and thresholds are documented and versioned
  (`src/fret/config/benchmark.yaml`).
- [x] Baseline run produces a complete pass/fail report
  (`docs/arco/baseline-report.md`).
- [x] Unit tests for metric computation utilities pass (71 tests added).
- [x] `validate_quality_gates.py` exits 0 on the baseline configuration.
- [x] CI runs the quality gate step on every pull request.
- Any failing metric opens a follow-up issue with the observed value,
  threshold, and remediation plan.

## Dependencies

- Issue 08 (end-to-end pipeline available).
