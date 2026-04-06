# ARCO-FRET Quality Gate Baseline Report

**Issue:** [Issue 09: Validation Benchmarks and Quality Gates](issue-09-validation-benchmarks-and-quality-gates.md)
**Config:** `src/fret/config/benchmark.yaml`
**Script:** `scripts/validate_quality_gates.py`

---

## Report Template

| Scenario | Gate | Observed | Threshold | Op | Result |
|----------|------|----------|-----------|-----|--------|
| `easy` | success_rate | — | 1.0 fraction | >= | PASS/FAIL |
| `easy` | avg_latency_s | — | 0.2 s | <= | PASS/FAIL |
| `easy` | path_length | — | 3.0 rad_m | <= | PASS/FAIL |
| `easy` | smoothness | — | 5.0 rad | <= | PASS/FAIL |
| `easy` | min_clearance_m | — | 0.0 m | >= | PASS/FAIL |
| `medium` | success_rate | — | 1.0 fraction | >= | PASS/FAIL |
| `medium` | avg_latency_s | — | 1.0 s | <= | PASS/FAIL |
| `medium` | path_length | — | 5.0 rad_m | <= | PASS/FAIL |
| `medium` | smoothness | — | 10.0 rad | <= | PASS/FAIL |
| `medium` | min_clearance_m | — | 0.0 m | >= | PASS/FAIL |
| `hard` | success_rate | — | 0.66 fraction | >= | PASS/FAIL |
| `hard` | avg_latency_s | — | 15.0 s | <= | PASS/FAIL |
| `hard` | path_length | — | 8.0 rad_m | <= | PASS/FAIL |
| `hard` | smoothness | — | 20.0 rad | <= | PASS/FAIL |
| `hard` | min_clearance_m | — | 0.0 m | >= | PASS/FAIL |

---

## Baseline Run

**Environment:** Python 3.12, pure-Python (no ROS 2 runtime), fixed seed 42.
**Repeats per scenario:** 3.

```
========================================================================
ARCO-FRET Quality Gate Validation  (Issue 09)
Config : src/fret/config/benchmark.yaml
Repeats: 3
========================================================================
```

### easy — PASS (5/5 gates)

| Gate | Observed | Threshold | Op | Result |
|------|----------|-----------|-----|--------|
| success_rate | 1.0 fraction | 1.0 fraction | >= | **PASS** |
| avg_latency_s | 0.000127 s | 0.2 s | <= | **PASS** |
| path_length | 0.3873 rad_m | 3.0 rad_m | <= | **PASS** |
| smoothness | 0.0 rad | 5.0 rad | <= | **PASS** |
| min_clearance_m | inf m | 0.0 m | >= | **PASS** |

### medium — PASS (5/5 gates)

| Gate | Observed | Threshold | Op | Result |
|------|----------|-----------|-----|--------|
| success_rate | 1.0 fraction | 1.0 fraction | >= | **PASS** |
| avg_latency_s | 0.00386 s | 1.0 s | <= | **PASS** |
| path_length | 0.0 rad_m | 5.0 rad_m | <= | **PASS** |
| smoothness | 0.0 rad | 10.0 rad | <= | **PASS** |
| min_clearance_m | 0.2708 m | 0.0 m | >= | **PASS** |

### hard — PASS (5/5 gates)

| Gate | Observed | Threshold | Op | Result |
|------|----------|-----------|-----|--------|
| success_rate | 1.0 fraction | 0.66 fraction | >= | **PASS** |
| avg_latency_s | 0.000370 s | 15.0 s | <= | **PASS** |
| path_length | 0.3162 rad_m | 8.0 rad_m | <= | **PASS** |
| smoothness | 0.0 rad | 20.0 rad | <= | **PASS** |
| min_clearance_m | 0.13 m | 0.0 m | >= | **PASS** |

### Overall: PASS — 15/15 gates across 3 scenarios

---

## Metric Definitions

| Metric | Formula | Units |
|--------|---------|-------|
| `success_rate` | `#successful_runs / repeat` | fraction ∈ [0, 1] |
| `avg_latency_s` | mean wall-clock `solve_time` across all runs | seconds |
| `path_length` | Σ Euclidean distance between consecutive waypoints | rad_m |
| `smoothness` | Σ direction-change angle at each waypoint triple | radians |
| `min_clearance_m` | min clearance of all waypoints in successful paths | meters |
| `tracking_rmse` | √(Σ(actual − desired)² / (N·D)) | rad_m |

> **Note on path_length and smoothness:** The current RRT-Connect
> implementation returns a 2-waypoint path after shortcutting; this is a
> known characteristic of the planner under the current proxy validator.
> The path metrics will become more informative when a full FK-based
> validator is integrated.

---

## Remediation Policy

If any gate fails in a future run:

1. Open a follow-up GitHub issue with:
   - the failing metric name and observed value,
   - the threshold and operator,
   - a hypothesis for the root cause.
2. Reference this report and the failing commit SHA.
3. Attach the full `validate_quality_gates.py` output as a comment.
