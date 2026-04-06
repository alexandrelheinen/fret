# Issue 05: ARCO Planner Adapter in FRET

## Goal

Integrate selected ARCO planner(s) into FRET through a stable adapter that
consumes occupancy and outputs collision-aware paths.

## Problem Statement

Without a dedicated adapter layer, planner integration will become tightly
coupled and hard to evolve.

## Functional Requirements

1. Implement planner adapter API in FRET:
   - input: start state, goal state, occupancy handle, planner config,
   - output: path, planning metadata, status code.

2. Support at least one ARCO continuous planner suitable for obstacle-rich scenes.
3. Expose planner parameters through FRET config.
4. Emit planner diagnostics (solve time, node count, fail reason).

## Non-Functional Requirements

- Adapter must be planner-agnostic for future planner swaps.
- Planner run must be bounded by configurable timeout.

## V-Cycle Tasks

### Descending Branch

- [x] Specify adapter interface and error contracts.
- [x] Define planner configuration schema and defaults.
- [x] Define deterministic planning test cases before implementation.

### Ascending Branch

- [x] Unit tests for adapter behavior and error paths.
- [x] Integration tests from occupancy input to valid path output.
- [x] Benchmark report for solve time in reference scenarios.

## Deliverables

| Artifact | Path |
|----------|------|
| Planner adapter module | `src/fret/planning/planner_adapter.py` |
| RRT-Connect planner | `src/fret/planning/rrt_connect.py` |
| Package init (re-exports) | `src/fret/planning/__init__.py` |
| Planner config file | `src/fret/config/planner.yaml` |
| Unit and integration tests | `tests/test_planner_adapter.py` |
| Benchmark script | `scripts/benchmark_planner.py` |

## Acceptance Criteria

- [x] Adapter returns valid path for baseline scenario (free space).
- [x] Timeout and no-solution paths are handled explicitly with status codes.
- [x] Diagnostics are emitted (`solve_time`, `node_count`, `failure_reason`).

## Implementation Notes

### Adapter API

The `PlannerAdapter` class in `src/fret/planning/planner_adapter.py` implements
the planning interface defined in `docs/arco/spec-integration-contract.md`.

**Constructor:**

```python
PlannerAdapter(
    occupancy_adapter: OccupancyAdapter,
    joint_limits: list[tuple[float, float]],
    state_validator: Callable[[list[float]], bool] | None = None,
    config: dict | None = None,
)
```

**plan() method:**

```python
result = adapter.plan(planning_request)
# result["status"]  ∈ {"success", "no_plan_found", "timeout", "invalid_request"}
# result["path"]    — list of waypoints or None
# result["solve_time"]    — seconds (float, always present)
# result["node_count"]    — tree nodes explored (int, always present)
# result["failure_reason"] — string or None
```

### Planner: RRT-Connect

`src/fret/planning/rrt_connect.py` implements bidirectional RRT-Connect
(Kuffner & LaValle, 2000).  It is suitable for obstacle-rich environments and
operates in pure Python without external dependencies.

Key properties:
- Two trees grow simultaneously from start and goal.
- A greedy shortcut smoother removes redundant waypoints after path extraction.
- Cooperative timeout check at the start of each iteration (no threads needed).
- Reproducible with a fixed `rng_seed` for regression testing.

### Collision Checking

The adapter is decoupled from robot kinematics through the `state_validator`
callable.  Callers supply a function `(joint_positions) -> bool` that maps a
joint configuration to a collision-free boolean.  For real robots the validator
must apply forward kinematics and query the `OccupancyAdapter`.

```python
def validator(q: list[float]) -> bool:
    ee_position = scara_fk(q)          # robot-specific FK
    return occupancy.is_free(ee_position)

adapter = PlannerAdapter(
    occupancy_adapter=occupancy,
    joint_limits=joint_limits,
    state_validator=validator,
)
```

If no validator is supplied, a free-space default (always `True`) is used.
**This default is only suitable for unit testing.**

### Configuration

All tunable parameters are in `src/fret/config/planner.yaml` and can be
overridden per-request via `PlanningRequest.planner_config`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `algorithm` | `rrt_connect` | Planner algorithm identifier |
| `default_timeout` | `5.0` s | Fallback timeout when `PlanningRequest.timeout` is omitted |
| `max_occupancy_age` | `2.0` s | Max age of occupancy snapshot |
| `rrt_connect.step_size` | `0.05` | Extension step (radians/meters) |
| `rrt_connect.max_iterations` | `10000` | Iteration budget |
| `rrt_connect.goal_bias` | `0.10` | Goal-sampling probability |
| `rrt_connect.rng_seed` | `null` | Seed for reproducibility |

### Status Codes

| `status` | Condition |
|----------|-----------|
| `success` | Valid path found within timeout |
| `no_plan_found` | Planner exhausted iteration budget without finding a path |
| `timeout` | Wall-clock deadline exceeded |
| `invalid_request` | Schema violation, stale occupancy, or unsupported algorithm |

### Diagnostics

Every `PlanningResult` includes:

- `solve_time` (float, seconds) — wall-clock duration of the planning call.
- `node_count` (int) — total tree nodes across both RRT-Connect trees.
- `failure_reason` (str or null) — human-readable reason on non-success status.

## Dependencies

- Issue 04 (occupancy adapter) — `OccupancyAdapter` is used for staleness
  checks and optionally in the `state_validator`.
- Issue 01 (contract definitions) — `PlanningRequest` / `PlanningResult`
  schema and status codes are defined in `spec-integration-contract.md`.
