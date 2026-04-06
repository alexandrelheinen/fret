# Issue 04: Point Cloud to KD-Tree Occupancy Adapter

## Goal

Convert transformed obstacle point clouds into an ARCO-compatible KD-tree occupancy representation.

## Problem Statement

ARCO planners need an occupancy backend with stable query semantics; raw point clouds are insufficient for fast collision queries.

## Functional Requirements

1. Build/update KD-tree occupancy from incoming cloud snapshots.
2. Implement occupancy query API required by selected ARCO planner(s).
3. Support incremental updates or bounded full rebuild strategy.
4. Define collision radius/inflation policy.

## Non-Functional Requirements

- Query complexity and update complexity must be documented.
- Memory use and rebuild frequency must be bounded.

## V-Cycle Tasks

### Descending Branch

- Specify occupancy semantics (`is_free`, `is_occupied`, clearance threshold).
- Define inflation policy and safety margins.
- Define benchmark scenarios for query correctness.

### Ascending Branch

- Unit tests for occupancy queries against known fixtures.
- Property tests for monotonic safety behavior under inflation changes.
- Micro-benchmarks for query and update time.

## Deliverables

- occupancy adapter module.
- tests and benchmark script.
- docs for parameters and trade-offs.

## Acceptance Criteria

- Occupancy API is ARCO-compatible and tested.
- Collision queries pass fixture-based correctness tests.
- Performance metrics meet baseline thresholds defined in Issue 09.

## Dependencies

- Issue 03 (cloud in canonical frame).
- Issue 01 (contract definitions).

---

## Implementation

### Module

`src/fret/perception/occupancy_adapter.py` — pure Python, no ROS 2 runtime
dependency.

Re-exported by `fret.perception` alongside `CloudFilter` and `FrameTransform`.

### Class: `OccupancyAdapter`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `inflation_radius` | `float` | `0.05` | Collision safety margin (meters). Must be > 0. |
| `max_points` | `int` | `100 000` | Maximum stored obstacle points. Caps memory. |
| `max_rebuild_age` | `float` | `2.0` | Staleness threshold in seconds. |

#### Occupancy Semantics

| Concept | Definition |
|---------|------------|
| **occupied** | `nearest_distance(q) ≤ inflation_radius` (inclusive) |
| **free** | `nearest_distance(q) > inflation_radius` |
| **clearance** | `max(0, nearest_distance(q) − inflation_radius)` |

#### API

```python
adapter = OccupancyAdapter(inflation_radius=0.05)
adapter.update(filtered_points)        # rebuild from snapshot

adapter.is_occupied([x, y, z])         # bool
adapter.is_free([x, y, z])             # bool
adapter.clearance([x, y, z])           # float ≥ 0, or math.inf if empty
adapter.nearest_distance([x, y, z])    # float ≥ 0, or math.inf if empty

adapter.point_count                    # int
adapter.last_update_stamp              # float | None (POSIX seconds)
adapter.is_stale                       # bool
```

#### Rebuild Strategy

Full rebuild on every `update()` call.  Safe for snapshot pipelines at ≤ 10 Hz.
`is_stale` may be polled to trigger a fresh update when the tree age exceeds
`max_rebuild_age`.

#### Complexity

| Operation | Time | Space |
|-----------|------|-------|
| `update(n points)` | O(n log n) | O(n) |
| `is_occupied` / `is_free` / `clearance` | O(log n) avg, O(n) worst | O(log n) stack |
| `nearest_distance` | O(log n) avg, O(n) worst | O(log n) stack |

#### Inflation Policy

`inflation_radius` adds a uniform safety margin around every obstacle point.
This accounts for the finite volume of the robot end-effector and any sensor
noise in the point cloud.  A value of **0.05 m** is recommended for tabletop
manipulation; larger robots or noisier sensors may require larger values.

Safety is monotonic: increasing `inflation_radius` never converts an occupied
point to free, and decreasing it never converts a free point to occupied.

### Tests

`tests/test_occupancy_adapter.py` — 54 test cases covering:

- Constructor validation (9 cases)
- Tree update behavior (7 cases)
- Occupancy query correctness against known fixtures (14 cases)
- Multiple-obstacle fixtures (3 cases)
- Edge cases: empty cloud, single point, duplicates, negative coords (5 cases)
- Staleness detection (5 cases)
- Monotonic safety property tests (3 cases)
- Micro-benchmarks — informational, no timing thresholds (4 cases)
- Package import smoke tests (2 cases)
