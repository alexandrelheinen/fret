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
