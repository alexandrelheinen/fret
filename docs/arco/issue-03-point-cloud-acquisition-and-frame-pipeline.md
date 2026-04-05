# Issue 03: Point Cloud Acquisition and Frame Pipeline

## Goal

Acquire obstacle geometry from Gazebo as point cloud data and transform it into the canonical planning frame.

## Problem Statement

Planner correctness depends on accurate and timely obstacle geometry in a consistent frame.

## Functional Requirements

1. Acquire point cloud stream from simulation source(s).
2. Transform incoming cloud to canonical frame defined in Issue 01.
3. Filter non-obstacle points (robot self points, floor noise, out-of-range).
4. Provide update stream to occupancy adapter.

## Non-Functional Requirements

- Pipeline latency budget must be defined and measured.
- Frame conversion failures must be observable and testable.

## V-Cycle Tasks

### Descending Branch

- Specify sensor/topic source, update rates, and QoS assumptions.
- Define filtering rules and exclusion zones.
- Define test fixtures with synthetic clouds.

### Ascending Branch

- Unit tests for filtering/transformation logic.
- Integration tests with simulated TF changes.
- Latency and drop-rate report for baseline load.

## Deliverables

- point cloud ingestion module.
- frame/transform utility with tests.
- baseline metrics in docs.

## Acceptance Criteria

- Cloud is available in canonical frame with bounded latency.
- Filtering removes known non-obstacle geometry per spec.
- Pipeline emits clear diagnostics on TF failures.

## Dependencies

- Issue 01 (frame definitions).
- Issue 02 (Gazebo scenario).
