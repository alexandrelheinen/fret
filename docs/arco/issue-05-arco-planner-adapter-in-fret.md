# Issue 05: ARCO Planner Adapter in FRET

## Goal

Integrate selected ARCO planner(s) into FRET through a stable adapter that consumes occupancy and outputs collision-aware paths.

## Problem Statement

Without a dedicated adapter layer, planner integration will become tightly coupled and hard to evolve.

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

- Specify adapter interface and error contracts.
- Define planner configuration schema and defaults.
- Define deterministic planning test cases before implementation.

### Ascending Branch

- Unit tests for adapter behavior and error paths.
- Integration tests from occupancy input to valid path output.
- Benchmark report for solve time in reference scenarios.

## Deliverables

- planner adapter module in FRET.
- planner config file entries.
- tests and benchmark script.

## Acceptance Criteria

- Adapter returns valid path for baseline scenario.
- Timeout and no-solution paths are handled explicitly.
- Diagnostics are emitted and documented.

## Dependencies

- Issue 04 (occupancy adapter).
- Issue 01 (contract definitions).
