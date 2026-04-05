# Issue 01: Integration Specification and Interface Contracts

## Goal

Define the authoritative system specification for ARCO-FRET integration, including data contracts, ownership boundaries, coordinate frames, and failure behavior.

## Problem Statement

Without a stable interface contract, planning, perception, and control teams will diverge, causing hidden integration failures late in the milestone.

## Functional Requirements

1. Define component ownership:
- ARCO owns occupancy representation and planner logic.
- FRET owns scene acquisition, robot state, control, and execution.

2. Define canonical frame strategy:
- world frame,
- robot base frame,
- sensor frame,
- and required transforms.

3. Define data interfaces:
- planning request,
- planning result,
- occupancy update payload,
- execution feedback.

4. Define failure semantics:
- no-plan-found,
- stale occupancy,
- transform unavailable,
- controller tracking failure.

## Non-Functional Requirements

- Contracts must be implementation-agnostic.
- Contracts must include explicit units and timing assumptions.
- Contracts must be testable via deterministic fixtures.

## V-Cycle Tasks

### Descending Branch

- Write architecture diagram and sequence diagram.
- Write contract tables (field name, type, units, constraints).
- Define acceptance tests before any adapter implementation.

### Ascending Branch

- Add schema-level validation tests for request/response payloads.
- Add transform and unit-consistency integration tests.
- Produce a short decision log with rationale for chosen contract.

## Deliverables

- `docs/arco/spec-integration-contract.md`
- `docs/arco/spec-frames-and-units.md`
- tests validating contract assumptions.

## Acceptance Criteria

- All interfaces are documented with units, reference frames, and error behavior.
- At least one end-to-end dry-run test validates payload compatibility.
- No unresolved TODO remains in contract docs.

## Dependencies

None (this issue is the milestone entry point).
