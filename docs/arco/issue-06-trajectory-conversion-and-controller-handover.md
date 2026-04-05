# Issue 06: Trajectory Conversion and Controller Handover

## Goal

Convert ARCO planner paths into time-parameterized references consumable by FRET control/IK stack for robust trajectory tracking.

## Problem Statement

Planner outputs are geometric paths; controllers require temporally feasible references with kinematic limits.

## Functional Requirements

1. Convert path waypoints into time-parameterized trajectory.
2. Enforce joint/velocity/acceleration constraints.
3. Define and implement handover contract from planner module to controller module.
4. Provide graceful behavior for infeasible trajectory conversion.

## Non-Functional Requirements

- Conversion must be deterministic for same input/config.
- Timing jitter and command update rate requirements must be specified.

## V-Cycle Tasks

### Descending Branch

- Specify path-to-trajectory algorithm and constraints.
- Define controller interface payload and timing assumptions.
- Define test vectors including infeasible cases.

### Ascending Branch

- Unit tests for time parameterization and limit enforcement.
- Integration tests with FRET controller node in simulation.
- Tracking metrics report (position error, settling, overshoot).

## Deliverables

- trajectory conversion module.
- controller handover interface docs.
- tests and metric plots.

## Acceptance Criteria

- Generated trajectories satisfy configured kinematic limits.
- Controller consumes trajectory and tracks target in Gazebo.
- Failure states (infeasible trajectory) are explicit and recoverable.

## Dependencies

- Issue 05 (planner adapter).
- Existing FRET controller/IK path.
