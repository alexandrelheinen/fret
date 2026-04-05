# Issue 02: Gazebo Obstacle World and Complex Robot Scenario

## Goal

Create a deterministic Gazebo scenario with a complex manipulator and realistic static obstacles suitable for planning and tracking validation.

## Problem Statement

Planning quality cannot be validated without a stable environment that includes narrow passages, occlusions, and reachable manipulation targets.

## Functional Requirements

1. Select and integrate one complex robot model in FRET for this milestone.
2. Create a Gazebo world containing:
- floor and support structures,
- at least 5 obstacle primitives/meshes,
- one start and one target pose region.

3. Publish all required transforms and robot states for downstream perception/planning.

## Non-Functional Requirements

- World setup must be reproducible from version-controlled assets.
- Environment must have deterministic spawn and startup behavior.

## V-Cycle Tasks

### Descending Branch

- Specify world geometry, obstacle IDs, and target pose definitions.
- Define reachability and collision challenge cases.
- Define startup success criteria and timeout budgets.

### Ascending Branch

- Add launch smoke tests for world + robot startup.
- Add checks that obstacle entities and TF frames are present.
- Record one validation artifact (image/video) proving scenario boot correctness.

## Deliverables

- world file and launch integration in FRET.
- scenario specification document under `docs/arco/`.

## Acceptance Criteria

- Single command launches robot + world + required topics/TF.
- Obstacle set appears with stable names and positions.
- Scenario can be replayed with equivalent initial state.

## Dependencies

- Issue 01 (contracts for frames and interfaces).
