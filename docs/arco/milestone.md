# Milestone 1: ARCO x FRET Integration for Obstacle-Aware Autonomous Manipulation

## Milestone Intent

Integrate ARCO planning capabilities into FRET so that a complex manipulator can:

1. Perceive obstacle geometry from Gazebo.
2. Convert perception into a continuous occupancy model (KD-tree).
3. Plan collision-free motion using ARCO.
4. Execute and track trajectories with FRET controllers in Gazebo.

This milestone is explicitly inspired by the PPP approach (planning in continuous space), but for a more complex robot and with perception-driven obstacle maps instead of manually coded obstacle sets.

## Why This Is Doable

Yes, this is feasible and technically coherent.

- ARCO already provides planning and occupancy abstractions that can be reused.
- FRET already owns the kinematics/control and Gazebo execution path.
- The integration boundary is clean:
  - ARCO: world model + planning logic.
  - FRET: robot model, IK/control, and trajectory execution.

The key complexity is not a single algorithm; it is system integration quality:
- synchronized frames,
- consistent collision model,
- robust replanning triggers,
- and validation discipline.

## Scope

### In Scope

- Simulation-only (SITL) end-to-end pipeline in Gazebo.
- Complex robot model already available in FRET (or selected and integrated before planning tests).
- Point cloud to occupancy conversion.
- ARCO planner integration with obstacle avoidance.
- Trajectory execution in Gazebo through FRET control stack.
- Metric-driven validation and reproducible experiments.

### Out of Scope

- Real hardware deployment (HITL/physical robot).
- Vision model training.
- Advanced dynamic obstacle prediction.
- Production hard real-time guarantees.

## System Requirements (Milestone Acceptance Level)

1. The planning scene must be generated from Gazebo obstacle geometry (point cloud source).
2. The occupancy model must be represented as a KD-tree-compatible structure usable by ARCO planners.
3. The planner must output a collision-free path for at least one complex manipulation scenario.
4. FRET must convert planner output into executable references and track them in Gazebo.
5. At least one replanning scenario must be demonstrated when the scene changes.
6. Validation artifacts must include metrics and at least one reproducible demo launch.

## Spec-Driven Development (V-cycle) Policy for This Milestone

Every issue in this milestone must include both sides of the V-cycle:

1. Descending branch
- Requirements and acceptance criteria.
- Architecture and interface contracts.
- Test strategy before implementation.

2. Ascending branch
- Unit tests.
- Integration tests.
- System acceptance evidence (logs, plots, videos/images).

No issue is considered done if only code is merged without the matching validation evidence.

## Proposed Issue Breakdown

1. [Issue 01 - Integration Specification and Interface Contracts](issue-01-integration-spec-and-interface-contracts.md)
2. [Issue 02 - Gazebo Obstacle World and Complex Robot Scenario](issue-02-gazebo-obstacle-world-and-complex-robot-scenario.md)
3. [Issue 03 - Point Cloud Acquisition and Frame Pipeline](issue-03-point-cloud-acquisition-and-frame-pipeline.md)
4. [Issue 04 - Point Cloud to KD-Tree Occupancy Adapter](issue-04-point-cloud-to-kdtree-occupancy-adapter.md)
5. [Issue 05 - ARCO Planner Adapter in FRET](issue-05-arco-planner-adapter-in-fret.md)
6. [Issue 06 - Trajectory Conversion and Controller Handover](issue-06-trajectory-conversion-and-controller-handover.md)
7. [Issue 07 - Replanning Triggers and Scene Update Loop](issue-07-replanning-triggers-and-scene-update-loop.md)
8. [Issue 08 - End-to-End SITL Launch and Demonstration](issue-08-end-to-end-sitl-launch-and-demonstration.md)
9. [Issue 09 - Validation Benchmarks and Quality Gates](issue-09-validation-benchmarks-and-quality-gates.md)
10. [Issue 10 - Documentation Closure and Milestone Review](issue-10-documentation-closure-and-milestone-review.md)

## Exit Criteria for Milestone Completion

The milestone is complete only when all these are true:

- All 10 issues are closed with acceptance evidence.
- End-to-end SITL demo is reproducible from a documented launch command sequence.
- Path planning and execution metrics are documented and meet threshold values defined in Issue 09.
- FRET docs are updated with architecture, usage, and known limitations.
- Remaining technical debt is captured as explicit backlog items for Milestone 2.
