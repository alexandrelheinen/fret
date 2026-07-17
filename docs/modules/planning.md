# Planning Module

**Package:** `fret.planning`  
**Source:** `src/fret/planning/`  
**Tests:** `tests/planning/`

> **All releases** use ARCO SST via this module. Collision checking adapts per robot
> (SE(2) occupancy for Dubins, RRP FK, 6-DOF per-link FK in v1.4). See [releases.md](../releases.md).

---

## Responsibility

The planning module computes collision-free trajectories from start to goal. It wraps
ARCO's SST planner, post-processes the path, and delivers a time-parameterized
trajectory to the controller.

---

## Components

### `planner_node.py` — PlannerNode (Level 3)

Pure-Python planning state machine. Executes the full planning pipeline:

1. Validate goal against joint limits (`ABORTED / INVALID_CONFIGURATION` on failure).
2. Call ARCO SST with `CSpaceChecker` as collision predicate.
   Falls back to a 2-waypoint linear path when ARCO is absent.
3. Post-process via `TrajectoryGenerator`.
4. Return `PlanningResult(status=SUCCESS, path=[...])`.

**No ROS dependency** — fully testable with `pytest`.

---

### `planner_node_ros.py` — PlannerNodeRos (Level 4)

ROS 2 Action server (`PlanRequest.action`) wrapping `PlannerNode`. Handles
Action goal receipt, progress feedback (iteration count, elapsed time), and
result delivery. Publishes the resulting `JointTrajectory` on `/joint_trajectory`.

**Tests:** `tests/planning/test_planner_node.py`, `test_planner_node_ros.py`

---

### `cspace_checker.py` — CSpaceChecker

Adapts FRET's Kinematics engine to ARCO's occupancy interface. For each joint
configuration `q`:

```
FK(q) → world-frame EE position → KDTreeOccupancy.clearance() → collision-free?
```

Implements duck-typed occupancy interface — no direct ARCO import required.

**Tests:** `tests/planning/test_cspace_checker.py`

---

### `dubins_obstacles.py` — race structure layout

Loads rectangular structures from ``config/worlds/dubins_race_obstacles.yml``
for SE(2) occupancy and clearance checks during the Dubins race.

**Tests:** `tests/planning/` (Dubins occupancy / race planning)

---

### `trajectory_generator.py` — TrajectoryGenerator

Post-processing chain applied to the raw planner output:

1. **Pruning:** removes redundant waypoints (`TrajectoryPruner`).
2. **Optimization:** time-optimal refinement (`TrajectoryOptimizer`).
3. **Interpolation:** C²-smooth B-spline interpolation (`BSplineInterpolator`).

When ARCO is absent, generates a simple linear interpolation between waypoints.
Output: `trajectory_msgs/JointTrajectory` (ROS 2 message).

**Tests:** `tests/planning/test_trajectory_generator.py`

---

### `trajectory_converter.py` — TrajectoryConverter

Converts a raw joint-space path (list of `np.ndarray` waypoints) into a
time-parameterized trajectory using per-joint trapezoidal velocity profiles.
Uses triangular profiles for short motions. Resamples at `control_hz` (50 Hz).

| Parameter | Default |
|---|---|
| `v_max` | `[1.5, 1.5, 0.1]` rad/s or m/s |
| `a_max` | `[2.0, 2.0, 0.2]` rad/s² or m/s² |
| `control_hz` | 50 Hz |

**Tests:** `tests/planning/test_trajectory_converter.py` — 94 unit tests

---

### `replanning_manager.py` — ReplanningManager

Higher-level state machine that manages replanning triggers, debouncing, and
attempt counting. Wraps `PlannerNode` and `TrajectoryConverter`.

#### FSM States

```
IDLE ──[start_execution]──► EXECUTING ──[trigger_replan]──► REPLANNING
                                                                 │
                                          [max attempts]         ▼
                                                              HALTED
```

**Tests:** `tests/planning/test_replanning_manager.py` — 76 unit tests

---

## ARCO Integration

ARCO is an **optional** dependency:

```python
try:
    from arco.planning import SST
except ImportError:
    SST = None  # fallback to linear interpolation
```

When ARCO is installed, `PlannerNode` uses SST with `CSpaceChecker` as the
collision predicate. When absent, a 2-waypoint straight-line path is returned.

See [docs/arco.md](../arco.md) for ARCO capabilities and integration boundary.

---

## Satisfies Requirements

| Requirement | Description |
|---|---|
| FR-PLN-01 | Collision-free joint-space path from start to goal |
| FR-PLN-02 | C-space planning; FK → KDTreeOccupancy collision check |
| FR-PLN-03 | Action feedback: iteration count, cost, elapsed time |
| FR-PLN-04 | Planning timeout ≤ 30 s |
| FR-PLN-05 | `ABORTED` on failure; no auto-retry |
| FR-PLN-06 | Post-processing: prune → optimize → B-spline |
| FR-PLN-07 | Joint limit validation before invoking planner |
