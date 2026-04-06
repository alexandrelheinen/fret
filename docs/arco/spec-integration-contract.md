# ARCO-FRET Integration Contract

## Purpose

This document is the authoritative specification for the integration interface
between ARCO (motion planning) and FRET (robot execution). It defines component
ownership, data contracts, and failure semantics for all crossing-boundary
interactions.

No implementation in either system may depend on undocumented behavior across
this boundary. All interfaces must be validated by the tests referenced at the
end of this document.

---

## 1. Component Ownership

| Component | Owner | Responsibilities |
|-----------|-------|-----------------|
| Occupancy representation | ARCO | KD-tree construction, update, and query API |
| Planner logic | ARCO | Sampling, collision checking, path search, diagnostics |
| Scene acquisition | FRET | Point cloud capture, frame transform pipeline |
| Robot state | FRET | Joint positions, velocities, and forward kinematics |
| Control and execution | FRET | Trajectory interpolation, controller dispatch, feedback |
| TF broadcast | FRET | Maintains `world → base_link → sensor` transform chain |

Ownership boundaries are hard. ARCO must not read robot state directly from
ROS topics. FRET must not call ARCO planner internals; it only calls the
planning request interface defined in Section 3.

---

## 2. Architecture Overview

```
FRET                                ARCO
─────────────────────                ────────────────────────
SceneAcquisition                     OccupancyModel
  │  point cloud (world frame)          │  KD-tree
  ▼                                     ▼
OccupancyUpdatePayload ──────────► occupancy_update()
                                         │
RobotState (joint positions,             │
  TF snapshot)                           ▼
PlanningRequest ─────────────────► plan()
                                         │
                                         ▼
PlanningResult ◄─────────────────  (path, status, diagnostics)
  │
  ▼
TrajectoryConverter ──────────────► ControllerDispatch
  │  JointTrajectory (ROS message)
  ▼
ExecutionFeedback ◄──────────────── TrackingMonitor
```

### Sequence Diagram

```
FRET:Scene   FRET:State   FRET:Exec   ARCO:Occ   ARCO:Planner
    │             │             │          │            │
    │──publish──►│             │          │            │
    │            │             │          │            │
    │──OccupancyUpdatePayload──────────►  │            │
    │            │             │          │──update()  │
    │            │             │          │◄──done     │
    │            │             │          │            │
    │            │──PlanningRequest────────────────►   │
    │            │             │          │    │plan() │
    │            │             │          │    │       │
    │            │             │◄─────────────PlanningResult
    │            │             │          │            │
    │            │             │──execute trajectory   │
    │            │             │          │            │
    │            │             │──ExecutionFeedback───►│ (diagnostics only)
```

---

## 3. Data Interface Contracts

### 3.1 PlanningRequest

Sent by FRET to ARCO to request a collision-free path.

| Field | Type | Units | Constraints |
|-------|------|-------|-------------|
| `request_id` | `str` (UUID v4) | — | Non-empty, globally unique per session |
| `start_joint_positions` | `list[float]` | radians | Length equals robot DOF; within joint limits |
| `goal_joint_positions` | `list[float]` | radians | Length equals robot DOF; within joint limits |
| `joint_count` | `int` | — | Must equal `len(start_joint_positions)` and `len(goal_joint_positions)` |
| `occupancy_stamp` | `float` | POSIX seconds (UTC) | POSIX timestamp of the occupancy snapshot used |
| `timeout` | `float` | seconds | Positive; maximum planner wall-clock time (default 5.0 s) |
| `planner_config` | `dict` | — | Planner-specific keys; must include `algorithm` (str) |
| `reference_frame` | `str` | — | Must equal `"world"` |

Invariants:
- `start_joint_positions` and `goal_joint_positions` must be within the
  joint limit ranges declared by the URDF.
- `occupancy_stamp` must be no older than `max_occupancy_age` (defined in
  Section 4.2) relative to the planning call time.

### 3.2 PlanningResult

Returned by ARCO after a planning call.

| Field | Type | Units | Constraints |
|-------|------|-------|-------------|
| `request_id` | `str` | — | Echoes the originating `PlanningRequest.request_id` |
| `status` | `str` | — | One of: `"success"`, `"no_plan_found"`, `"timeout"`, `"invalid_request"` |
| `path` | `list[list[float]]` or `null` | radians | Non-empty when `status == "success"`; `null` otherwise |
| `waypoint_count` | `int` | — | Equals `len(path)` when `status == "success"`; 0 otherwise |
| `solve_time` | `float` | seconds | Wall-clock duration of the planning call; always present |
| `node_count` | `int` | — | Number of graph/tree nodes explored; 0 when not applicable |
| `failure_reason` | `str` or `null` | — | Human-readable reason when `status != "success"`; `null` on success |
| `reference_frame` | `str` | — | Echoes `PlanningRequest.reference_frame` |

Invariants:
- Every waypoint in `path` satisfies the same joint-limit constraints as the
  request fields.
- `waypoint_count` equals `len(path)` exactly.

### 3.3 OccupancyUpdatePayload

Sent by FRET to ARCO to update the world model.

| Field | Type | Units | Constraints |
|-------|------|-------|-------------|
| `update_id` | `str` (UUID v4) | — | Non-empty, monotonically increasing per session |
| `stamp` | `float` | POSIX seconds (UTC) | Timestamp of the point cloud capture |
| `reference_frame` | `str` | — | Must equal `"world"` |
| `points` | `list[list[float]]` | meters | Each point is `[x, y, z]`; list may be empty |
| `point_count` | `int` | — | Must equal `len(points)` |
| `voxel_size` | `float` | meters | Positive; voxel resolution for KD-tree construction |
| `source` | `str` | — | One of: `"lidar"`, `"depth_camera"`, `"simulated"` |

Invariants:
- `point_count` must equal `len(points)` exactly.
- All coordinates are expressed in the `world` frame.
- `voxel_size` is strictly positive and in the range [0.001, 1.0] meters.

### 3.4 ExecutionFeedback

Emitted by FRET during and after trajectory execution.

| Field | Type | Units | Constraints |
|-------|------|-------|-------------|
| `request_id` | `str` | — | Echoes the originating `PlanningRequest.request_id` |
| `status` | `str` | — | One of: `"executing"`, `"succeeded"`, `"tracking_failure"`, `"aborted"` |
| `stamp` | `float` | POSIX seconds (UTC) | Timestamp of this feedback message |
| `progress` | `float` | fraction [0.0, 1.0] | Completion fraction; 1.0 on success |
| `current_joint_positions` | `list[float]` | radians | Actual joint positions at feedback time |
| `tracking_error` | `float` | radians | RMS joint tracking error at feedback time |
| `message` | `str` or `null` | — | Human-readable detail; required when `status` is not `"executing"` |

Invariants:
- `progress` is non-decreasing within a single execution session.
- `tracking_error` is non-negative.
- `len(current_joint_positions)` equals the robot DOF.

---

## 4. Failure Semantics

### 4.1 no-plan-found

**Trigger**: ARCO planner exhausts its budget without finding a collision-free path.

**Response**:
- `PlanningResult.status = "no_plan_found"`
- `PlanningResult.path = null`
- `PlanningResult.failure_reason` contains a descriptive message.

**FRET behavior**: Log the failure at WARN level. Retain the current robot
state. Notify the calling component via the `ExecutionFeedback` channel with
`status = "aborted"`.

### 4.2 stale-occupancy

**Trigger**: The `PlanningRequest.occupancy_stamp` is older than
`max_occupancy_age` seconds at planning call time.

**Default `max_occupancy_age`**: 2.0 seconds.

**Response**:
- ARCO returns `PlanningResult.status = "invalid_request"`
- `PlanningResult.failure_reason` includes the string `"stale_occupancy"` and
  the actual age in seconds.

**FRET behavior**: Trigger a fresh occupancy update before retrying the
planning request. Do not execute a trajectory based on a rejected plan.

### 4.3 transform-unavailable

**Trigger**: FRET cannot obtain the required TF2 transform within the
configured lookup timeout.

**Default lookup timeout**: 0.5 seconds.

**Response**:
- FRET does not forward the `OccupancyUpdatePayload` or `PlanningRequest`.
- FRET logs the failure at ERROR level and emits an `ExecutionFeedback` with
  `status = "aborted"` and a `message` containing `"transform_unavailable"`.

**ARCO behavior**: ARCO does not receive this failure; no action required.

### 4.4 controller-tracking-failure

**Trigger**: The tracking error exceeds `max_tracking_error` radians for
longer than `tracking_failure_window` seconds.

**Default `max_tracking_error`**: 0.1 radians (RMS).
**Default `tracking_failure_window`**: 0.5 seconds.

**Response**:
- FRET controller node halts trajectory execution immediately.
- `ExecutionFeedback.status = "tracking_failure"` is published.
- FRET requests an emergency stop via the hardware interface.

**ARCO behavior**: ARCO may receive the `ExecutionFeedback` for diagnostics
purposes. It must not attempt to continue planning until a new `PlanningRequest`
is received.

---

## 5. Timing Assumptions

| Parameter | Value | Notes |
|-----------|-------|-------|
| `max_occupancy_age` | 2.0 s | Maximum age of occupancy snapshot at planning time |
| Planning `timeout` default | 5.0 s | Wall-clock planner budget |
| TF lookup timeout | 0.5 s | Maximum wait for required transform |
| `tracking_failure_window` | 0.5 s | Duration of sustained tracking error before abort |
| `max_tracking_error` | 0.1 rad | RMS joint error threshold |
| Feedback publish rate | 10 Hz | Minimum rate during active execution |

---

## 6. Decision Log

| Decision | Rationale |
|----------|-----------|
| All coordinates in `world` frame | Eliminates frame-dependent bugs at the boundary; FRET is responsible for all transforms before sending. |
| UUID v4 for request/update IDs | Globally unique without coordination; enables log correlation across systems. |
| `null` path on failure (not empty list) | Distinguishes "no path exists" from "zero-length path" (which would be a bug). |
| Explicit `point_count` field | Allows receivers to validate payload integrity without recomputing `len()`. |
| POSIX float timestamp (not ROS stamp) | Makes the contract language-agnostic and independent of ROS message types. |
| `max_occupancy_age` default of 2.0 s | Matches a 10 Hz scene update rate with one full update cycle of margin. |

---

## 7. Test References

Contract assumptions are validated by:

- `tests/test_integration_contracts.py` — schema validation, field constraints,
  failure code coverage, and end-to-end dry-run payload compatibility test.

See also `docs/arco/spec-frames-and-units.md` for frame and unit conventions.
