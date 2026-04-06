# ARCO-FRET Frames and Units Specification

## Purpose

This document defines the canonical coordinate frame strategy and physical unit
conventions used across the ARCO-FRET integration boundary. All data exchanged
between ARCO and FRET must conform to the frames and units defined here.

---

## 1. Coordinate Frame Strategy

The integration boundary uses a single canonical reference frame for all
exchanged data. Frame transforms are FRET's responsibility; ARCO consumes
pre-transformed data and never performs frame lookups.

### 1.1 Canonical Frames

| Frame ID | Owner | Description |
|----------|-------|-------------|
| `world` | FRET (broadcast) | Inertial world frame; origin at simulation/scene reference. All exchanged data must be expressed in this frame. |
| `base_link` | FRET | Robot base frame; attached to the robot mounting point. Origin at the center of the base flange. |
| `sensor_frame` | FRET | Sensor optical frame; origin at the sensor focal point or lidar center of rotation. |
| `tool0` | FRET | End-effector frame; attached to the robot flange (before tool offset). |

### 1.2 Frame Chain

```
world
  └── base_link          (static or slow-moving; broadcast by FRET at 100 Hz)
        └── link_0
              └── link_1
                    └── ... (per URDF kinematic chain)
                          └── tool0

world
  └── sensor_frame       (static mount; broadcast by FRET at 100 Hz)
```

The `world → base_link` transform is the only transform crossing the
integration boundary that matters for correctness. All occupancy data and
planning coordinates must be expressed in `world` before being sent to ARCO.

### 1.3 Transform Requirements

| Transform | Source | Minimum Rate | Notes |
|-----------|--------|--------------|-------|
| `world → base_link` | FRET TF broadcaster | 100 Hz | Must be available before any planning request |
| `world → sensor_frame` | FRET TF broadcaster | 100 Hz | Must be available before any occupancy update |
| `base_link → tool0` | FRET (FK from joint states) | 100 Hz | Used internally by FRET; not sent to ARCO |

### 1.4 Frame Lookup Policy

FRET must look up and apply transforms before sending any payload to ARCO.
If a required transform is unavailable within the configured timeout (default
0.5 s), FRET must not send the payload and must emit an `ExecutionFeedback`
with `status = "aborted"` and `message = "transform_unavailable"`.

ARCO must not call TF2. ARCO must treat all received data as already expressed
in the `world` frame.

---

## 2. Physical Unit Conventions

All physical quantities at the integration boundary use SI units unless an
explicit exemption is documented here.

### 2.1 Primary Unit Table

| Quantity | Unit | Symbol | Notes |
|----------|------|--------|-------|
| Length (positions, point cloud) | meter | m | 3D Cartesian coordinates in `world` frame |
| Joint angle | radian | rad | Joint space positions and goal states |
| Joint velocity | radian per second | rad/s | Not transmitted at the boundary in v1; reserved |
| Time (timestamps) | second | s | POSIX float (seconds since 1970-01-01T00:00:00 UTC) |
| Duration (timeout, age) | second | s | Wall-clock duration; strictly positive |
| Angular error (tracking) | radian | rad | RMS error across all joints |
| Voxel size | meter | m | KD-tree leaf size for occupancy representation |
| Progress | dimensionless fraction | — | Range [0.0, 1.0]; 1.0 = complete |

### 2.2 Conventions and Invariants

- **Joint angles** are always expressed in radians at the integration boundary.
  Human-readable configuration files (YAML) may use degrees and must document
  the unit with a comment.
- **Timestamps** use POSIX float seconds (not ROS `builtin_interfaces/Time`
  stamps). FRET is responsible for converting ROS timestamps before sending.
- **Positions** are always 3-element `[x, y, z]` vectors in meters in the
  `world` frame.
- **Velocity quantities** are reserved for future interface versions and must
  not appear in v1 payloads.

### 2.3 Angle Wrapping

Joint angles are not wrapped to `[−π, π]` at the boundary. They reflect the
raw joint state values from the URDF joint limits. It is the caller's
responsibility to ensure values are within `[lower_limit, upper_limit]` as
declared in the URDF.

---

## 3. Axis Conventions

The `world` frame follows the REP-103 convention used by ROS 2:

| Axis | Direction |
|------|-----------|
| +X | Forward |
| +Y | Left |
| +Z | Up |

Rotations follow the right-hand rule about each axis.

The `sensor_frame` optical frame follows the ROS camera convention:

| Axis | Direction |
|------|-----------|
| +X | Right |
| +Y | Down |
| +Z | Forward (optical axis) |

FRET is responsible for transforming sensor data from the optical frame to the
`world` frame before forwarding point clouds to ARCO.

---

## 4. Timing Conventions

### 4.1 Timestamp Semantics

All `stamp` fields in the integration payloads represent the time at which the
data was captured or computed, not the time at which it was sent. Receivers
must use the `stamp` field for staleness checks, not the message arrival time.

### 4.2 Clock Source

Both FRET and ARCO use the system clock (wall clock). In simulation, FRET
synchronizes its clock to the Gazebo simulation clock via the `/clock` topic.
ARCO must be configured to use the same clock source.

### 4.3 Maximum Latency Budget

The following latency budget applies to the full scene-update-to-execution path:

| Stage | Budget |
|-------|--------|
| Point cloud capture to `OccupancyUpdatePayload` send | ≤ 100 ms |
| `OccupancyUpdatePayload` send to ARCO update complete | ≤ 200 ms |
| Planning request to `PlanningResult` return | ≤ 5000 ms (configurable `timeout`) |
| `PlanningResult` to trajectory execution start | ≤ 50 ms |
| Trajectory execution to first `ExecutionFeedback` | ≤ 100 ms |

Total worst-case latency (excluding planning time): ≤ 450 ms.

---

## 5. Joint Space Convention

### 5.1 Joint Ordering

Joint arrays in `PlanningRequest`, `PlanningResult`, and `ExecutionFeedback`
follow the joint ordering declared in the URDF `<joint>` elements, in
depth-first traversal order of the kinematic tree from `base_link` to `tool0`.

For the SCARA robot: `[joint_1, joint_2, joint_3, joint_4]` (four revolute
joints, root to tip).

### 5.2 Joint Limits

Joint limits are defined in the URDF and are the single source of truth. No
interface contract field may override URDF joint limits. Both ARCO and FRET
must validate joint positions against URDF limits before accepting a request.

---

## 6. Voxel and Occupancy Conventions

| Parameter | Default Value | Unit | Notes |
|-----------|---------------|------|-------|
| `voxel_size` | 0.05 | m | 5 cm cells; balances memory and resolution |
| Minimum `voxel_size` | 0.001 | m | 1 mm; finer resolution requires explicit justification |
| Maximum `voxel_size` | 1.0 | m | 1 m; coarser resolution is not useful for manipulation |
| Maximum point count per update | 500 000 | — | Larger payloads must be decimated before sending |

---

## 7. Decision Log

| Decision | Rationale |
|----------|-----------|
| REP-103 axes for `world` frame | Aligns with all standard ROS 2 tools and avoids axis-flip bugs. |
| POSIX float for all timestamps | Language-agnostic and avoids ROS message dependency in the contract. |
| FRET owns all TF lookups | Keeps ARCO independent of the ROS TF subsystem; simplifies testing. |
| Joint order = URDF depth-first | Deterministic and self-documenting; requires no additional metadata. |
| Radians at boundary, degrees allowed in config | Radians are the SI unit; degrees aid human-readable tuning. |
| 5 cm default voxel size | Empirical compromise: sufficient for arm-scale obstacles without excessive memory. |

---

## 8. Test References

Frame and unit conventions are validated by:

- `tests/test_integration_contracts.py` — unit consistency tests, frame
  labeling tests, joint ordering validation, and timestamp semantics tests.

See also `docs/arco/spec-integration-contract.md` for the full data interface
contracts.
