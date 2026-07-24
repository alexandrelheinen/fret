# Control Module

**Package:** `fret.control`  
**Source:** `src/fret/control/`  
**Tests:** `tests/control/`

> **Release roadmap:** v1.1 ARCO Dubins path-following MPC; v1.2 MuJoCo physics;
> v1.2.3 OMX `JointSpaceMPC` + FSM; v1.2.4 OMY 6-DOF; v1.4+ ball pose from
> `fret.vision`. See [releases.md](../releases.md).

---

## Responsibility

The control module implements kinematic computations and trajectory tracking
controllers. The current implementation targets the **bootstrap arm (manipulator)**;
v1.1+ adds per-model kinematics selected by `model:=` at launch.

---

## Components

### `kinematics.py` — Kinematics

Closed-form forward kinematics (FK), analytical inverse kinematics (IK), and
the geometric Jacobian for the arm manipulator (3-DOF) manipulator.

| Method | Description |
|---|---|
| `fk(q)` | Returns `EEPose(position, quaternion)` from joint configuration `q` |
| `ik(pos, z_offset)` | Returns all valid joint configurations reaching Cartesian target |
| `jacobian(q)` | Returns the 3×3 geometric Jacobian matrix |
| `is_within_limits(q)` | Validates joint configuration against the operational envelope |

**arm model constants** (from `open_manipulator_x.xacro`):

| Parameter | Value |
|---|---|
| Link 1 length (`L1`) | 0.325 m |
| Link 2 length (`L2`) | 0.275 m |
| Joint 1 (`joint_arm_0`) range | ±132° |
| Joint 2 (`joint_arm_1`) range | ±150° |
| Joint 3 (`joint_extension`) range | [0, 0.2] m |
| EE base height (`z_base`) | 0.238 m |

**Tests:** `tests/control/test_kinematics.py` — 19 unit tests covering FK/IK round-trip,
Jacobian finite-difference validation, and joint limit checks.

---

### `controller_node.py` — ControllerNode

Jacobian-based 50 Hz trajectory tracking controller, organized in two levels:

- **Level 3 (`ControllerNode`):** Pure-Python state machine and Jacobian tracking.
  Fully testable without a ROS context.
- **Level 4 (`ControllerRosNode`):** Subclasses `rclpy.node.Node`; wires 50 Hz timer,
  `/joint_trajectory` subscription, `/joint_states` subscription (via `StateEstimator`),
  and `/joint_commands` publisher.

#### FSM States

```
IDLE ──[trajectory received]──► TRACKING ──[fault]──► HALTED
       ◄──[trajectory complete]──
```

| State | Description |
|---|---|
| `IDLE` | Waiting for a trajectory message |
| `TRACKING` | Actively publishing `/joint_commands` at 50 Hz |
| `HALTED` | EE error exceeded 20 mm for > 0.5 s; all commands zeroed |

#### Key behavior

- Computes Jacobian pseudoinverse at each step; uses damped least-squares
  (`λ = 0.01`) to avoid singularities.
- Fault threshold: 20 mm EE error sustained for 0.5 s → publishes fault,
  transitions to `HALTED`.
- Default Cartesian proportional gain `Kp = 20.0`.

**Topics (Level 4):**

| Topic | Type | Direction |
|---|---|---|
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Subscribe |
| `/joint_states` | `sensor_msgs/JointState` | Subscribe (via StateEstimator) |
| `/joint_commands` | `std_msgs/Float64MultiArray` | Publish |
| `/controller_fault` | `std_msgs/Bool` | Publish on fault |

**Tests:** `tests/control/test_controller_node.py`, `test_controller_node_ros.py`,
`test_controller_node_tracking.py` — covers FSM transitions, Jacobian tracking,
fault injection, and ROS wiring.

---

### `state_estimator.py` — StateEstimator

Subscribes to `/joint_states`, runs FK, and broadcasts the `base_link → tool0`
TF2 transform. Provides a `RobotState` snapshot to the controller at each step.

**Tests:** `tests/control/test_state_estimator.py`

---

## Configuration

Controller gains are in `src/fret/config/controllers/open_manipulator_x.yml`:

```yaml
kp: 20.0
damping: 0.01
max_velocity: 1.57  # rad/s
fault_threshold: 0.020  # m
rate_hz: 50.0
```

Dubins MPC / vehicle limits live in `src/fret/config/controllers/dubins.yml`.
RRT*/SST agents use ARCO `DubinsPathFollowingMPC` with progress-first
weights (`weight_lag`, `contour_deadzone`) loaded by
`fret.scenario.dubins_race_runner._mpc_config`. The free lateral band is
lab-scaled (~chassis half-width), not ARCO city meters — see
[arco.md](../arco.md#progress-first-contouring-arco147).
OMX joint-space MPC helpers live in `src/fret/control/joint_mpc.py`
(unaffected by progress-first contouring).

---

## Satisfies Requirements

| Requirement | Description |
|---|---|
| FR-CTL-01 | Controller runs at 50 Hz |
| FR-CTL-02 | Tracking error within release limits |
| FR-CTL-03 | Velocity commands published to `/joint_commands` |
| FR-CTL-04 | FK → TF2 broadcast at 50 Hz |
| FR-CTL-05 | Controller independent of planning node |
| FR-CTL-06 | Fault detection and HALTED state |
