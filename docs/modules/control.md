# Control Module

**Package:** `fret.control`  
**Source:** `src/fret/control/`  
**Tests:** `tests/control/`

> **Release roadmap:** v1.0 adds PPP prismatic control + magnetic grasp FSM;
> v1.1 uses ARCO Dubins/Pure Pursuit; v1.2 enables MuJoCo physics actuators;
> v1.3 extends existing SCARA Jacobian stack; v1.4 adds 6-DOF numerical IK.
> See [releases.md](../releases.md).

---

## Responsibility

The control module implements kinematic computations and trajectory tracking
controllers. The current implementation targets the **bootstrap SCARA (RRP)**;
v1.0+ adds per-model kinematics selected by `model:=` at launch.

---

## Components

### `kinematics.py` — Kinematics

Closed-form forward kinematics (FK), analytical inverse kinematics (IK), and
the geometric Jacobian for the SCARA RRP (3-DOF) manipulator.

| Method | Description |
|---|---|
| `fk(q)` | Returns `EEPose(position, quaternion)` from joint configuration `q` |
| `ik(pos, z_offset)` | Returns all valid joint configurations reaching Cartesian target |
| `jacobian(q)` | Returns the 3×3 geometric Jacobian matrix |
| `is_within_limits(q)` | Validates joint configuration against the operational envelope |

**SCARA model constants** (from `scara.xacro`):

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

### `kinematics_ppp.py` — PPPKinematics (v1.0)

Identity-map FK/IK for the PPP gantry (`q ≡ p_ee`). Selected via
`Kinematics(model="ppp")`.

| Property / method | PPP value |
|---|---|
| `dof` | 3 |
| `joint_names` | `joint_x`, `joint_y`, `joint_z` |
| `joint_limits` | X [0, 60], Y [0, 20], Z [0, 6] m |
| `forward_kinematics(q)` | EE position = `q` |
| `inverse_kinematics(pose)` | `q = p_ee` with limit check |

**Tests:** `tests/control/test_kinematics_ppp.py` — 15 unit tests.

---

### `grasp_magnet.py` — MagneticGraspFSM (v1.0)

Pure-Python magnetic weld / release state machine for PPP pick-and-place.

```
IDLE → APPROACH → CAPTURE → TRANSPORT → RELEASE → IDLE
```

| Symbol | Description |
|---|---|
| `GraspState` | FSM states (FR-GSP-04) |
| `GraspConfig` | `capture_radius`, `goal_radius`, `weld_offset`, `box_half_extent` |
| `MagneticGraspFSM` | Level-3 FSM; call `begin_transport()` then `update(ee, box, goal)` |
| `is_welded` | `True` during CAPTURE / TRANSPORT |
| `cargo_position` | World-frame cargo centre (tracks EE when welded) |
| `cargo_corners()` | 8 corners of cargo AABB (FR-GSP-02 hook for planning) |

**Demo:** `python3 scripts/demo_grasp.py`  
**Tests:** `tests/control/test_grasp_magnet.py` — 10 unit tests.

---

### `controller_ppp.py` — PPPControllerNode (v1.0)

Per-axis proportional velocity control for the PPP gantry at 50 Hz.
Because ``q ≡ p_ee``, joint-space P-control is equivalent to Cartesian
P-control:

```
q̇ = Kp · (q_ref − q)   (clipped per joint)
```

| Property / method | Description |
|---|---|
| `compute_prismatic_command(kin, q)` | Per-axis velocity command [m/s] |
| `get_ee_error_m(kin, q)` | EE position error against current waypoint |
| `set_trajectory(waypoints)` | Load waypoints and enter ``TRACKING`` |
| `fault_threshold` | 10 mm joint-space error → ``HALTED`` |

**Config:** `src/fret/config/controllers/ppp.yml` — Kp=1.5,
max velocity [3, 3, 1.5] m/s, 50 Hz.

**Demo:** `python3 scripts/demo_ppp_controller.py`  
**Tests:** `tests/control/test_controller_ppp.py` — 12 unit tests.

Use ``make_controller_node(model, config_path)`` to dispatch PPP vs SCARA.

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

Controller gains are in `src/fret/config/controllers/jacobian.yml`:

```yaml
kp: 20.0
damping: 0.01
max_velocity: 1.57  # rad/s
fault_threshold: 0.020  # m
rate_hz: 50.0
```

---

## Satisfies Requirements

| Requirement | Description |
|---|---|
| FR-CTL-01 | Controller runs at 50 Hz |
| FR-CTL-02 | EE tracking error ≤ 5 mm in SITL |
| FR-CTL-03 | Jacobian pseudoinverse velocity commands |
| FR-CTL-04 | FK → TF2 broadcast at 50 Hz |
| FR-CTL-05 | Controller independent of planning node |
| FR-CTL-06 | Fault detection and HALTED state |
| FR-GSP-01 | Magnetic weld grasp (no finger DOF) |
| FR-GSP-03 | Release at goal; cargo frozen |
| FR-GSP-04 | Grasp FSM states |
