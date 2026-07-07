# PPP Gantry Robot (v1.0)

> **Release:** v1.0 · **Scenario:** SC-v10 (`ppp_warehouse`) ·
> [Release spec](../releases.md#v10--ppp-gantry-warehouse-pick-and-place)

---

## Overview

The PPP robot is a **3-axis gantry manipulator** with prismatic joints along X, Y, and Z.
It models an industrial high-bay warehouse crane used for pallet and box handling.

Configuration space **is** Cartesian workspace position: `q = (x, y, z)` in metres.

---

## Kinematic model

| Joint | Type | Range (v1.0) | Max velocity |
|---|---|---|---|
| `joint_x` | Prismatic | [0, 60] m | 3.0 m/s |
| `joint_y` | Prismatic | [0, 20] m | 3.0 m/s |
| `joint_z` | Prismatic | [0, 6] m | 1.5 m/s |

Forward kinematics: `p_ee = q` (identity map).

Inverse kinematics: `q_goal = p_target` (direct, no redundancy).

---

## End-effector

| Property | Value |
|---|---|
| Face size | 0.5 m × 0.5 m |
| Safety clearance | 1.5 m (≈ 2 × face diagonal) |

---

## Magnetic grasp (v1.0)

The cargo box attaches to the EE without a gripper mechanism:

```
IDLE → APPROACH → CAPTURE (weld) → TRANSPORT → RELEASE → IDLE
```

| State | Condition |
|---|---|
| CAPTURE | `‖p_ee − p_box‖ < capture_radius` |
| TRANSPORT | Box pose = EE pose + offset (fixed weld) |
| RELEASE | `‖p_ee − p_goal‖ < goal_radius` |

The welded box geometry is included in collision checking during TRANSPORT.

---

## ARCO alignment

FRET v1.0 PPP scenarios derive obstacle layouts from ARCO:

- `arco/map/ppp.yml`
- `arco/simulator/scenes/ppp.py`

Planning uses ARCO `SSTPlanner` with `KDTreeOccupancy` in 3-D C-space.

---

## Assets (planned)

| File | Purpose |
|---|---|
| `src/fret/mjcf/ppp_warehouse.xml` | MuJoCo gantry + warehouse |
| `src/fret/urdf/ppp.xacro` | Gazebo backend (optional; v1.2+) |
| `scripts/view.sh` | MuJoCo interactive viewer (v1.0) |
| `src/fret/config/scenarios/ppp_warehouse.yml` | SC-v10 scenario |
