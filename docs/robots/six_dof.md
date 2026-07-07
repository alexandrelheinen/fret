# 6-DOF Manipulator (v1.3)

> **Release:** v1.3 · **Scenario:** SC-v13 ·
> [Release spec](../releases.md#v13--6-dof-manipulator-final-challenge)

---

## Overview

The 6-DOF release is the **final challenge**: a general revolute manipulator performing
full C-space planning and execution in a cluttered cell.

Specific robot model (UR5, UR3, or custom) will be selected at the start of v1.3 work.

---

## Planned capabilities

| Capability | Detail |
|---|---|
| DOF | 6 revolute |
| IK | Numerical (Jacobian-based) |
| Planning | ARCO SST in 6-D |
| Collision | Per-link FK + KDTree; self-collision |
| Control | Jacobian pseudoinverse at 50 Hz |

---

## Assets (planned)

| File | Purpose |
|---|---|
| `src/fret/urdf/six_dof.xacro` | Robot description |
| `src/fret/mjcf/six_dof_cell.xml` | MuJoCo cell |
| `src/fret/config/scenarios/six_dof_challenge.yml` | SC-v13 |

*Specification will be expanded when v1.2 is complete.*
