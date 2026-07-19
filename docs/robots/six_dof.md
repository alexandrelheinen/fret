# 6-DOF Manipulator (v1.2.4)

> **Release:** v1.2.4 · **Scenario:** SC-v14 ·
> [Release spec](../releases.md#v124--6-dof-manipulator-challenge)

---

## Overview

The v1.2.4 release is the **simulation capstone** before v1.3 hardware: a general revolute
manipulator performing full C-space planning and execution in a cluttered cell on
**MuJoCo physics SITL**.

Specific robot model (UR5, UR3, or custom) will be selected at the start of v1.2.4 work.
Prefer **OpenMANIPULATOR-Y** from `third_party/robotis_mujoco_menagerie/robotis_omy` (same submodule policy as TB3 and OM-X).

---

## Planned capabilities

| Capability | Detail |
|---|---|
| DOF | 6 revolute |
| IK | Numerical (Jacobian-based) |
| Planning | ARCO SST in 6-D |
| Collision | Per-link FK + KDTree; self-collision in MJCF |
| Control | Jacobian pseudoinverse at 50 Hz |
| Simulation | MuJoCo physics SITL |

---

## Assets (planned)

| File | Purpose |
|---|---|
| `src/fret/mjcf/six_dof_cell.xml` | MuJoCo cell + robot |
| `src/fret/config/scenarios/six_dof_challenge.yml` | SC-v14 |

*Specification will be expanded when v1.2.3 is tagged.*

Simulation: [mujoco.md](../mujoco.md).
