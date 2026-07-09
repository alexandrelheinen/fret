# 6-DOF Manipulator (v1.4)

> **Release:** v1.4 · **Scenario:** SC-v14 ·
> [Release spec](../releases.md#v14--6-dof-manipulator-final-challenge)

---

## Overview

The 6-DOF release is the **final challenge**: a general revolute manipulator performing
full C-space planning and execution in a cluttered cell on **MuJoCo physics SITL**.

Specific robot model (UR5, UR3, or custom) will be selected at the start of v1.4 work.
Candidate meshes may be imported from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie).

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

*Specification will be expanded when v1.3 is complete.*

Simulation: [mujoco.md](../mujoco.md).
