# 6-DOF Manipulator — OpenMANIPULATOR-Y (v1.2.4)

> **Release:** v1.2.4 · **Scenarios:** SC-v14a–c ·
> [Release spec](../releases.md#v124--6-dof-manipulator-challenge)

---

## Overview

The v1.2.4 release uses **OpenMANIPULATOR-Y** from
`third_party/robotis_mujoco_menagerie/robotis_omy` (same submodule policy as
TB3 and OM-X). Three MuJoCo physics scenarios mirror the OM-X v1.2.3 ladder
without intermediate maze steps:

| Scenario | File | Purpose |
|---|---|---|
| SC-v14a | `omy_reach.yml` | Empty tabletop joint-space reach |
| SC-v14b | `omy_pick_place.yml` | Floor ball → cone pick-and-place |
| SC-v14c | `omy_clutter.yml` | Cluttered floor pick-and-place with detour |

Model key: `omy` (aliases: `six_dof`, `open_manipulator_y`).

---

## Capabilities

| Capability | Detail |
|---|---|
| DOF | 6 revolute |
| IK | Numerical (Jacobian-based) in `kinematics_open_manipulator_y.py` |
| Planning | Joint-space RRT* + fallback detour for clutter transfer |
| Control | Shared stack with OMX: ``PickPlaceFSM`` → planner (clutter) → ARCO ``JointSpaceMPC`` → MuJoCo joints |
| Simulation | MuJoCo SITL; pad-mid grasp targets + pad-mid carry through lift/transfer (fang + Ø86 mm ball) |

---

## Assets

| File | Purpose |
|---|---|
| `src/fret/mjcf/omy_tabletop.xml` | Empty reach cell |
| `src/fret/mjcf/omy_pick_place.xml` | Floor ball + place cone |
| `src/fret/mjcf/omy_clutter.xml` | Pick-place + transfer wall |
| `src/fret/mjcf/omy.py` | Menagerie inject + fingertip adhesion pads |
| `src/fret/config/scenarios/omy_*.yml` | SC-v14a–c parameters |
| `src/fret/control/omy_pick_place_sim.py` | Ground pick-place FSM runner |
| `src/fret/control/omy_clutter_sim.py` | Cluttered pick-place with planned detour |

Simulation: [mujoco.md](../mujoco.md).
