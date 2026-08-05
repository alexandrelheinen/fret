# 6-DOF Manipulator — OpenMANIPULATOR-Y

> **Scenarios:** SC-v14a–c · **CV:** [vision/README.md](../vision/README.md) ·
> [Release spec](../releases.md#v124--6-dof-manipulator-challenge)

---

## Overview

**OpenMANIPULATOR-Y** from `third_party/robotis_mujoco_menagerie/robotis_omy`
(same submodule policy as TB3 and OM-X). Three MuJoCo physics scenarios mirror
the OM-X ladder without intermediate maze steps:

| Scenario | File | Purpose |
|---|---|---|
| SC-v14a | `omy_reach.yml` | Empty tabletop joint-space reach |
| SC-v14b | `omy_pick_place.yml` | Floor ball → place-bin pick-and-place (+ CV) |
| SC-v14c | `omy_clutter.yml` | Cluttered floor pick-and-place with detour |

Model key: `omy` (aliases: `six_dof`, `open_manipulator_y`).

---

## Capabilities

| Capability | Detail |
|---|---|
| DOF | 6 revolute |
| IK | Numerical (Jacobian-based) in `kinematics_open_manipulator_y.py` |
| Planning | Joint-space RRT* + fallback detour for clutter transfer |
| Control | Shared stack: ``PickPlaceFSM`` (robot-agnostic) → planner → ARCO ``JointSpaceMPC`` → MuJoCo joints |
| Simulation | MuJoCo SITL; physics pad contact + adhesion grasp (no kinematic carry or ball teleport) |

---

## Assets

| File | Purpose |
|---|---|
| `src/fret/mjcf/omy_tabletop.xml` | Empty reach cell |
| `src/fret/mjcf/omy_pick_place.xml` | Floor ball + place bin |
| `src/fret/mjcf/omy_clutter.xml` | Pick-place + transfer wall |
| `src/fret/mjcf/omy.py` | Menagerie inject + fingertip adhesion pads |
| `src/fret/config/scenarios/omy_*.yml` | SC-v14a–c parameters |
| `src/fret/control/omy_pick_place_sim.py` | Ground pick-place FSM runner |
| `src/fret/control/omy_clutter_sim.py` | Cluttered pick-place with planned detour |

Simulation: [mujoco.md](../mujoco.md).
