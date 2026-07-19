# OpenMANIPULATOR-X (v1.3)

> **Release:** v1.3 · **Scenarios:** SC-v13a (`omx_reach`), SC-v13b (`omx_pick_place`),
> SC-v13c (`omx_desk_clutter`), SC-v13d (`omx_wall_maze`) ·
> [Release spec](../releases.md#v13--openmanipulator-x-tabletop)

---

## Overview

**ROBOTIS OpenMANIPULATOR-X** — 4 revolute DOF + parallel gripper — loaded from the
Menagerie submodule:

`third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x/`

v1.3 showcase:

1. **SC-v13a** — empty tabletop, end-effector pose A → B (validate command chain)
2. **SC-v13b** — pick-and-place FSM: green → grasp plain box → red (no obstacles)
3. **SC-v13c** — mid-cell wall forces a joint-space retract detour
4. **SC-v13d** — Γ (inverted-L) wall maze: retract → climb → place

**Pick object:** tennis-like MuJoCo ball (Ø 25 mm, grippy friction, density 400).
Pick sits on the same cylinder pedestal; place is a transparent non-colliding
plate over a tip-down cone funnel (`mjcf/assets/cone.obj` visual + `funnel_w*`
wall collision — MuJoCo convex-hulls meshes, so a solid mesh cone would not
catch the ball). Cone mouth / rim match the red zone disk (r=0.05) so that disk is the funnel basis; mesh is double-sided for top-down visibility. AWS RoboMaker meshes stay for denser clutter later — too large
to grasp.

---

## Joints

| Joint | Role | Hard range (Menagerie) | Usable (±10° margin) |
|---|---|---|---|
| `Joint1` (yaw about +z) | Base slew | ±180° (±π rad) | ±170° |
| `Joint2` | Shoulder | ±85.9° (±1.5 rad) | ±75.9° |
| `Joint3` | Elbow | −85.9° … +80.2° | −75.9° … +70.2° |
| `Joint4` | Wrist | −97.4° … +112.9° | −87.4° … +102.9° |
| `Gripper` / `Gripper_mimic` | Parallel jaw | slide | Fixed open for demos |

**Yaw note:** the ~90° swing in early SC-v13a clips was a chosen start/goal, not a
hardware cap. `Joint1` can slew nearly a full turn; demos keep **10°** off each
hard stop so controllers/planners never ride the limit.

**SC-v13a free-space path:** 160° yaw (±80°) at a high-reach elbow posture —
about **0.49 m** EE XY travel — leaving ~90° of unused yaw each side of the
arc (and 10° to the hard stop) for later obstacle detours (SC-v13c).

**Pedestal height:** min clear pad_z ≈ 0.0275 m → min top ≈ 0.0175 m; with a
**4×box-edge margin** (4·0.02 = 0.08 m) the pedestal top is **0.098 m**.

**SC-v13b FSM:**
`IDLE → APPROACH → DESCEND → GRASP → LIFT → MOVE → DESCEND_PLACE → RELEASE → RETREAT → DONE`
(with `FAULT` on timeout / drop). Grasp is **full MuJoCo physics**: injected
finger-pad geoms close first, then low-gain adhesion holds the free ball.
Pedestals sit at ~0.32 m fingertip radius so the arm must stretch.

**SC-v13c:** same grasp cell plus a mid-cell wall. ``MOVE_PLACE`` is planned
with ARCO RRT* (inflated wall occupancy) and tracked by ARCO
``JointSpaceMPC`` (carrot NMPC) so the arm retracts around the wall.
SC-v13b phase targets use the same joint-space MPC.

**SC-v13d:** Γ maze (vertical stem + horizontal roof overhang toward the
pick ball). The planner must back out from under the roof, climb, then place
— a simple lift-and-fly over the stem is blocked. Wall geoms use
``contype=1`` / ``conaffinity=1`` so they collide with arm collision meshes
(Menagerie default bit 1); the ball does not collide with the wall (ball
bits exclude 1).

---

## Assets

| File | Status |
|---|---|
| Menagerie `open_manipulator_x.xml` | ✅ Submodule |
| `src/fret/mjcf/omx_tabletop.xml` | ✅ Empty tabletop template |
| `src/fret/mjcf/omx_pick_place.xml` | ✅ Tabletop + ball + place cone |
| `src/fret/mjcf/omx_desk_clutter.xml` | ✅ Mid-cell wall (SC-v13c) |
| `src/fret/mjcf/omx_wall_maze.xml` | ✅ Γ stem+cap maze (SC-v13d) |
| `src/fret/mjcf/assets/cone.obj` | ✅ Place-funnel mesh primitive |
| `src/fret/mjcf/omx.py` | ✅ Merges Menagerie + template → `.generated/` |
| `src/fret/config/scenarios/omx_reach.yml` | ✅ SC-v13a joint-space A→B |
| `src/fret/config/scenarios/omx_pick_place.yml` | ✅ SC-v13b pick-and-place |
| `src/fret/config/scenarios/omx_desk_clutter.yml` | ✅ SC-v13c desk clutter |
| `src/fret/config/scenarios/omx_wall_maze.yml` | ✅ SC-v13d Γ-wall maze |
| `src/fret/control/kinematics_open_manipulator_x.py` | ✅ MuJoCo FK / numerical IK |
| `src/fret/control/pick_place_fsm.py` | ✅ Manipulation FSM |

Simulation: [mujoco.md](../mujoco.md). Roadmap: [roadmap.md](../roadmap.md).
