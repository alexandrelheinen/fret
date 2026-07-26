# OpenMANIPULATOR-X (v1.2.3)

> **Release:** v1.2.3 · **Scenarios:** SC-v13a (`omx_reach`), SC-v13b (`omx_pick_place`),
> SC-v13c (`omx_desk_clutter`), SC-v13d (`omx_wall_maze`) ·
> **CV integration:** from v1.4 ([vision/README.md](../vision/README.md)) ·
> [Release spec](../releases.md#v123--openmanipulator-x-tabletop)

---

## Overview

**ROBOTIS OpenMANIPULATOR-X** — 4 revolute DOF + parallel gripper — loaded from the
Menagerie submodule:

`third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x/`

v1.2.3 showcase:

1. **SC-v13a** — empty tabletop, end-effector pose A → B (validate command chain)
2. **SC-v13b** — pick-and-place FSM: green → grasp plain box → red (no obstacles)
3. **SC-v13c** — mid-chord wall + front place cone forces a retract detour
4. **SC-v13d** — dual Γ wall maze (front place): retract → climb → place

**Pick object:** tennis-like MuJoCo ball (Ø 25 mm, grippy friction, density 400).
Pick rests on the floor / table plane; place is a transparent non-colliding
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
25 mm ball, the pick pose sits ~0.04 m above the tabletop.

**SC-v13b FSM** (robot-agnostic; shared with OMY):

Logical cycle — see [pick_place_fsm.md](../modules/pick_place_fsm.md) (Mermaid):

`IDLE → (ball) PLAN/MOVE_PICK → GRASP → PLAN/MOVE_PLACE → RELEASE → PLAN/MOVE_HOME → IDLE`

Implementation substates:

`IDLE → APPROACH_PICK → DESCEND_PICK → GRASP → LIFT → MOVE_PLACE → DESCEND_PLACE → RELEASE → RETREAT → DONE`

**SC-v13c:** floor pick + front place cone plus a mid-chord wall. ``MOVE_PLACE`` is planned
around the wall (not a straight joint-space line). Grasp / lift / release
SC-v13b phase targets use the same joint-space MPC.

**SC-v13d:** dual Γ maze (stems flank the front place corridor; caps overhang ±Y toward the
pick ball) — forces back-out from under the roof, climb, then place.

---

## Assets

| File | Status |
|---|---|
| `src/fret/mjcf/omx_tabletop.xml` | ✅ Empty cell (SC-v13a) |
| `src/fret/mjcf/omx_pick_place.xml` | ✅ Ball + cone (SC-v13b) |
| `src/fret/mjcf/omx_desk_clutter.xml` | ✅ Mid-cell wall (SC-v13c) |
| `src/fret/mjcf/omx_wall_maze.xml` | ✅ Γ stem+cap maze (SC-v13d) |
| `src/fret/control/kinematics_open_manipulator_x.py` | ✅ Menagerie FK |
| `src/fret/control/pick_place_fsm.py` | ✅ SC-v13b FSM |
| `src/fret/config/scenarios/omx_reach.yml` | ✅ SC-v13a joint-space A→B |
| `src/fret/config/scenarios/omx_pick_place.yml` | ✅ SC-v13b pick-and-place |
| `src/fret/config/scenarios/omx_desk_clutter.yml` | ✅ SC-v13c desk clutter |
| `src/fret/config/scenarios/omx_wall_maze.yml` | ✅ SC-v13d Γ-wall maze |

Simulation: [mujoco.md](../mujoco.md). Roadmap: [roadmap.md](../roadmap.md).
