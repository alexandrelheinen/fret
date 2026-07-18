# OpenMANIPULATOR-X (v1.3)

> **Release:** v1.3 · **Scenarios:** SC-v13a (`omx_reach`), SC-v13b (`omx_desk_clutter`) ·
> [Release spec](../releases.md#v13--openmanipulator-x-tabletop)

---

## Overview

**ROBOTIS OpenMANIPULATOR-X** — 4 revolute DOF + parallel gripper — loaded from the
Menagerie submodule:

`third_party/robotis_mujoco_menagerie/robotis_open_manipulator_x/`

v1.3 showcase:

1. **SC-v13a** — empty tabletop, end-effector pose A → B (validate command chain)
2. **SC-v13b** — AWS desk / clutter props force a joint-space detour

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
arc (and 10° to the hard stop) for later obstacle detours (SC-v13b).

---

## Assets

| File | Status |
|---|---|
| Menagerie `open_manipulator_x.xml` | ✅ Submodule |
| `src/fret/mjcf/omx_tabletop.xml` | ✅ Empty tabletop template |
| `src/fret/mjcf/omx.py` | ✅ Merges Menagerie + template → `.generated/` (absolute meshdir) |
| `src/fret/config/scenarios/omx_reach.yml` | ✅ SC-v13a joint-space A→B |
| `src/fret/control/kinematics_open_manipulator_x.py` | ✅ MuJoCo FK / numerical IK |

Simulation: [mujoco.md](../mujoco.md). Roadmap: [roadmap.md](../roadmap.md).
