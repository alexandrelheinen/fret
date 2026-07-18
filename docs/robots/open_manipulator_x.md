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

| Joint | Role |
|---|---|
| `Joint1`…`Joint4` | Arm (planning DOF) |
| `Gripper` / `Gripper_mimic` | Fixed open for positional demos |

---

## Assets

| File | Status |
|---|---|
| Menagerie `open_manipulator_x.xml` | ✅ Submodule |
| `src/fret/mjcf/omx_tabletop.xml` | 🔲 v1.3 |
| `src/fret/config/scenarios/omx_reach.yml` | 🔲 v1.3 |

Simulation: [mujoco.md](../mujoco.md). Roadmap: [roadmap.md](../roadmap.md).
