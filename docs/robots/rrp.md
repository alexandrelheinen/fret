# RRP / SCARA Robot (v1.3)

> **Release:** v1.3 · **Scenarios:** SC-v13a, SC-v13b ·
> [Release spec](../releases.md#v13--rrp--scara-arco-reproduction)

---

## Overview

The RRP robot is a **3-DOF SCARA-like arm**: two revolute joints in the horizontal plane
plus one vertical prismatic joint. FRET already implements this topology as the bootstrap
`scara` model (MS-1–5).

v1.3 aligns FRET with ARCO `map/rrp.yml` and `map/rr.yml` scenarios on **MuJoCo
physics SITL** (v1.2 foundation).

---

## Kinematic model (RRP)

| Joint | Type | Symbol |
|---|---|---|
| `joint_arm_0` | Revolute | θ₁ |
| `joint_arm_1` | Revolute | θ₂ |
| `joint_extension` | Prismatic | z |

Link lengths (ARCO `rrp.yml`): `l1 = 1.0 m`, `l2 = 0.8 m`, `z ∈ [0, 4] m`.

See `src/fret/control/kinematics.py`.

---

## ARCO scenarios to reproduce

| ARCO map | FRET scenario | Description |
|---|---|---|
| `rrp.yml` | `rrp_pillars.yml` | Pillars + slabs, 3-D C-space |
| `rr.yml` | `rr_planar.yml` | 2-D planar arm, box obstacles |

---

## Assets (existing / planned)

| File | Status |
|---|---|
| `src/fret/control/kinematics.py` | ✅ Bootstrap |
| `src/fret/config/scenarios/pillar_avoidance.yml` | ✅ → migrate to `rrp_pillars.yml` |
| `src/fret/mjcf/rrp_pillars.xml` | 🔲 v1.3 |

Reference photos: `docs/robots/scara_*.jpg`.

Simulation: [mujoco.md](../mujoco.md).
