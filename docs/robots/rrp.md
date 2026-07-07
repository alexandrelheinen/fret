# RRP / SCARA Robot (v1.2)

> **Release:** v1.2 · **Scenarios:** SC-v12a, SC-v12b ·
> [Release spec](../releases.md#v12--rrp--scara-arco-reproduction)

---

## Overview

The RRP robot is a **3-DOF SCARA-like arm**: two revolute joints in the horizontal plane
plus one vertical prismatic joint. FRET already implements this topology as the bootstrap
`scara` model (MS-1–5).

v1.2 aligns FRET with ARCO `map/rrp.yml` and `map/rr.yml` scenarios.

---

## Kinematic model (RRP)

| Joint | Type | Symbol |
|---|---|---|
| `joint_arm_0` | Revolute | θ₁ |
| `joint_arm_1` | Revolute | θ₂ |
| `joint_extension` | Prismatic | z |

Link lengths (ARCO `rrp.yml`): `l1 = 1.0 m`, `l2 = 0.8 m`, `z ∈ [0, 4] m`.

See `src/fret/control/kinematics.py` and `src/fret/urdf/scara.xacro`.

---

## ARCO scenarios to reproduce

| ARCO map | FRET scenario | Description |
|---|---|---|
| `rrp.yml` | `rrp_pillars.yml` | Pillars + slabs, 3-D C-space |
| `rr.yml` | `rr_planar.yml` | 2-D planar arm, box obstacles |

---

## Assets (existing)

| File | Status |
|---|---|
| `src/fret/urdf/scara.xacro` | ✅ Bootstrap |
| `src/fret/worlds/pillar_scenario.sdf` | ✅ Bootstrap |
| `src/fret/config/scenarios/pillar_avoidance.yml` | ✅ → migrate to `rrp_pillars.yml` |

Reference photos: `docs/robots/scara_*.jpg`.
