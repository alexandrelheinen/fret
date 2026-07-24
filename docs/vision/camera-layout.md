# Camera layout (v1.4)

> **Release:** v1.4 · Related: [architecture.md](architecture.md) ·
> [algorithm-selection.md](algorithm-selection.md)

Perception cameras (ball tracking) are **not** the same as release **overview**
encode cameras in `showcase.yml`.

---

## Design goals

1. Ball resting on the pick pedestal (any side of the cell, ±90° yaw) is visible
   from at least one gate camera with high probability.
2. Extrinsics are **stable and documented** (YAML), matching MJCF.
3. Mount looks **realistic** for a lab / light-industrial cell (structural gate,
   not a free-floating eye).
4. Place / dispenser sits **in front of the arm** (+X); the arm stretches forward
   to drop. Pick is on the side so future “any-side pick” stays plausible.

---

## Default mount: rear structural gate (dual Cam-A/B)

**Choice:** a rigid **gate** behind the manipulator (negative X), with square
tube posts, top/bottom beams, and an **X-brace**, holding two cameras at the
**top corners**.

```
          gate (behind base)
     Cam-L *====X====* Cam-R     ← toed-in toward workspace
           ||  / \  ||
           || /   \ ||
           ||/     \||
   -Y ←─── base / arm ───→ +Y
                  |
                  v  +X forward
            [ place cone ]
         (pick pedestal on −Y)
```

| Property | OM-X | OMY |
| --- | --- | --- |
| Gate plane | `x = −0.16 m` | `x = −0.22 m` |
| Half-width (post Y) | ±0.30 m | ±0.42 m |
| Top height | 0.72 m | 1.00 m |
| Tube half-size (MuJoCo) | 12 mm → **24 mm** bar | 16 mm → **32 mm** bar |
| Cameras | `gate_cam_left`, `gate_cam_right` | same names |
| `fovy` | 42° | 45° |
| Look-at (shared) | ~(0.22, −0.08, 0.08) | ~(0.38, −0.10, 0.10) |
| Pick (side) | `(0.22, −0.20)` | `(0.35, −0.30)` |
| Place (front) | `(0.26, 0.0)` | `(0.48, 0.0)` |

Gate geoms are visual-only (`contype=0`). The gate may clip a little rear
workspace, but leaves margin for side picks at ±90° yaw and a forward place
reach.

Adapter: `fret.simulation.MujocoCameraAdapter` (per camera name).  
Fusion: confidence-weighted mean of table-plane lifts; **one view is enough**
if the other is obstructed.

---

## Required camera resolution

| Setting | Value | Rationale |
| --- | --- | --- |
| **Perception render** | **1280 × 720** | Small OM-X ball still ≥ ~10 px under gate FOV |
| Showcase encode | 1280 × 720 | Unchanged |

Constants: `PERCEPTION_WIDTH_PX` / `PERCEPTION_HEIGHT_PX` in
`fret.simulation.mujoco_camera`.

---

## Benchmark gates (detect + lift; no FSM feed)

| Metric | Gate |
| --- | --- |
| Image centre error (per hitting view) | ≤ **5 px** |
| World XY (fused or mono) OM-X / OMY | ≤ **15 mm** / **20 mm** |
| World Z | ≤ **5 mm** |
| YAML ↔ live MJCF calib | ≤ ~1e-5 |

CI: `tests/simulation/test_mujoco_portal_vision.py`.

---

## Parallel vs oblique (toed-in) camera axes

**Shipped choice: mildly oblique / toed-in** — both cameras share a look-at near
the pick/place workspace rather than parallel optical axes.

| Option | Pros | Cons |
| --- | --- | --- |
| **Parallel axes** | Simple calib story; uniform ground sampling; easy stereo baseline math | Outer FOV wastes pixels on floor behind the gate; ball near side pick may leave one view |
| **Toed-in (oblique)** | Both FOVs cover the working volume; better single-view fallback when one cam is blocked; matches “gate corner” industrial mounts | Baseline not purely lateral; stereo triangulation needs care (we use **plane lift + fuse**, not pure stereo) |

For **HSV + table-plane lift**, parallel vs toed-in barely changes precision when
both see the ball (errors already ≪ 1 mm). The practical win of toed-in is
**coverage and obstruction robustness**, not millimetres. Pure stereo depth
without a plane would prefer a known baseline and limited toe-in; that is not
the v1.4 primary path.

**Recommendation:** keep toed-in for this gate; revisit parallel only if a
future stereo lifter needs a calibrated rectified pair.

---

## Scenario / MJCF requirements

Each SC-v16 cell must:

1. Include `vision_gate` + `gate_cam_left` / `gate_cam_right`.
2. Reference dual-camera YAML (`omx_portal_overhead.yml` / `omy_portal_overhead.yml`).
3. Keep **place / dispenser** as known scenario parameters (front cone for now).
4. Expose RGB via `MujocoCameraAdapter` per camera id.
