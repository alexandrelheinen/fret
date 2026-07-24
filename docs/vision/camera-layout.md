# Camera layout (v1.4)

> **Release:** v1.4 · Related: [architecture.md](architecture.md) ·
> [algorithm-selection.md](algorithm-selection.md)

Perception cameras (ball tracking) are **not** the same as release **overview**
encode cameras in `showcase.yml`.

---

## Design goals

1. Ball resting on table or floor is visible with high probability.
2. Extrinsics are **stable and documented** (YAML), matching MJCF.
3. Mount looks **realistic** for a lab / light-industrial cell (not a free-floating eye).
4. Layout supports the **selected** algorithm once chosen (mono or multi-view).

---

## Default mount: overhead portal

**Choice:** a rigid **portal / gantry** frame above the manipulation cell holding
one downward-facing camera on the centreline of the workspace.

```
        ____ portal beam ____
       /                      \
      Cam-A (down)             optional Cam-B (oblique)
       |                        /
       v                       /
   +--------+  arm  +--------+
   | ball?  |       | place  |
   +--------+       +--------+
```

| Property | Guidance |
| --- | --- |
| Height | Above max EE / wall obstacles used in SC-v13/14 clutter |
| FOV | Cover pick region + approach; place fixture may be partial |
| Intrinsics | Fixed; stored in `config/vision/*_portal_overhead.yml` |
| Extrinsics | `T_world_cam` from MJCF body pose; unit-tested vs model |

**Why portal over eye-in-hand (for v1.4):** simpler calibration, no motion blur
during detection-before-grasp, matches “detect then pick” FSM. Wrist cameras
may be studied later; they are not the v1.4 default.

### Shipped MJCF mounts (T14-01)

| Cell | Portal body | Cam-A | Eye (world) | `fovy` |
| --- | --- | --- | --- | --- |
| OM-X pick / clutter / maze | `vision_portal` @ `(0.273, 0, 0)` | `overhead` | `(0.273, 0, 0.96)` | 45° |
| OMY pick / clutter | `vision_portal` @ `(0.40, 0, 0)` | `overhead` | `(0.40, 0, 1.30)` | 50° |

Posts + beam + camera plate are visual-only (`contype=0`). Release encode
cameras `overview` / `follow` remain free-floating (not perception sensors).

Adapter: `fret.simulation.MujocoCameraAdapter` (MuJoCo → OpenCV optical frame).

---

## Required camera resolution

| Setting | Value | Rationale |
| --- | --- | --- |
| **Perception render** | **1280 × 720** | OM-X Ø25 mm ball ≈ 20–26 px diameter under portal FOV; 640×480 drops to ~13 px and is too fragile for HSV gates |
| Showcase encode | 1280 × 720 (`offwidth`/`offheight`) | Unchanged; independent of perception YAML |
| Fixture / web demos | may stay 640 × 480 | Synthetic / qualitative only |

Constants: `PERCEPTION_WIDTH_PX` / `PERCEPTION_HEIGHT_PX` in
`fret.simulation.mujoco_camera`.

---

## Benchmark gates (first PR — detect + lift only)

Measured on seeded OM-X / OMY pick-place cells at rest (ball on pedestal).
Oracle: MuJoCo `pick_box` body pose. Image GT: pinhole projection of that pose
through live extrinsics/intrinsics.

| Metric | Gate |
| --- | --- |
| Image centre error vs projection | ≤ **5 px** |
| World XY error (OM-X) | ≤ **15 mm** |
| World XY error (OMY) | ≤ **20 mm** |
| World Z error (plane lift) | ≤ **5 mm** |
| YAML ↔ live MJCF extrinsics / intrinsics | ≤ 1e-6 (m / rad-scale) |

CI: `tests/simulation/test_mujoco_portal_vision.py`.  
Local chart: `scripts/benchmark_mujoco_vision.py`.

**Out of scope for this PR:** feeding `BallObservation` into the FSM / replacing
`pick_xy` (T14-03).

---

## Optional second camera

If multi-view lift is selected later:

- Cam-B on the portal leg or a side post, ~30–45° toward the pick region.
- Baseline and overlap documented in the camera YAML.
- Fusion stays inside `PoseLifter` / detector implementations; scenarios only
  list camera ids.

Monocular HSV + plane remains the primary path; Cam-B is **not** required for
the first portal PR.

---

## Scenario / MJCF requirements

Each SC-v16 cell must:

1. Include portal geometry (simple boxes OK) + `<camera name="overhead">`.
2. Reference calibration YAML from the scenario file (when SC-v16 lands).
3. Keep **place / dispenser** pose as ros parameters (known).
4. Expose a MuJoCo adapter that renders RGB for listed camera names
   (`MujocoCameraAdapter` — done for Cam-A).
