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
4. Layout supports the **selected** algorithm (HSV mono + optional stereo).

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
| Intrinsics | Fixed; stored in `config/vision/cameras_*.yml` |
| Extrinsics | `T_world_cam` from MJCF body pose; unit-tested vs model |

**Why portal over eye-in-hand (for v1.4):** simpler calibration, no motion blur
during detection-before-grasp, matches “detect then pick” FSM. Wrist cameras
may be studied later; they are not the v1.4 default.

---

## Optional second camera

If stereo refinement (candidate C) is enabled:

- Cam-B on the portal leg or a side post, ~30–45° toward the pick region.
- Baseline and overlap documented in the camera YAML.
- Fusion stays inside `fret.vision`; scenarios only list camera ids.

---

## Scenario / MJCF requirements

Each SC-v16 cell must:

1. Include portal geometry (simple boxes OK) + `<camera>` elements.
2. Reference calibration YAML from the scenario file.
3. Keep **place / dispenser** pose as ros parameters (known).
4. Expose a MuJoCo adapter that renders RGB for listed camera names each tick
   (or each pick cycle).

---

## Open points (resolve during T14-01)

- Exact portal dimensions per OM-X vs OMY cell scale.
- Whether clutter walls require Cam-B for V14 acceptance or remain optional.
- Sync rate: 10–30 Hz vision vs 50 Hz control (vision may run slower).
