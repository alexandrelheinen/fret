# Computer vision program (v1.3–v1.5)

> **Era:** Simulation & algorithms (v1.x only).  
> **Release criteria:** [releases.md](../releases.md) · **Roadmap:** [roadmap.md](../roadmap.md)

FRET’s computer-vision work lives entirely in **simulation** through **v1.5**.
Hardware cameras and real images belong to **v2.x**.

---

## Why vision exists here

Manipulators **grasp scene objects**. Today’s OM-X / OMY scenarios hardcode ball
poses (`pick_xy`, joint grasp configs derived offline). The CV line replaces
that cheat with a measured ball pose while keeping planning, FSM, and MPC.

**TurtleBot3 / Dubins does not use CV.** It remains an ARCO → MuJoCo mobility
case study (no graspable object interaction in the product scenarios).

---

## Release ladder

| Tag | Deliverable | Doc |
| --- | --- | --- |
| **v1.3** | Pipeline + unit tests + **algorithm selection** | [algorithm-selection.md](algorithm-selection.md) |
| **v1.4** | MuJoCo cameras + wire into pick-place (no hardcoded ball) | [camera-layout.md](camera-layout.md) · [architecture.md](architecture.md) |
| **v1.5** | Rolling ball, pickability, industrial container | [releases.md § v1.5](../releases.md#v15--dynamic-ball--industrial-place) |

---

## Locked product decisions

| Decision | Choice | Rationale |
| --- | --- | --- |
| Who uses CV | OM-X and OMY only | Only manipulators interact with graspable objects |
| What CV must find | Ball centre (and pickability from v1.5) | Grasp entry needs object pose |
| What stays parametric | Place / dispenser / container pose | Fixed industrial fixture; no need to detect every cycle |
| Primary algo (v1.3) | HSV blob + circularity + table-plane Z | Deterministic, unit-testable, no heavy ML deps in CI |
| Multi-camera | Interface required in v1.3; dual overhead+side optional in v1.4 | Occlusion robustness without blocking mono baseline |
| v1.5 cadence | One ball per cycle (provisional) | Clearer FSM and metrics; multi-ball is stretch |

---

## Package layout (target)

```
src/fret/vision/          # pure Python — no rclpy
  types.py                # CameraFrame, BallObservation, …
  detect/                 # selected + candidate detectors
  track/                  # temporal association (optional in v1.3)
  geometry/               # pixel → world (plane / stereo)
  pipeline.py             # BallVisionPipeline

src/fret/ros/             # thin adapters only (v1.4+)
  vision_bridge.py        # images in → BallObservation out (topic)

src/fret/config/vision/   # HSV ranges, camera extrinsics, thresholds
```

Module API spec: [modules/vision.md](../modules/vision.md).

---

## Relationship to existing “perception”

`PerceptionBridgeNode` (`fret.ros.perception_bridge`) publishes a **synthetic
obstacle point cloud** from YAML boxes/cylinders for occupancy. It is **not**
the ball CV pipeline. Naming:

| Component | Role |
| --- | --- |
| `perception_bridge` | Occupancy obstacles → `/obstacle_cloud` (existing) |
| `fret.vision` | Ball detection / tracking → `BallObservation` (new) |

Do not overload `perception_bridge` with ball tracking; keep contracts separate.
