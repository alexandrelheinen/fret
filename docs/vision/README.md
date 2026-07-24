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
| What stays parametric | Place / dispenser / container pose | Fixed industrial fixture |
| Algorithm | **Open** — see [algorithm-selection.md](algorithm-selection.md) | Scaffold API is algo-agnostic |
| Multi-camera | Interface supports `N ≥ 1` frames | Fusion is an implementation detail |
| v1.5 cadence | One ball per cycle (provisional) | Clearer FSM and metrics; multi-ball stretch |

---

## Package layout

```
src/fret/vision/          # pure Python — no rclpy
  types.py                # CameraFrame, BallObservation, VisionConfig, …
  protocols.py            # BallDetector, BallTracker, PoseLifter
  pipeline.py             # BallVisionPipeline (Level-3 stub on process)
  detect/                 # future detector implementations
  geometry/               # future pose-lift implementations
  track/                  # future trackers

src/fret/config/vision/   # calibration / scenario wiring (algo knobs later)

src/fret/ros/             # thin adapters only (v1.4+)
```

Module API: [modules/vision.md](../modules/vision.md).

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
