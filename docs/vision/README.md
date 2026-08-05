# Computer vision program

> **Release criteria:** [releases.md](../releases.md) · **Roadmap:** [roadmap.md](../roadmap.md)

FRET’s computer-vision work lives in **simulation** through the current v1.x
line. Hardware cameras and real images belong to the **v2.x** hardware era.

---

## Why vision exists here

Manipulators **grasp scene objects**. OM-X / OMY pick-place takes the ball pose
from gate-camera `BallObservation` (SC-v16). YAML `pick_xy` remains a **test
oracle** only; place / dispenser stays parametric.

**TurtleBot3 / Dubins does not use CV.** It remains an ARCO → MuJoCo mobility
case study (no graspable object interaction in the product scenarios).

---

## Program ladder

| Stage | Deliverable | Doc | Status |
| --- | --- | --- | --- |
| Pipeline | Unit tests + **algorithm selection** | [algorithm-selection.md](algorithm-selection.md) | ✅ |
| Integration | MuJoCo cameras → pick-place (no hardcoded ball) | [camera-layout.md](camera-layout.md) · [architecture.md](architecture.md) | ✅ |
| Dynamic scene | Rolling ball, pickability, industrial container | [releases.md § v1.5](../releases.md#v15--dynamic-ball--industrial-place) | 🔲 |

---

## Locked product decisions

| Decision | Choice | Rationale |
| --- | --- | --- |
| Who uses CV | OM-X and OMY only | Only manipulators interact with graspable objects |
| What CV must find | Ball centre (and pickability on the dynamic line) | Grasp entry needs object pose |
| What stays parametric | Place / dispenser / container pose | Fixed industrial fixture |
| Algorithm | **HSV blob + table-plane lift** (MVP) | See [algorithm-selection.md](algorithm-selection.md); contracts stay swappable |
| Multi-camera | Interface supports `N ≥ 1` frames | Fusion is an implementation detail |
| Dynamic cadence | One ball per cycle (provisional) | Clearer FSM and metrics; multi-ball stretch |

---

## Package layout

```
src/fret/vision/          # pure Python — no rclpy
  types.py                # CameraFrame, BallObservation, VisionConfig, …
  protocols.py            # BallDetector, BallTracker, PoseLifter
  pipeline.py             # BallVisionPipeline
  detect/                 # detector implementations
  geometry/               # pose-lift implementations
  track/                  # trackers

src/fret/config/vision/   # calibration / scenario wiring

src/fret/ros/             # thin adapters only
```

Module API: [modules/vision.md](../modules/vision.md).

---

## Relationship to existing “perception”

`PerceptionBridgeNode` (`fret.ros.perception_bridge`) publishes a **synthetic
obstacle point cloud** from YAML boxes/cylinders for occupancy. It is **not**
the ball CV pipeline. Naming:

| Component | Role |
| --- | --- |
| `perception_bridge` | Occupancy obstacles → `/obstacle_cloud` |
| `fret.vision` | Ball detection / tracking → `BallObservation` |

Do not overload `perception_bridge` with ball tracking; keep contracts separate.
