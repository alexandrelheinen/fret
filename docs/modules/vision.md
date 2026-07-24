# Vision Module

**Package:** `fret.vision`  
**Source:** `src/fret/vision/`  
**Tests:** `tests/vision/`  
**Program docs:** [vision/README.md](../vision/README.md)

> **Release:** v1.3 · See [releases.md](../releases.md).

---

## Responsibility

Detect a graspable ball in camera images and lift to a world-frame
`BallObservation` for manipulator pick-and-place. Does **not** detect the
place dispenser. Does **not** use robot proprioception in the MVP. Does
**not** serve TB3.

Distinct from `PerceptionBridgeNode` (YAML obstacle clouds).

---

## Components

### Contracts (`types`, `protocols`)

Algorithm-agnostic I/O: `CameraFrame` → `BallDetection` → `BallObservation`.
Protocols: `BallDetector`, `BallTracker`, `PoseLifter`.

### MVP implementation (OpenCV)

| Piece | Module |
| --- | --- |
| HSV blob detector | `detect.hsv_blob.HsvBlobBallDetector` |
| Table-plane lifter | `geometry.plane_lifter.TablePlanePoseLifter` |
| Factory | `factory.build_hsv_plane_pipeline()` |
| YAML | `config/vision/hsv_blob_overhead.yml` |

```python
from fret.vision import build_hsv_plane_pipeline

pipe = build_hsv_plane_pipeline()
obs = pipe.process([frame])  # BallObservation | None
```

Dependency: optional extra `vision` / included in `sim`
(`opencv-python-headless`).

### `BallVisionPipeline`

Runs detect → optional track → lift. Missing ball → `None`.

---

## Configuration

All HSV / area / circularity / plane / camera tunables live under
`src/fret/config/vision/`. No method knobs hardcoded in Python.

---

## Satisfies requirements

| Requirement | Description |
|---|---|
| FR-VIS-01–02 | Pure-Python package; `N≥1` frames |
| FR-VIS-03–04 | Selection + fixture gates (HSV MVP) |
