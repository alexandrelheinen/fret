# Vision Module

**Package:** `fret.vision` *(target — scaffold in v1.3)*  
**Source:** `src/fret/vision/`  
**Tests:** `tests/vision/`  
**Program docs:** [vision/README.md](../vision/README.md)

> **Release:** v1.3 pipeline · v1.4 MuJoCo integration · v1.5 pickability.  
> See [releases.md](../releases.md).

---

## Responsibility

Detect and track a graspable **ball** in camera images and lift detections to a
world-frame `BallObservation` for manipulator pick-and-place. Does **not**
detect the place dispenser (known scenario parameter). Does **not** serve TB3.

This module is distinct from `PerceptionBridgeNode` (YAML obstacle clouds for
occupancy).

---

## Components (Level 3 stubs — v1.3)

### `types` — contracts

See [interfaces.md § Vision](../interfaces.md#vision--manipulation-boundary-v13).

### `pipeline.BallVisionPipeline`

```python
class BallVisionPipeline:
    def __init__(self, config: VisionConfig) -> None: ...

    def process(
        self, frames: Sequence[CameraFrame]
    ) -> BallObservation | None:
        """Detect (+ optional track) and lift to world frame."""
        ...
```

### `detect` — algorithm plugins

Common interface; **primary** implementation is HSV + circularity
([algorithm-selection.md](../vision/algorithm-selection.md)).

### `geometry` — pixel → world

Table/floor plane intersection for monocular Z; optional stereo triangulation.

---

## Configuration

All thresholds and camera extrinsics live under `src/fret/config/vision/`
(created with the v1.3 implementation). No magic numbers in Python for tunables.

---

## ROS boundary (v1.4+)

`fret.ros.vision_bridge` (name TBD) adapts sensor messages ↔ pipeline. Core
vision code remains ROS-free.

---

## Satisfies requirements

| Requirement | Description |
|---|---|
| FR-VIS-01–… | See [requirements.md](../requirements.md#computer-vision-v13v15) |
