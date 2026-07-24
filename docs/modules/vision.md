# Vision Module

**Package:** `fret.vision`  
**Source:** `src/fret/vision/`  
**Tests:** `tests/vision/`  
**Program docs:** [vision/README.md](../vision/README.md)

> **Release:** v1.3 pipeline scaffold · v1.4 MuJoCo integration · v1.5
> pickability. See [releases.md](../releases.md).

---

## Responsibility

Expose **algorithm-agnostic** contracts so manipulator pick-and-place can
consume a world-frame ball observation. Does **not** detect the place
dispenser (known scenario parameter). Does **not** serve TB3.

This module is distinct from `PerceptionBridgeNode` (YAML obstacle clouds for
occupancy).

**No preferred detector / tracker / lifter is encoded in this package.**
Algorithm selection is an open v1.3 work product
([algorithm-selection.md](../vision/algorithm-selection.md)).

---

## Components

### `types` — I/O contracts

`CameraFrame`, `BallDetection`, `BallObservation`, `PlaceTarget`,
`CameraIntrinsics`, `CameraExtrinsics`, `VisionConfig`.

See [interfaces.md § Vision](../interfaces.md#vision--manipulation-boundary-v13).

### `protocols` — stage interfaces

| Protocol | Role |
| --- | --- |
| `BallDetector` | `frames → list[BallDetection]` |
| `BallTracker` | optional temporal filter on detections |
| `PoseLifter` | `detections (+ frames) → BallObservation \| None` |

### `pipeline.BallVisionPipeline`

Constructor accepts injected detector + lifter (+ optional tracker) and
shared `VisionConfig` (calibration only). `process()` is a Level-3 stub
(`NotImplementedError`) until orchestration is implemented.

```python
pipe = BallVisionPipeline(detector, lifter, config=cfg)
obs = pipe.process(frames)  # BallObservation | None
```

### `detect/` · `geometry/` · `track/`

Empty package markers for future implementations. No concrete algorithms yet.

---

## Configuration

`src/fret/config/vision/` holds algorithm-agnostic camera / scenario wiring
once scenarios need it. Method-specific tunables stay with the chosen
implementation’s YAML after selection.

---

## ROS boundary (v1.4+)

A thin `fret.ros` adapter may publish/subscribe images and observations.
Core vision code remains ROS-free.

---

## Satisfies requirements

| Requirement | Description |
|---|---|
| FR-VIS-01 | Pure-Python package; no ROS in core |
| FR-VIS-02 | Pipeline API accepts `N ≥ 1` frames |
| FR-VIS-03–04 | Selection + fixture gates (later in v1.3) |
