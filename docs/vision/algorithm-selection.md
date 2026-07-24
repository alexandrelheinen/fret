# Algorithm selection (v1.3)

> **Release:** v1.3 · Acceptance: V13-2 / V13-3 in [releases.md](../releases.md)

---

## Goal

Track a **single graspable ball** with **one or more** cameras, producing a
metric `BallObservation` when calibration and scene geometry allow.

---

## Selection status (MVP)

**Primary (implemented):** HSV colour blob + contour circularity
(`HsvBlobBallDetector`) with **table-plane ray lift**
(`TablePlanePoseLifter`), single overhead camera.

| Item | Choice |
| --- | --- |
| Cameras | 1× overhead (portal later in v1.4 MJCF) |
| Robot proprioception | Not used in vision |
| Library | OpenCV (`opencv-python-headless`) |
| Config | `config/vision/hsv_blob_overhead.yml` |
| Factory | `fret.vision.build_hsv_plane_pipeline()` |

Public contracts (`CameraFrame`, protocols, `BallVisionPipeline`) remain
usable with other detectors/lifters.

---

## Evaluation criteria

| ID | Criterion | Weight |
| --- | --- | --- |
| C1 | Centre error on fixtures (px / mm) | High |
| C2 | Determinism / CI friendliness | High |
| C3 | Works under MuJoCo lighting | High |
| C4 | Multi-camera extensibility | Medium |
| C5 | Path to real cameras (v2.x) | Medium |
| C6 | Implementation size | Medium |

**Fixture gate (MVP tests):** synthetic orange disk — centre error ≤ **3 px**;
nadir / look-at lift XY error ≤ **~30 mm** on the unit test geometry.

---

## Candidates (historical discussion)

| ID | Method | Status |
| --- | --- | --- |
| A | Colour segmentation + shape score | **Selected for MVP** |
| B | Edge / circle fitting | Not implemented |
| C | Multi-view geometric lift | Deferred (API already `N≥1`) |
| D | Learned detector | Deferred |

---

## Interface (independent of algo)

```text
Sequence[CameraFrame]
  → BallDetector.detect()   → list[BallDetection]
  → BallTracker.update()    → list[BallDetection]   (optional; unused in MVP)
  → PoseLifter.lift()       → BallObservation | None
```
