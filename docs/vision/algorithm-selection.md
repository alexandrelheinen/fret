# Algorithm selection (v1.3)

> **Release:** v1.3 · Acceptance: V13-2 / V13-3 in [releases.md](../releases.md)  
> This document is the **selection work product**. It must be filled during
> v1.3; the scaffold (`fret.vision`) deliberately does **not** encode a choice.

---

## Goal

Track a **single graspable ball** in a tabletop / floor cell using **one or more
cameras**, producing an image-space detection that can lift to a metric
`BallObservation` when calibration and scene geometry allow.

Out of scope for the v1.3 *selection*: full pick-place, TB3, detecting the
place dispenser. The stable API is independent of which algorithm wins:
[architecture.md](architecture.md), [modules/vision.md](../modules/vision.md).

---

## Evaluation criteria

| ID | Criterion | Weight | Notes |
| --- | --- | --- | --- |
| C1 | Centre error on fixtures (px / mm) | High | Primary gate |
| C2 | Determinism / CI friendliness | High | Prefer no GPU / large weights in default CI |
| C3 | Works under MuJoCo lighting | High | Controlled sim renders |
| C4 | Multi-camera extensibility | Medium | Same `BallDetector` / `PoseLifter` shapes |
| C5 | Path to real cameras (v2.x) | Medium | Same contracts |
| C6 | Implementation size | Medium | Prefer small, testable modules |

**Fixture gate (proposed for V13-3):** median ball-centre error ≤ **3 px** on
640×480 synthetic fixtures; when plane geometry is available, world XY error
≤ **10 mm** on the table plane. Thresholds may be revised when selection runs.

---

## Candidates (discussion list — not ranked)

Examples to evaluate; none is selected by the scaffold:

| ID | Method family | Notes |
| --- | --- | --- |
| A | Colour segmentation + shape score | Classic blob track |
| B | Edge / circle fitting | Radius prior in image |
| C | Multi-view geometric lift | Needs ≥2 calibrated cameras |
| D | Learned detector | Heavier deps; optional stretch |

Add / remove rows as the discussion proceeds. Record measured C1–C6 here
before locking a primary.

---

## Selection status

**Status: open.** No primary algorithm is locked.

When selection completes, this section must record:

1. Chosen primary (and optional secondary) with rationale against C1–C6.
2. Measured fixture metrics.
3. Pointer to the implementing modules under `fret.vision.detect` /
   `geometry` / `track`.

Until then, `BallVisionPipeline` accepts any objects matching
`BallDetector` / `PoseLifter` / `BallTracker`.

---

## Interface expectation (independent of algo)

```text
Sequence[CameraFrame]
  → BallDetector.detect()   → list[BallDetection]
  → BallTracker.update()    → list[BallDetection]   (optional)
  → PoseLifter.lift()       → BallObservation | None
```

Multi-camera: the detector and lifter receive the full frame sequence; fusion
strategy is an implementation detail behind those protocols.

---

## Dependencies

Dependency choices (OpenCV vs skimage vs other) are part of algorithm
selection and are **not** pinned by the scaffold. Optional extras may be added
to `pyproject.toml` when an implementation lands.
