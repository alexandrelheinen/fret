# Algorithm selection (v1.3)

> **Release:** v1.3 · Acceptance: V13-2 / V13-3 in [releases.md](../releases.md)  
> This document **is** the selection work product. Implementation fills the
> comparison table; the **provisional primary** below is the starting choice.

---

## Goal

Track a **single graspable ball** in a tabletop / floor cell using **one or more
cameras**, producing an image-space detection that can lift to a metric
`BallObservation` when calibration and scene geometry allow.

Out of scope for the v1.3 *selection*: full pick-place, learned detectors as
**required** baseline, TB3, detecting the place dispenser.

---

## Evaluation criteria

| ID | Criterion | Weight | Notes |
| --- | --- | --- | --- |
| C1 | Centre error on fixtures (px / mm) | High | Primary gate |
| C2 | Determinism / CI friendliness | High | No GPU / large weights required |
| C3 | Works under MuJoCo lighting | High | Controlled HDR-ish renders |
| C4 | Multi-camera extensibility | Medium | Same interface for N≥1 |
| C5 | Path to real cameras (v2.x) | Medium | Classical ops transfer; ML optional later |
| C6 | Implementation size | Medium | Prefer small, testable modules |

**Fixture gate (proposed for V13-3):** median ball-centre error ≤ **3 px** on
640×480 synthetic fixtures; when plane geometry is available, world XY error
≤ **10 mm** on the table plane.

---

## Candidates

| ID | Method | Pros | Cons |
| --- | --- | --- | --- |
| **A** | **HSV colour segmentation + contour circularity** | Fast, deterministic, easy unit tests | Needs colour contrast; lighting sensitivity |
| **B** | Hough circle on edges / gradient | Radius prior helps | Weaker under texture / partial occlusion |
| **C** | Stereo triangulation (2 calibrated views) | True 3-D without plane assumption | Needs dual cameras + calibration; heavier MJCF |
| **D** | Learned detector (e.g. YOLO-class) | Robust appearance | CI weight, non-determinism, overkill for v1.3 |

---

## Provisional primary selection

**Primary (v1.3):** **Candidate A** — HSV blob + circularity score.  
**Depth / Z lift:** intersect camera ray with the **known table / floor plane**
(scenario parameter), using configured ball radius for contact offset.  
**Secondary (optional in v1.3, useful in v1.4):** Candidate **C** as a refinement
when two cameras are mounted — same `BallObservation` output type.  
**Not required for v1.3 tag:** Candidate D.

### Why A

1. Tennis-like balls in FRET cells are **high-chroma** against table/floor.
2. MuJoCo lighting is controllable → HSV thresholds live in
   `config/vision/*.yml` (configuration-over-code).
3. Matches FRET’s SDD/CI culture: pure NumPy/OpenCV-class ops, golden fixtures.
4. Interface still allows swapping in B/C/D without changing FSM consumers.

### What “selection complete” means

v1.3 is done when:

1. This table has **measured** C1 results for at least A and one runner-up.
2. Primary remains A **or** is explicitly changed here with rationale.
3. Unit tests encode the fixture gate for the chosen primary.

Until measurements land, treat **A** as the engineering default.

---

## Interface expectation (independent of algo)

```text
CameraFrame[+…]  →  detect()  →  BallDetection[+…]
                 →  lift()    →  BallObservation | None
```

Multi-camera: detections may be fused in `lift()` (stereo) or selected
(best monocular). The pipeline must accept `Sequence[CameraFrame]`.

---

## Dependencies (allowed)

| Allowed in v1.3 core | Avoid as hard requirement |
| --- | --- |
| NumPy | GPU runtimes |
| OpenCV *or* skimage (pick one in implementation PR) | Large ONNX/Torch weights in default CI |

If OpenCV is chosen, document the apt/pip pin in the vision module README /
`pyproject` optional extra `vision`.
