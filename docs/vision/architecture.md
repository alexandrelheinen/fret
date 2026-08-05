# Vision architecture

> Level-2 design for the computer-vision line. Contracts also appear in
> [interfaces.md](../interfaces.md). Module API: [modules/vision.md](../modules/vision.md).

---

## Context

Pick-and-place with vision:

```
MuJoCo cameras → CameraAdapter → BallVisionPipeline → BallObservation
                                                      ↓
                         PickPlaceFSM / IK entry (ball pose)
place / dispenser ← scenario YAML (known parameter)
```

The pipeline also runs against fixtures for unit gates. The open dynamic-scene
line (v1.5) adds pickability and rolling-ball delivery upstream of the same pipe.

FSM charts: [modules/pick_place_fsm.md](../modules/pick_place_fsm.md).

---

## Layering

```
┌─────────────────────────────────────────────────────────────┐
│  Scenario / SITL (manipulators only)                        │
│  PickPlaceFSM · planners · JointSpaceMPC · MuJoCo actuators │
└──────────────▲──────────────────────────────────────────────┘
               │ BallObservation (world frame)
┌──────────────┴──────────────────────────────────────────────┐
│  fret.ros.vision_bridge   (thin)                            │
│  topics: Image(s) in → BallObservation / diagnostics out    │
└──────────────▲──────────────────────────────────────────────┘
               │ CameraFrame(s)
┌──────────────┴──────────────────────────────────────────────┐
│  fret.vision  (pure Python)                                 │
│  BallDetector → (BallTracker) → PoseLifter                  │
│  via BallVisionPipeline  →  BallObservation | None          │
└──────────────▲──────────────────────────────────────────────┘
               │ RGB (+ optional depth later)
┌──────────────┴──────────────────────────────────────────────┐
│  Camera sources                                             │
│  fixtures · MuJoCo <camera> · (future) real hardware        │
└─────────────────────────────────────────────────────────────┘
```

**Rules:**

1. `fret.vision` must not import `rclpy` or MuJoCo.
2. Camera drivers / MuJoCo renderers live in adapters (`fret.ros`,
   `fret.simulation`, or scripts).
3. Occupancy perception (`perception_bridge`) stays a separate path.
4. TB3 / Dubins never subscribe to ball vision outputs.

---

## Data contracts (summary)

Full dataclasses: [interfaces.md § Vision](../interfaces.md#vision--manipulation-boundary-v13).

| Type | Meaning |
| --- | --- |
| `CameraFrame` | Image + camera_id + timestamp + intrinsics ref |
| `BallDetection` | Image-space centre, radius_px, confidence, camera_id |
| `BallObservation` | World-frame position (± covariance), pickable flag, timestamp |
| `PlaceTarget` | Known dispenser / container pose from scenario YAML — **not** from CV |

---

## Control integration

1. On each vision tick (or once per pick cycle), obtain `BallObservation`.
2. If missing / low confidence → FSM stays IDLE or FAULT policy (scenario YAML).
3. If present → compute grasp approach from ball position + known ball radius
   (scenario or detection) via existing IK / pad-mid helpers.
4. `place_*` configurations continue to come from YAML (fixed fixture).

Hardcoded `pick_xy` / offline grasp joints used as **ground truth for the ball**
are forbidden on SC-v16 release paths. They may remain as **test oracles**
(compare vision vs MuJoCo `qpos`) in CI.

---

## Camera mount

Default design: **rear structural gate** with dual corner cameras so the ball
rests in view with stable lighting. Optional additional views if the selected
algorithm needs stereo or occlusion recovery.

Detail and MJCF conventions: [camera-layout.md](camera-layout.md).

---

## Failure modes

| Failure | Policy (default) |
| --- | --- |
| No detection | Do not grasp; log; optional replan wait |
| Multiple blobs | Prefer highest circularity × area score; emit diagnostics |
| Outside workspace | `pickable=False` (required on the dynamic-scene line) |
| Place fixture missing in YAML | Fail at scenario load (config policy) |
