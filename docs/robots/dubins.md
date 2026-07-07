# Dubins Mobile Robot (v1.1)

> **Release:** v1.1 · **Scenario:** SC-v11 (`dubins_race`) ·
> [Release spec](../releases.md#v11--dubins-dual-robot-race)

---

## Overview

The Dubins vehicle is a **car-like mobile robot** constrained to forward motion with a
minimum turning radius. v1.1 features **two robots racing** from A to B through a
column forest.

State: `q = (x, y, θ)` in SE(2).

---

## Kinematic model

Uses ARCO directly:

| Component | ARCO symbol |
|---|---|
| Vehicle model | `arco.guidance.vehicle.DubinsVehicle` |
| Steering primitive | `arco.guidance.primitive.dubins.DubinsPrimitive` |
| Path tracking | `arco.guidance.control.pure_pursuit.PurePursuitController` |

---

## Environment

Column forest with **varied column heights** for visual depth in MuJoCo. Layout inspired
by ARCO `map/vehicle.yml` (wall gaps, scattered obstacles) extended to 3-D posts.

---

## Assets (planned)

| File | Purpose |
|---|---|
| `src/fret/mjcf/dubins_race.xml` | MuJoCo race world |
| `src/fret/config/scenarios/dubins_race.yml` | SC-v11 scenario |
