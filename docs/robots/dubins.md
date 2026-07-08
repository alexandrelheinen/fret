# Dubins Mobile Robot (v1.1)

> **Release:** v1.1 · **Scenario:** SC-v11 (`dubins_race`) ·
> [Release spec](../releases.md#v11--dubins-dual-robot-race) ·
> [Implementation guide](../v1.1-dubins-race.md)

---

## Overview

The Dubins vehicle is a **car-like mobile robot** constrained to forward motion with a
minimum turning radius. v1.1 features **two robots racing** from A to B through a
warehouse column forest (RRT* vs SST).

State: `q = (x, y, θ)` in SE(2).

---

## Kinematic model

Uses ARCO directly:

| Component | ARCO symbol |
|---|---|
| Vehicle model | `arco.guidance.vehicle.DubinsVehicle` |
| Path tracking | `arco.control.pure_pursuit.PurePursuitController` |
| Tracking loop | `arco.control.tracking.TrackingLoop` |

FRET adapter: `fret.control.kinematics_dubins.DubinsKinematics`

---

## Environment

Warehouse floor (AWS RoboMaker ground texture) with **staggered columns** of varied
height. Unlike the ARCO `vehicle` maze, multiple corridors remain open from corner A
to corner B.

---

## Visual identity

| Agent | Planner | MJCF material | Marker |
|---|---|---|---|
| 1 | RRT* | `car_rrt` (blue) | White roof sphere |
| 2 | SST | `car_sst` (green) | Dark roof sphere |

Colors match `arco.config.palette.layer_rgb("rrt"|"sst", "vehicle")`.

---

## Assets

| File | Purpose |
|---|---|
| `src/fret/mjcf/dubins_race.xml` | MuJoCo race world + dual agents |
| `src/fret/config/scenarios/dubins_race.yml` | SC-v11 scenario |
| `src/fret/config/worlds/dubins_race_obstacles.yml` | Column forest layout |
| `src/fret/config/controllers/dubins.yml` | Pure Pursuit / vehicle gains |
| `src/fret/scenario/dubins_race_runner.py` | Pure-Python E2E orchestrator |

---

## Running

```bash
pytest tests/scenario/test_dubins_race_e2e.py -v
python3 -c "from fret.scenario import DubinsRaceRunner; print(DubinsRaceRunner().run())"
```
