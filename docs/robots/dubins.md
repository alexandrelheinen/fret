# Dubins Mobile Robot (v1.1)

> **Release:** v1.1 · **Scenario:** SC-v11 (`dubins_race`) ·
> [Release spec](../releases.md#v11--dubins-dual-robot-race)

---

## Overview

The Dubins vehicle is a **car-like mobile robot** constrained to forward motion with a
minimum turning radius. v1.1 features **two robots racing** from A to B through a
warehouse structure forest (RRT* vs SST).

State: `q = (x, y, θ)` in SE(2).

| Agent | Planner | Visual identity |
|---|---|---|
| Car 1 | ARCO RRT* | Blue chassis + marker **1** |
| Car 2 | ARCO SST | Green chassis + marker **2** |

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

Warehouse floor (AWS RoboMaker ground texture) with **rectangular structures** and
**U-shaped dead-end alcoves**. Multiple corridors remain open from corner A to corner B,
so RRT* and SST can choose different routes.

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
| `src/fret/config/worlds/dubins_race_obstacles.yml` | Structure forest layout |
| `src/fret/config/controllers/dubins.yml` | Pure Pursuit / vehicle gains |
| `src/fret/scenario/dubins_race_runner.py` | Pure-Python E2E orchestrator |

Regenerate AWS warehouse meshes when needed:

```bash
python3 scripts/import_aws_warehouse_assets.py
```

---

## Running

```bash
# Pure-Python E2E (CI)
pytest tests/scenario/test_dubins_race_e2e.py -v

# Quick orchestrator smoke test
python3 -c "from fret.scenario import DubinsRaceRunner; print(DubinsRaceRunner().run())"

# MuJoCo SITL
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins backend:=mujoco

# Headless showcase video
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras -o /tmp/dubins

# Download CI/R2 release clips
./scripts/download_showcase.sh --scenario dubins_race --camera overview
```
