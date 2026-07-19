# Dubins Mobile Robot (v1.1)

> **Release:** v1.1 · **Scenario:** SC-v11 (`dubins_race`) ·
> [Release spec](../releases.md#v11--dubins-dual-robot-race)

---

## Overview

The Dubins vehicle is a **car-like mobile robot** constrained to forward motion with a
minimum turning radius. The showcase races **three TurtleBot3 Burgers** from A to B
through a warehouse maze (RRT* vs SST vs straight-line dummy), stepped concurrently.

State: `q = (x, y, θ)` in SE(2).

| Agent | Planner | Visual identity |
|---|---|---|
| Car 1 | ARCO RRT* | Blue chassis |
| Car 2 | ARCO SST | Green chassis |
| Car 3 | Straight start→goal (foil) | Grey chassis |

Agent–agent contacts are disabled in MJCF so racers may overlap; they still collide
with warehouse structures.

---

## Kinematic model

Uses ARCO directly:

| Component | ARCO symbol |
|---|---|
| Vehicle model | `arco.guidance.vehicle.DubinsVehicle` |
| Path tracking (RRT*/SST) | `arco.control.mpc.DubinsPathFollowingMPC` |
| Tracking loop (RRT*/SST) | `arco.control.mpc.MPCTrackingLoop` |
| Grey foil tracker | `arco.control.pure_pursuit.PurePursuitController` |

RRT*/SST agents track the planned polyline with path-following MPC
(contour + cruise; no track-time occupancy barriers — avoidance stays in
the planner). The grey dummy stays on Pure Pursuit with no repulsion so it
collides on the naive diagonal.

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
| `src/fret/config/controllers/dubins.yml` | Vehicle limits + MPC weights / dummy PP |
| `src/fret/scenario/dubins_race_runner.py` | Pure-Python E2E orchestrator |

Visual asset benchmark (vehicle + warehouse mesh picks): [docs/assets/dubins/README.md](../assets/dubins/README.md).

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
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins

# Headless showcase video
```bash
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/dubins --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
```

# Download CI/R2 release clips
./scripts/download_showcase.sh --scenario dubins_race --camera overview
```
