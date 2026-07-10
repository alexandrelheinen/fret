# FretSim — Simulation Guide

> **MuJoCo integration spec:** [mujoco.md](mujoco.md)  
> **Visual tutorial:** [tutorial.md](tutorial.md)  
> **Release focus:** v1.0 PPP · v1.1 Dubins · v1.2 physics SITL. See [releases.md](releases.md).

---

## Modes

| Mode | Requirements | Purpose |
|---|---|---|
| **Pure-Python** | Python 3.12+, numpy | Unit tests, algorithm validation |
| **MuJoCo viewer** | `mujoco` | **Live 3D window** — `./scripts/view.sh` (all flags required) |
| **MuJoCo MP4** | `mujoco`, `imageio` | Headless showcase video |
| **MuJoCo SITL** | `mujoco`, ROS 2 Jazzy | Full ROS pipeline via `sitl.py` |
| **MuJoCo physics SITL** | `mujoco`, ROS 2 Jazzy | Actuator-driven `mj_step` (v1.2+) |

MuJoCo is FRET's sole simulation engine — physics, contacts, rendering, and SITL.

---

## Quick start (no simulator)

```bash
git clone https://github.com/alexandrelheinen/fret.git && cd fret
pip install -e ".[dev]"
pytest tests/ -v --ignore=tests/integration
```

---

## Interactive MuJoCo viewer

```bash
pip install -e ".[sim]"
./scripts/view.sh --model ppp --scenario ppp_warehouse \
  --duration 30 --fps 60 --camera overview
```

See [tutorial.md](tutorial.md) for controls and options.
**WSL2 users:** [wsl.md](wsl.md) — MuJoCo display and rendering on Windows.

---

## Full workspace build

```bash
./scripts/install.sh -y
./scripts/setup.sh -y
./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

---

## v1.0 — PPP warehouse

```bash
# Visual preview (no ROS)
./scripts/view.sh --model ppp --scenario ppp_warehouse \
  --duration 30 --fps 60 --camera overview

# Full SITL pipeline
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp
```

**Deliverables:**

| Asset | Path |
|---|---|
| MJCF world | `src/fret/mjcf/ppp_warehouse.xml` |
| Interactive viewer | `scripts/view_mujoco.py` / `scripts/view.sh` |
| MuJoCo bridge | `src/fret/ros/mujoco_bridge.py` |
| Bridge config | `src/fret/config/simulation/mujoco.yml` |
| Scenario | `src/fret/config/scenarios/ppp_warehouse.yml` |
| Perception layout | `src/fret/config/perception_ppp_warehouse.yaml` |
| Video script | `scripts/render_mujoco.py` / `scripts/video.sh` |

---

## v1.1 — Dubins race

```bash
./scripts/view.sh --model dubins --scenario dubins_race \
  --duration 30 --fps 60 --camera overview
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/dubins --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
```

---

## v1.2 — Physics SITL (planned)

Enable actuator-driven simulation with contact dynamics:

```bash
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp physics_mode:=true
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=true
```

See [mujoco.md § Physics SITL](mujoco.md#physics-sitl-v12-target) for architecture
and tuning workflow.

---

## Regression — bootstrap SCARA (pure-Python)

```bash
python3 -m pytest tests/integration/test_scenario_pillar_avoidance.py -v
python3 -m pytest tests/scene/ tests/planning/ -q
```

These validate the MS-1–5 pipeline without ROS or MuJoCo physics. Visual inspection
uses `./scripts/view.sh` once RRP MJCF is available (v1.3).

---

## Recording

```bash
# MuJoCo headless MP4
./scripts/video.sh --model ppp --scenario ppp_warehouse --camera overview \
  -o /tmp/fret.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm rrt_star --full-duration

# Magnetic grasp FSM demo (pure Python, no sim)
python3 scripts/demo_grasp.py

# PPP C-space checker demo (warehouse obstacles)
python3 scripts/demo_ppp_checker.py

# ROS bag
ros2 bag record /joint_states /joint_commands /joint_trajectory
```

---

## Log locations

| Log | Path |
|---|---|
| colcon build | `./log/latest_build/logger_all.log` |
| ROS 2 | `~/.ros/log/latest/launch.log` |
| Sim results | `/tmp/sim_<name>/results.env` |

---

## Known limitations

- Interactive viewer requires a desktop display (or X11 forwarding). On WSL2,
  see [wsl.md](wsl.md).
- v1.2+ SITL and release showcases default to MuJoCo physics (`physics_mode:=true`).
  Kinematic mirroring remains via `physics_mode:=false` or `--kinematic-mode`.
- Full acceptance criteria: see [releases.md](releases.md).
