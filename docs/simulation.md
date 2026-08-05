# FretSim — Simulation Guide

> **MuJoCo integration spec:** [mujoco.md](mujoco.md)  
> **Visual tutorial:** [tutorial.md](tutorial.md)  
> **Release criteria:** [releases.md](releases.md)

---

## Modes

| Mode | Requirements | Purpose |
|---|---|---|
| **Pure-Python** | Python 3.12+, numpy | Unit tests, algorithm validation |
| **MuJoCo viewer** | `mujoco` | **Live 3D window** — `./scripts/view.sh` (all flags required) |
| **MuJoCo MP4** | `mujoco`, `imageio` | Headless showcase video |
| **MuJoCo SITL** | `mujoco`, ROS 2 Jazzy | Full ROS pipeline via `sitl.py` |
| **MuJoCo physics SITL** | `mujoco`, ROS 2 Jazzy | Actuator-driven `mj_step` (default) |

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
./scripts/view.sh --model dubins --scenario dubins_race \
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

## Dubins race

```bash
./scripts/view.sh --model dubins --scenario dubins_race \
  --duration 30 --fps 60 --camera overview
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/dubins --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
```

**Deliverables:**

| Asset | Path |
|---|---|
| MJCF world | `src/fret/mjcf/dubins_race.xml` |
| Interactive viewer | `scripts/view_mujoco.py` / `scripts/view.sh` |
| MuJoCo bridge | `src/fret/ros/mujoco_bridge.py` |
| Bridge config | `src/fret/config/simulation/mujoco.yml` |
| Scenario | `src/fret/config/scenarios/dubins_race.yml` |
| Video script | `scripts/render_mujoco.py` / `scripts/video.sh` |

---

## Physics SITL

Actuator-driven simulation with contact dynamics is the default:

```bash
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/dubins_physics --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration \
  --physics-mode
```

Kinematic mirror (fast regression): `physics_mode:=false` or `--kinematic-mode`.

See [mujoco.md § Physics SITL](mujoco.md#physics-sitl-v12) for architecture
and tuning workflow. Implementation spec: [mujoco_physics_v1.2.md](mujoco_physics_v1.2.md).

---

## Manipulation SITL

OpenMANIPULATOR-X / OMY tabletop scenarios use MuJoCo physics SITL with
Menagerie meshes. See [robots/](robots/README.md).


## Recording

```bash
# MuJoCo headless MP4
./scripts/video.sh --model dubins --scenario dubins_race --camera overview \
  -o /tmp/fret.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration

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
- SITL and release showcases default to MuJoCo physics (`physics_mode:=true`).
  Kinematic mirroring remains via `physics_mode:=false` or `--kinematic-mode`.
- Full acceptance criteria: see [releases.md](releases.md).
