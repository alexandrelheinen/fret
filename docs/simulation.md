# FretSim — Simulation Tutorial

> **Start here for visuals:** [mujoco_for_dummies.md](mujoco_for_dummies.md) —
> consolidated MuJoCo guide (interactive viewer + MP4 + SITL).
>
> **Release focus:** v1.0 PPP warehouse in MuJoCo. See [releases.md](releases.md).

---

## Modes

| Mode | Requirements | Purpose |
|---|---|---|
| **Pure-Python** | Python 3.12+, numpy | Unit tests, algorithm validation |
| **MuJoCo viewer** | `mujoco` | **Live 3D window** — `./scripts/view.sh` |
| **MuJoCo MP4** | `mujoco`, `imageio` | Headless showcase video |
| **MuJoCo SITL** | `mujoco`, ROS 2 Jazzy | Full ROS pipeline (`backend:=mujoco`) |
| **Gazebo SITL** | Gazebo Harmonic, ROS 2 Jazzy | Headless physics backend (SCARA regression) |

FRET uses **MuJoCo only** for 3D visualization. RViz and Gazebo GUIs are not used.

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
./scripts/view.sh
```

See [mujoco_for_dummies.md](mujoco_for_dummies.md) for controls, options, and
troubleshooting.

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
./scripts/view.sh

# Full SITL pipeline
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco
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

## Regression — bootstrap SCARA (headless Gazebo)

```bash
ros2 launch fret sim.py model:=scara
ros2 launch fret sitl.py scenario:=static_reach model:=scara backend:=gazebo
```

These validate the MS-1–5 pipeline. Use MuJoCo (`./scripts/view.sh`) for any
visual inspection.

---

## Recording

```bash
# MuJoCo headless MP4 (v1.0 CI target)
./scripts/video.sh -o /tmp/v10.mp4

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

- Interactive viewer requires a desktop display (or X11 forwarding).
- Gazebo SITL runs headless; it is a physics backend, not a visual viewer.
- Full v1.0 acceptance criteria: see [releases.md](releases.md).
