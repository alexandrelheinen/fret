# FretSim — Simulation Tutorial

> **Release focus:** v1.0 PPP warehouse in MuJoCo. See [releases.md](releases.md).

---

## Modes

| Mode | Requirements | Purpose |
|---|---|---|
| **Pure-Python** | Python 3.12+, numpy | Unit tests, algorithm validation |
| **MuJoCo SITL** | `mujoco`, ROS 2 Jazzy | **v1.0 showcase** — PPP warehouse video |
| **Gazebo SITL** | Gazebo Harmonic, ROS 2 Jazzy | Engineering validation (v1.2+ RRP) |

---

## Quick start (no simulator)

```bash
git clone https://github.com/alexandrelheinen/fret.git && cd fret
pip install -e ".[dev]"
pytest tests/ -v --ignore=tests/integration
```

---

## Full workspace build

```bash
./scripts/install.sh -y
./scripts/setup.sh -y
./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

---

## v1.0 — PPP warehouse (planned)

```bash
# Primary v1.0 entry point (not yet implemented)
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco
```

**Deliverables:**

| Asset | Path |
|---|---|
| MJCF world | `src/fret/mjcf/ppp_warehouse.xml` |
| Scenario | `src/fret/config/scenarios/ppp_warehouse.yml` |
| Video script | `scripts/render_mujoco.py` |

---

## Regression — bootstrap SCARA

```bash
ros2 launch fret view.py model:=scara
ros2 launch fret sim.py model:=scara
ros2 launch fret sitl.py scenario:=static_reach model:=scara
```

These validate the MS-1–5 pipeline. They are **not** the v1.0 product demo.

---

## Recording

```bash
# MuJoCo headless MP4 (v1.0 CI target)
python3 scripts/render_mujoco.py --scenario ppp_warehouse --output /tmp/v10.mp4
# or via wrapper:
./scripts/video.sh -o /tmp/v10.mp4

# Magnetic grasp FSM demo (pure Python, no sim)
python3 scripts/demo_grasp.py

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

See [releases.md](releases.md) for current release status. v1.0 MuJoCo backend and
PPP model are not yet implemented in code — specification only.
