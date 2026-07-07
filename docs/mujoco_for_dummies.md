# MuJoCo for Dummies — FRET v1.0 Visual Guide

> **One-stop tutorial** for seeing the PPP warehouse gantry in MuJoCo.
> FRET uses **MuJoCo only** for 3D visualization. RViz and Gazebo GUIs are
> not part of this project.

**Related:** [simulation.md](simulation.md) · [releases.md](releases.md) ·
[robots/ppp.md](robots/ppp.md)

---

## What you need

| Goal | Install |
|---|---|
| Watch live 3D animation | `pip install -e ".[sim]"` |
| Save an MP4 file | same (`mujoco` + `imageio`) |
| Full ROS pipeline | ROS 2 Jazzy + `./scripts/build.sh` |

Minimum Python packages:

```bash
pip install mujoco          # interactive viewer
pip install imageio imageio-ffmpeg   # MP4 export only
```

---

## 1. Interactive 3D viewer (easiest)

Opens a **live MuJoCo window** with the PPP gantry moving through the
warehouse preview scene. No ROS required.

```bash
git clone https://github.com/alexandrelheinen/fret.git && cd fret
pip install -e ".[sim]"
./scripts/view.sh
```

### Useful options

```bash
# Longer animation cycle (45 s per loop)
./scripts/view.sh --duration 45

# Play once and exit
./scripts/view.sh --no-loop

# CI / headless check (no window)
python3 scripts/view_mujoco.py --dry-run
```

**Controls (MuJoCo viewer):**

| Input | Action |
|---|---|
| Left drag | Orbit camera |
| Right drag | Pan |
| Scroll | Zoom |
| Double-click | Select body |
| Esc / close window | Quit |

**Scene file:** `src/fret/mjcf/ppp_warehouse.xml`

---

## 2. Headless MP4 video

Renders a shareable `.mp4` without opening a window — ideal for README,
articles, and release tags.

```bash
pip install -e ".[sim]"
./scripts/video.sh -o /tmp/fret_ppp_warehouse.mp4
```

Options:

```bash
./scripts/video.sh --duration 30 --fps 30 --width 1920 --height 1080 -o out.mp4
```

Underlying script: `scripts/render_mujoco.py`

Release CI builds the same asset on version tags (`v*.*.*`) via
`.github/workflows/release.yml`.

---

## 3. Full ROS 2 SITL (planning + control + sim)

Runs the complete v1.0 product pipeline with the MuJoCo backend adapter.
This is **simulation I/O**, not a 3D window — pair it with the viewer
above or record topics if needed.

```bash
./scripts/install.sh -y && ./scripts/setup.sh -y && ./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco
```

| Component | Path |
|---|---|
| MJCF world | `src/fret/mjcf/ppp_warehouse.xml` |
| MuJoCo bridge | `src/fret/ros/mujoco_bridge.py` |
| Bridge config | `src/fret/config/simulation/mujoco.yml` |
| Scenario | `src/fret/config/scenarios/ppp_warehouse.yml` |

### Typical workflow

1. **Develop algorithms** — `pytest tests/ -v --ignore=tests/integration`
2. **Preview visually** — `./scripts/view.sh`
3. **Validate E2E** — `pytest tests/integration/test_scenario_ppp_warehouse.py -v`
4. **Export video** — `./scripts/video.sh -o demo.mp4`
5. **Run SITL** — `ros2 launch fret sitl.py … backend:=mujoco`

---

## 4. Pure-Python demos (terminal only)

| Demo | Command |
|---|---|
| MuJoCo bridge I/O | `python3 scripts/demo_mujoco_bridge.py` |
| Magnetic grasp FSM | `python3 scripts/demo_grasp.py` |
| C-space checker | `python3 scripts/demo_ppp_checker.py` |
| PPP controller | `python3 scripts/demo_ppp_controller.py` |

---

## 5. Gazebo (simulation engine only — no viewer)

Gazebo Harmonic remains available as a **headless physics backend** for
bootstrap SCARA regression tests (`backend:=gazebo`). It does **not**
provide a FRET visualization window.

```bash
# Headless Gazebo SITL (no 3D GUI)
ros2 launch fret sitl.py scenario:=static_reach model:=scara backend:=gazebo
```

For any visual inspection, always use MuJoCo (`./scripts/view.sh` or
`./scripts/video.sh`).

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `MuJoCo is required` | `pip install mujoco` |
| Black / empty viewer | Check display / Wayland; try native desktop session |
| `Joint not found` | Ensure `model:=ppp scenario:=ppp_warehouse` |
| MP4 won't play | `pip install imageio-ffmpeg` |
| SITL launch fails | Build workspace, source `install/setup.bash` |

---

## File map

```
scripts/view.sh              ← interactive 3D viewer (start here)
scripts/view_mujoco.py
scripts/video.sh             ← headless MP4
scripts/render_mujoco.py
src/fret/mjcf/ppp_warehouse.xml
src/fret/ros/mujoco_bridge.py
src/fret/launch/mujoco.py
src/fret/launch/sitl.py      ← backend:=mujoco
```
