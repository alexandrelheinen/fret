# MuJoCo Tutorial — FRET Visual Guide (v1.0 PPP · v1.1 Dubins)

> **One-stop tutorial** for seeing FRET robots in MuJoCo.
> MuJoCo is FRET's simulation engine for physics, contacts, rendering, and SITL.

**Related:** [mujoco.md](mujoco.md) · [simulation.md](simulation.md) · [wsl.md](wsl.md) ·
[releases.md](releases.md) · [robots/ppp.md](robots/ppp.md)

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

Opens a **live MuJoCo window**. No ROS required. **All parameters are required**
— invoking `./scripts/view.sh` with no arguments prints `missing arguments` and
the full `--help` text.

```bash
git clone https://github.com/alexandrelheinen/fret.git && cd fret
pip install -e ".[sim]"
./scripts/view.sh --model ppp --scenario ppp_warehouse \
  --duration 30 --fps 60 --camera overview
```

### Useful options

```bash
# Play once and exit (still requires all base flags)
./scripts/view.sh --model ppp --scenario ppp_warehouse \
  --duration 45 --fps 60 --camera overview --no-loop

# CI / headless check (no window)
python3 scripts/view_mujoco.py --model ppp --scenario ppp_warehouse \
  --duration 30 --fps 60 --camera overview --dry-run
```

**Controls (MuJoCo viewer):**

| Input | Action |
|---|---|
| Left drag | Orbit camera |
| Right drag | Pan |
| Scroll | Zoom |
| Double-click | Select body |
| Esc / close window | Quit |

**Scene files:** `src/fret/mjcf/ppp_warehouse.xml`, `src/fret/mjcf/dubins_race.xml`

---

## 2. Headless MP4 video

Renders a shareable `.mp4` without opening a window. **All parameters are
required** — bare `./scripts/video.sh` prints `missing arguments` and `--help`.

```bash
pip install -e ".[sim]"
./scripts/video.sh --model ppp --scenario ppp_warehouse --camera overview \
  -o /tmp/fret_ppp_warehouse.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm rrt_star --full-duration
```

Release-style multi-camera export:

```bash
./scripts/video.sh --model dubins --scenario dubins_race --all-cameras \
  --output-dir /tmp/showcase --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
```

Underlying script: `scripts/render_mujoco.py` (same required flags).

Release CI builds showcase MP4s on version tags (`v*.*.*`) via
`.github/workflows/release.yml`, uploads to Cloudflare R2, and keeps a
GitHub Actions artifact backup. Each release scenario exports two POVs:
**overview** (oblique whole-scene) and **follow** (chase camera).

| Scenario | Model | Release duration |
|---|---|---|
| `ppp_warehouse` | PPP | full simulation (real-time post-process) |
| `dubins_race` | Dubins | full simulation (real-time post-process) |

### Download from R2 (WSL-friendly — no MuJoCo rendering needed)

CI uploads to a **private** R2 bucket. Download locally with the same API
token used for CI (never commit secrets to git):

```bash
cp .env.example .env    # fill R2_* values once; .env is gitignored
sudo apt install awscli # if needed
./scripts/download_showcase.sh
./scripts/download_showcase.sh --tag v1.1.0 --all
./scripts/download_showcase.sh --scenario dubins_race --camera follow
```

Default output: `artifacts/r2/ppp_warehouse_latest.mp4` (also gitignored).
Use `--all` to fetch every release POV (both scenarios × overview + follow).

---

## 3. Full ROS 2 SITL (planning + control + sim)

Runs the complete product pipeline with the MuJoCo bridge. Pair with the
viewer above or record topics if needed.

```bash
./scripts/install.sh -y && ./scripts/setup.sh -y && ./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
```

| Component | Path |
|---|---|
| MJCF worlds | `src/fret/mjcf/` |
| MuJoCo bridge | `src/fret/ros/mujoco_bridge.py` |
| Bridge config | `src/fret/config/simulation/mujoco.yml` |
| Integration spec | [mujoco.md](mujoco.md) |

### Typical workflow

1. **Develop algorithms** — `pytest tests/ -v --ignore=tests/integration`
2. **Preview visually** — `./scripts/view.sh --model … --scenario … …`
3. **Validate E2E** — `pytest tests/integration/test_scenario_ppp_warehouse.py -v`
4. **Export video** — `./scripts/video.sh --model … --scenario … …`
5. **Run SITL** — `ros2 launch fret sitl.py …`

---

## 4. Pure-Python demos (terminal only)

| Demo | Command |
|---|---|
| MuJoCo bridge I/O | `python3 scripts/demo_mujoco_bridge.py` |
| Magnetic grasp FSM | `python3 scripts/demo_grasp.py` |
| C-space checker | `python3 scripts/demo_ppp_checker.py` |
| PPP controller | `python3 scripts/demo_ppp_controller.py` |

---

## 5. Physics SITL (v1.2)

From v1.2, MuJoCo advances the simulation with `mj_step` and actuator forces
instead of kinematic pose mirroring. Enable with:

```bash
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp physics_mode:=true
```

Architecture, actuator mapping, and tuning workflow: [mujoco.md](mujoco.md).

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `MuJoCo is required` | `pip install mujoco` |
| Black / empty viewer | WSL2: see [wsl.md](wsl.md); otherwise check display / Wayland |
| `Joint not found` | Ensure `model:=ppp scenario:=ppp_warehouse` |
| MP4 won't play | `pip install imageio-ffmpeg` |
| SITL launch fails | Build workspace, source `install/setup.bash` |
| Headless render fails | `export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl` |

---

## Visual assets and realism

FRET v1.0 ships a **hybrid MJCF** scene (`ppp_warehouse.xml`):

- **Gantry:** procedural primitives (Cartesian topology, identity FK)
- **Warehouse:** [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) meshes (MIT-0) for shelves, box clusters, floor, and wall textures

Assets live under `src/fret/mjcf/assets/aws_warehouse/`. Regenerate with:

```bash
pip install trimesh pycollada
python3 scripts/import_aws_warehouse_assets.py
```

Collision boxes for planning/perception are unchanged (invisible box geoms).

Reference photos and axis mapping: [robots/ppp.md](robots/ppp.md).

---

## File map

```
scripts/view.sh              ← interactive 3D viewer (start here)
scripts/view_mujoco.py
scripts/video.sh             ← headless MP4 (local; needs working GL)
scripts/download_showcase.sh ← download CI/R2 MP4 (no GL needed)
scripts/render_mujoco.py
.env.example                 ← R2 credential template (.env gitignored)
docs/mujoco.md               ← full integration specification
src/fret/mjcf/               ← all MJCF scenes
src/fret/ros/mujoco_bridge.py
src/fret/launch/mujoco.py
src/fret/launch/sitl.py
```
