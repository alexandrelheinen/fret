# MuJoCo Tutorial — FRET Visual Guide (v1.1 Dubins · v1.2 physics)

> **One-stop tutorial** for seeing FRET robots in MuJoCo.
> MuJoCo is FRET's simulation engine for physics, contacts, rendering, and SITL.

**Related:** [mujoco.md](mujoco.md) · [simulation.md](simulation.md) · [wsl.md](wsl.md) ·
[releases.md](releases.md) · [robots/dubins.md](robots/dubins.md)

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
./scripts/view.sh --model dubins --scenario dubins_race \
  --duration 30 --fps 60 --camera overview
```

### Useful options

```bash
# Play once and exit (still requires all base flags)
./scripts/view.sh --model dubins --scenario dubins_race \
  --duration 45 --fps 60 --camera overview --no-loop

# CI / headless check (no window)
python3 scripts/view_mujoco.py --model dubins --scenario dubins_race \
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

**Scene files:** `src/fret/mjcf/dubins_race.xml`

---

## 2. Headless MP4 video

Renders a shareable `.mp4` without opening a window. **All parameters are
required** — bare `./scripts/video.sh` prints `missing arguments` and `--help`.

```bash
pip install -e ".[sim]"
./scripts/video.sh --model dubins --scenario dubins_race --camera overview \
  -o /tmp/fret_dubins_race.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration
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
GitHub Actions artifact backup. Camera policy (see
[`config/release/showcase.yml`](../src/fret/config/release/showcase.yml)):

| Scenario | Model | Class | Cameras |
|---|---|---|---|
| `dubins_race` | Dubins / TB3 | mobile | `overview` (follow optional / local) |
| `omx_wall_maze_rrt` | OpenMANIPULATOR-X (Γ maze, RRT*) | static | `overview` |
| `omx_wall_maze_sst` | OpenMANIPULATOR-X (Γ maze, SST) | static | `overview` |
| `omy_pick_place` | OpenMANIPULATOR-Y (floor ball → cone) | static | `overview` |
| `omy_clutter_rrt` | OpenMANIPULATOR-Y (clutter, RRT*) | static | `overview` |
| `omy_clutter_sst` | OpenMANIPULATOR-Y (clutter, SST) | static | `overview` |

Same maze / wall geometry + tracking for each planner pair; only the transfer
planner changes (OM-X Γ-maze and OMY mid-cell wall).

### Download from R2 (WSL-friendly — no MuJoCo rendering needed)

CI uploads to a **private** R2 bucket. Download locally with the same API
token used for CI (never commit secrets to git):

```bash
cp .env.example .env    # fill R2_* values once; .env is gitignored
sudo apt install awscli # if needed
./scripts/download_showcase.sh
./scripts/download_showcase.sh --tag v1.2.3 --all
./scripts/download_showcase.sh --scenario dubins_race --camera follow
./scripts/download_showcase.sh --scenario omx_wall_maze_rrt --camera overview
./scripts/download_showcase.sh --scenario omx_wall_maze_sst --camera overview
./scripts/download_showcase.sh --scenario omy_pick_place --camera overview
./scripts/download_showcase.sh --scenario omy_clutter_rrt --camera overview
./scripts/download_showcase.sh --scenario omy_clutter_sst --camera overview
```

Default output: `artifacts/r2/dubins_race_latest.mp4` (also gitignored).
Use `--all` to fetch every release POV.

---

## 3. Full ROS 2 SITL (planning + control + sim)

Runs the complete product pipeline with the MuJoCo bridge. Pair with the
viewer above or record topics if needed.

```bash
./scripts/install.sh -y && ./scripts/setup.sh -y && ./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash

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
3. **Validate E2E** — `pytest tests/scenario/test_dubins_race_e2e.py -v`
4. **Export video** — `./scripts/video.sh --model … --scenario … …`
5. **Run SITL** — `ros2 launch fret sitl.py …`

---

## 4. Pure-Python demos (terminal only)

| Demo | Command |
|---|---|
| MuJoCo bridge I/O | `python3 scripts/demo_mujoco_bridge.py` |

---

## 5. Physics SITL (v1.2, default)

From v1.2, MuJoCo advances the simulation with `mj_step` and velocity actuators
instead of kinematic pose mirroring. Release CI and showcase renders pass
`--physics-mode`; ROS SITL enables physics by default.

```bash
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins
# explicit (same as default):
ros2 launch fret sitl.py scenario:=dubins_race model:=dubins physics_mode:=true

# headless physics showcase MP4
./scripts/video.sh --model dubins --scenario dubins_race --camera overview \
  -o /tmp/fret_dubins_physics.mp4 --fps 30 --width 1280 --height 720 \
  --collision-backend mujoco --planner-algorithm sst --full-duration \
  --physics-mode
```

Kinematic mirror for fast regression: `physics_mode:=false` or `--kinematic-mode`.

Architecture, actuator mapping, and tuning workflow: [mujoco.md](mujoco.md).

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `MuJoCo is required` | `pip install mujoco` |
| Black / empty viewer | WSL2: see [wsl.md](wsl.md); otherwise check display / Wayland |
| `Joint not found` | Ensure `model:=dubins scenario:=dubins_race` |
| MP4 won't play | `pip install imageio-ffmpeg` |
| SITL launch fails | Build workspace, source `install/setup.bash` |
| Headless render fails | `export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl` |

---

## Visual assets and realism

FRET v1.1 ships a **hybrid MJCF** scene (`dubins_race.xml`):

- **Agents:** ROBOTIS TurtleBot3 Burger MJCF (Apache-2.0) with true wheel hinges + velocity actuators on freejoints
- **Warehouse:** [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) meshes (MIT-0) for floor, clutter, and shelf visuals

Assets live under `src/fret/mjcf/assets/aws_warehouse/` and
`third_party/robotis_mujoco_menagerie/` (git submodule). Regenerate with:

```bash
pip install trimesh pycollada
python3 scripts/import_aws_warehouse_assets.py
python3 scripts/import_turtlebot3_assets.py
```

Collision boxes for planning remain analytic rectangles in YAML.

Reference notes: [robots/dubins.md](robots/dubins.md) ·
[assets/dubins/README.md](assets/dubins/README.md).

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
