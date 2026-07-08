# MuJoCo Tutorial — FRET v1.0 Visual Guide

> **One-stop tutorial** for seeing the PPP warehouse gantry in MuJoCo.
> FRET uses **MuJoCo only** for 3D visualization. RViz and Gazebo GUIs are
> not part of this project.

**Related:** [simulation.md](simulation.md) · [wsl.md](wsl.md) · [releases.md](releases.md) ·
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
`.github/workflows/release.yml`, uploads to Cloudflare R2, and keeps a
GitHub Actions artifact backup.

### Download from R2 (WSL-friendly — no MuJoCo rendering needed)

CI uploads to a **private** R2 bucket. Download locally with the same API
token used for CI (never commit secrets to git):

```bash
cp .env.example .env    # fill R2_* values once; .env is gitignored
sudo apt install awscli # if needed
./scripts/download_showcase.sh
./scripts/download_showcase.sh --tag v0.2.0
```

Default output: `artifacts/r2/ppp_warehouse_latest.mp4` (also gitignored).

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
| Black / empty viewer | WSL2: see [wsl.md](wsl.md); otherwise check display / Wayland |
| `Joint not found` | Ensure `model:=ppp scenario:=ppp_warehouse` |
| MP4 won't play | `pip install imageio-ffmpeg` |
| SITL launch fails | Build workspace, source `install/setup.bash` |

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

### Reference photos (mechanical layout)

![Industrial Cartesian gantry](../assets/ppp/cartesian-robot-reference.jpg)

![Gantry XYZ axis convention](../assets/ppp/gantry-xyz-axes-reference.png)

Copies live under `docs/assets/ppp/` (vendor product images, layout reference
only). See [robots/ppp.md](robots/ppp.md) for axis ↔ joint mapping.

### Public models surveyed (none ready for drop-in use)

| Source | License | Meshes? | Gantry fit |
|---|---|---|---|
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Per-model (Apache-2.0, BSD, …) | OBJ/STL | **No** — serial arms, quadrupeds, hands |
| [VSP-AR/urdf_tutorials `cartesian_robot`](https://github.com/VSP-AR/urdf_tutorials/tree/main/cartesian_robot) | Tutorial | Box primitives only | **Topology match** — dual-rail + XYZ slides, no meshes |
| [GTEC-UDC/linuxcnc_gantry_robot](https://github.com/GTEC-UDC/linuxcnc_gantry_robot) | Open docs | Photos/CAD, no MJCF | Real 5.3 m gantry, not packaged for MuJoCo |
| [COMPAS FAB UR10 on tower](https://compas.dev/compas_fab/latest/examples/03_backends_ros/09_ros_create_urdf_ur10_on_tower.html) | Docs | STL linear stages | Arm-on-gantry, not PPP-only |

**Conclusion:** no maintained public MJCF with photorealistic gantry meshes was
found. FRET reproduces the Cartesian topology procedurally, following the
`cartesian_robot` URDF tutorial pattern (parallel top rails + XYZ slides).

### What MuJoCo provides out of the box

| Source | Content | Fit for PPP warehouse |
|---|---|---|
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Curated robot MJCF + OBJ/STL meshes (UR5e, Franka, quadrupeds, …) | **No gantry / warehouse kits** — industrial arms only |
| Built-in MJCF | `builtin="checker"`, `flat` textures, primitive geoms | Used for gantry frame |
| [obj2mjcf](https://github.com/kevinzakka/obj2mjcf) | Converts composite OBJ → MJCF with materials | Needed when importing external meshes |

Menagerie is the best maintained MuJoCo asset library, but it targets
**serial manipulators and mobile robots**, not overhead cranes or warehouse
environments.

### Third-party meshes (manual import)

For higher visual fidelity you would:

1. Obtain OBJ/STL meshes (e.g. warehouse modular kits on CGTrader, Sketchfab,
   or vendor CAD exports) under a license compatible with your release.
2. Simplify / convex-decompose collision meshes (MeshLab, V-HACD, Open3D).
3. Run `obj2mjcf` or hand-author `<asset><mesh …/></asset>` entries.
4. Replace procedural geoms in `ppp_warehouse.xml` with `type="mesh"` visuals
   while keeping the same joint names and FK offsets.

Typical search terms: *overhead gantry crane*, *bridge crane*, *modular
warehouse*, *pallet rack*. Expect to adapt units and split models into static
frame vs. moving carriage bodies.

### FRET recommendation (v1.0 → v1.2)

| Stage | Approach |
|---|---|
| **v1.0 (current)** | Hybrid MJCF: procedural gantry + AWS warehouse meshes |
| **v1.1** | Optional textured meshes for gantry frame + pallets (visual-only geoms) |
| **v1.2+** | Consider Menagerie arm as secondary cell robot; warehouse still custom |

Track mesh licensing in `src/fret/mjcf/LICENSE` if external assets are added.

---

## File map

```
scripts/view.sh              ← interactive 3D viewer (start here)
scripts/view_mujoco.py
scripts/video.sh             ← headless MP4 (local; needs working GL)
scripts/download_showcase.sh ← download CI/R2 MP4 (no GL needed)
scripts/render_mujoco.py
.env.example                 ← R2 credential template (.env gitignored)
src/fret/mjcf/ppp_warehouse.xml
src/fret/ros/mujoco_bridge.py
src/fret/launch/mujoco.py
src/fret/launch/sitl.py      ← backend:=mujoco
```
