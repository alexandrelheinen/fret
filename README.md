# FRET

<img src="docs/images/fret.svg" alt="FRET Logo" width="120" align="left">

FRET (Full-stack Robotic End-effector control and Trajectory planning) is a ROS 2
robotics framework that connects the **ARCO** motion-planning library to simulators
and hardware.

<br clear="left">

---

## Release roadmap

| Version | Robot | Scenario |
|---|---|---|
| **v1.0** *(complete)* | PPP gantry | Warehouse box pick-and-place (magnetic grasp) |
| **v1.1** | Dubins mobile × 2 | Dual race A→B through column forest |
| **v1.2** | RRP / SCARA | Reproduce ARCO `rrp` + `rr` examples |
| **v1.3** | 6-DOF arm | Final challenge |

Full specification: **[docs/releases.md](docs/releases.md)**

---

## Stack

| Layer | Technology |
|---|---|
| Planning | ARCO (RRT*, SST, KDTree, pruner) |
| Middleware | ROS 2 Jazzy |
| Visual simulation | MuJoCo |
| Engineering SITL | Gazebo Harmonic (v1.2+) |
| Control | Per-robot (prismatic P, Dubins/Pure Pursuit, Jacobian) |

---

## Quick start

> **Visual guide:** [docs/tutorial.md](docs/tutorial.md)

### Pure Python (algorithms only — no ROS)

```bash
git clone https://github.com/alexandrelheinen/fret.git && cd fret
pip install -e ".[dev]"
pytest tests/ -v --ignore=tests/integration
```

### MuJoCo interactive viewer (easiest visual)

```bash
pip install -e ".[sim]"
./scripts/view.sh
```

### Full workspace (ROS 2 SITL)

```bash
./scripts/install.sh -y && ./scripts/setup.sh -y && ./scripts/build.sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

### MuJoCo preview video (optional)

```bash
pip install -e ".[sim]"
./scripts/video.sh --collision-backend mujoco --planner-algorithm rrt_star -o /tmp/fret_ppp_warehouse.mp4
```

**Release showcase (v1.0.0+):** after tagging, CI uploads multi-POV warehouse clips to
Cloudflare R2. Download locally:

```bash
./scripts/download_showcase.sh --tag v1.0.0 --all
# or latest overview:
./scripts/download_showcase.sh --latest
```

---

## v1.0 local development (incremental)

Each PR adds a testable slice. After `pip install -e ".[dev]"`:

| Step | Feature | Verify locally |
|---|---|---|
| 1 | PPP kinematics (T10-01) | `pytest tests/control/test_kinematics_ppp.py -v` |
| 2 | Magnetic grasp FSM (T10-04) | `pytest tests/control/test_grasp_magnet.py -v` |
| 3 | MuJoCo interactive viewer | `pip install -e ".[sim]" && ./scripts/view.sh` |
| 4 | MuJoCo preview video (T10-07) | `pip install -e ".[sim]" && ./scripts/video.sh -o /tmp/v10.mp4` |
| 5 | PPP C-space checker (T10-05, T10-08) | `pytest tests/planning/test_cspace_checker_ppp.py -v` |
| 6 | PPP prismatic controller (T10-09) | `pytest tests/control/test_controller_ppp.py -v` |
| 7 | MuJoCo ROS bridge (T10-03) | `pytest tests/ros/test_mujoco_bridge.py -v` |
| 8 | PPP warehouse launch config (T10-06) | `pytest tests/test_sitl_config.py -v` |
| 9 | PPP warehouse E2E validation (V10-2–5) | `pytest tests/integration/test_scenario_ppp_warehouse.py -v` |

**PPP kinematics** — identity-map FK/IK for the gantry:

```bash
python3 -c "
from fret.control import Kinematics
import numpy as np
k = Kinematics('ppp')
q = np.array([10.0, 5.0, 2.0])
print('EE position:', k.forward_kinematics(q)[:3, 3])
"
```

**Magnetic grasp FSM** — weld / transport / release without ROS:

```bash
python3 scripts/demo_grasp.py
# or run the unit tests:
pytest tests/control/test_grasp_magnet.py -v
```

**MuJoCo interactive viewer** — live 3D warehouse preview:

```bash
pip install -e ".[sim]"
./scripts/view.sh
```

**MuJoCo warehouse preview** — headless MP4 of gantry motion:

```bash
pip install -e ".[sim]"
./scripts/video.sh --duration 30 --fps 30 -o /tmp/v10.mp4
```

**PPP C-space checker** — warehouse obstacles + EE/cargo envelope:

```bash
python3 scripts/demo_ppp_checker.py
# or run the unit tests:
pytest tests/planning/test_cspace_checker_ppp.py tests/planning/test_ppp_obstacles.py -v
```

**PPP prismatic controller** — per-axis P-control at 50 Hz:

```bash
python3 scripts/demo_ppp_controller.py
# or run the unit tests:
pytest tests/control/test_controller_ppp.py -v
```

**MuJoCo ROS bridge** — `/joint_commands` → `/joint_states` core:

```bash
python3 scripts/demo_mujoco_bridge.py
# or run the unit tests:
pytest tests/ros/test_mujoco_bridge.py -v
```

**PPP warehouse E2E** — pure-Python acceptance for V10-2 – V10-5:

```bash
pytest tests/integration/test_scenario_ppp_warehouse.py -v
```

---

## v1.0 launch

```bash
ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco
```

---

## Documentation

### Product

- [Release specification v1.0–v1.3](docs/releases.md)
- [Project roadmap](docs/roadmap.md)
- [Scenario library](docs/scenarios.md)
- [MuJoCo tutorial (visual guide)](docs/tutorial.md)
- [Simulation tutorial](docs/simulation.md)

### Architecture

- [Architecture overview](docs/architecture.md)
- [Functional requirements](docs/requirements.md)
- [Interface contracts](docs/interfaces.md)
- [ARCO integration](docs/arco.md)

### Robots

- [Robot model index](docs/robots/README.md)
- [PPP gantry (v1.0)](docs/robots/ppp.md)
- [Dubins mobile (v1.1)](docs/robots/dubins.md)
- [RRP / SCARA (v1.2)](docs/robots/rrp.md)
- [6-DOF (v1.3)](docs/robots/six_dof.md)

### Modules

- [Control](docs/modules/control.md) · [Planning](docs/modules/planning.md)
- [Scene](docs/modules/scene.md) · [ROS nodes](docs/modules/ros_nodes.md)
- [Validation](docs/modules/validation.md) · [Hardware](docs/modules/hardware.md)

### Engineering

- [Coding guidelines](docs/guidelines.md)
- [Contributing](CONTRIBUTING.md)

---

## Architecture

```
Scenario YAML → Scene → ARCO Planner → Controller → MuJoCo / Gazebo
```

Algorithm layers (`control/`, `planning/`, `scene/`, `validation/`) are pure Python.
ROS nodes in `fret.ros` handle simulator I/O only.

---

## Project status

| Item | Status |
|---|---|
| Bootstrap SCARA pipeline (MS-1–5) | ✅ Done (regression CI) |
| v1.0 PPP kinematics (T10-01) | ✅ Done |
| v1.0 magnetic grasp FSM (T10-04) | ✅ Done |
| v1.0 PPP C-space checker (T10-05, T10-08) | ✅ Done |
| v1.0 MuJoCo preview video script (T10-07) | ✅ Done |
| v1.0 PPP prismatic controller (T10-09) | ✅ Done |
| v1.0 MuJoCo bridge (T10-03) | ✅ Done |
| v1.0 scenario launch (T10-06) | ✅ Done |
| v1.0 E2E acceptance V10-2…5 | ✅ Done |
| v1.0 ROS SITL smoke V10-1 | ✅ Done |
| v1.0 release video CI (V10-6) | ✅ Done |
| v1.0 cargo-inclusive transit planning (FR-GSP-02) | ✅ Done |
| v1.0.0 git tag + R2 showcase | 🔲 Pending tag |
| v1.1 – v1.3 | 🔲 Specified |

---

## CI

```bash
bash scripts/check/pre_push.sh
```

| Workflow | Checks |
|---|---|
| `tests.yml` | Build, pytest, smoke |
| `formatting.yml` | Black, isort, clang-format |
| `type_check.yml` | mypy strict |
| `integration.yml` | launch_testing |
| `release.yml` | Showcase MP4 on version tags |

---

## License

MIT — see [LICENSE](LICENSE).
