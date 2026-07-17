# MuJoCo robot-model repertoire

> **Purpose:** Single inventory of external MuJoCo projects and model packs that
> FRET can reuse instead of reinventing MJCF. Complements the visual-asset notes
> in [assets/dubins/README.md](assets/dubins/README.md) and the integration spec
> in [mujoco.md](mujoco.md).
>
> **Related:** FR-SIM-11 unit sandboxes · [robots/README.md](robots/README.md)

FRET vendors the ROBOTIS TurtleBot3 Burger MJCF for unit physics
(`turtlebot3_unit`) and race agents (`dubins_race`). External packs remain the
source of meshes and validated wheel actuators — not a replacement for the FRET
bridge/ROS layer.

---

## How to read the tables

| Column | Meaning |
|---|---|
| **FRET use today** | Already vendored, cited, or wired in this repo |
| **Fit** | How well the project matches FRET's diff-drive / arm roadmap |
| **Adopt?** | Recommendation for the next physics-foundation work |

---

## Model collections (MJCF libraries)

| Project | License | What it provides | FRET use today | Fit | Adopt? |
|---|---|---|---|---|---|
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Per-model (often Apache-2.0 / BSD) | Arms, humanoids, mobile manipulators, props | Planned for 6-DOF meshes ([robots/six_dof.md](robots/six_dof.md)); Stretch/TidyBot cited in Dubins asset notes | High for arms / props | **Yes** — import meshes for v1.4; do not take whole stack |
| [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) (`robotis_tb3`) | Apache-2.0 | TurtleBot3 Burger / Waffle with **real wheel joints + actuators** | **In use** — `turtlebot3_burger.xml` / `turtlebot3_unit.xml` + race agents; import via `scripts/import_turtlebot3_assets.py` | **Best** for true diff-drive | **Done** for unit + race wheel physics |
| [AuTURBO/robotis_mujoco_menagerie](https://github.com/AuTURBO/robotis_mujoco_menagerie) | Apache-2.0 | Community mirror / extensions of ROBOTIS models | Not used | Same as ROBOTIS | Optional mirror if upstream layout changes |

---

## Simulation frameworks / bridges (extra layer on MuJoCo)

| Project | License | What it provides | FRET use today | Fit | Adopt? |
|---|---|---|---|---|---|
| [hakoniwalab/hakoniwa-mujoco-robots](https://github.com/hakoniwalab/hakoniwa-mujoco-robots) | Check repo | TB3 + LiDAR on MuJoCo, Hakoniwa PDU, gamepad samples | Not used | Strong **reference** for wheel `ctrl` mapping and sensors | **Borrow ideas** — too heavy to adopt as FRET's sim runtime |
| [Centre-for-Biorobotics/dr_mujoco](https://github.com/Centre-for-Biorobotics/dr_mujoco) | Check repo | Small ROS 2 ↔ MuJoCo differential-drive bridge | Not used | Good educational bridge pattern | **Reference only** — FRET already has `MuJoCoBridgeNode` |
| MuJoCo native (`pip install mujoco`) | Apache-2.0 | Physics engine + Python bindings | **In use** (`.[sim]`, showcase, SITL) | Required | Keep as sole engine (FR-SIM-01) |
| [Gymnasium / mujoco bindings](https://gymnasium.farama.org/) | Apache-2.0 | RL env wrappers around MuJoCo | Not used | Out of scope for SITL/planning | No |
| dm_control | Apache-2.0 | DeepMind control suite + composer | Not used | Research RL; not ROS SITL | No |

---

## Warehouse / visual assets (already in FRET)

| Asset | Source | License | FRET path | Notes |
|---|---|---|---|---|
| AWS RoboMaker warehouse meshes + ground | AWS RoboMaker | MIT-0 | `mjcf/assets/aws_warehouse/` | Used by Dubins showcase; import via `scripts/import_aws_warehouse_assets.py` |
| TurtleBot3 Burger MJCF + meshes | ROBOTIS Menagerie submodule | Apache-2.0 | `mjcf/turtlebot3_*.xml` + `third_party/robotis_mujoco_menagerie/` | Full physics model (wheels + actuators) |
| Procedural diff-drive twin | FRET | — | `mjcf/diffdrive_unit.xml` | Legacy open-loop twin; TB3 is primary |

---

## Mapping to FRET robots

| FRET robot | Preferred external source | Current FRET MJCF | Gap |
|---|---|---|---|
| Diff-drive / “Dubins” mobile | ROBOTIS TB3 Menagerie | `turtlebot3_unit.xml` + `dubins_race.xml` (true wheel actuators) | Race E2E controller tuning may still need work; unit physics is green |
| RRP / SCARA (v1.3) | URDF→MJCF or custom | Planned | — |
| 6-DOF (v1.4) | MuJoCo Menagerie (UR / similar) | Planned | Mesh import only |

---

## Decision rules

1. **Unit physics first** — prove motion on `*_unit.xml` (`fret.simulation`) before changing showcase worlds.
2. **Import meshes, keep FRET control** — vendor OBJ/STL under `mjcf/assets/`; do not pull foreign ROS bridges as the SITL runtime.
3. **True wheels for mobile bases** — prefer hinge-wheel MJCF (TB3 or `diffdrive_unit`) over SE(2) slides + `qvel` projection for any new mobile work.

---

## Regeneration / import scripts

| Script | Purpose |
|---|---|
| `scripts/import_aws_warehouse_assets.py` | AWS warehouse meshes + textures |
| `scripts/import_turtlebot3_assets.py` | TurtleBot3 physics meshes from ROBOTIS Menagerie |
| `scripts/import_turtlebot3_assets.py` | Verifies Menagerie submodule meshes (no local STL copy) |
| `scripts/generate_dubins_race_mjcf.py` | Rebuild race obstacle visual block |

---

## References

- [MuJoCo model gallery](https://mujoco.readthedocs.io/en/stable/models.html)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- FRET Dubins visual benchmark: [assets/dubins/README.md](assets/dubins/README.md)
