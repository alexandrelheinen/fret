# MuJoCo robot-model repertoire

> **Purpose:** Single inventory of external MuJoCo projects and model packs that
> FRET can reuse instead of reinventing MJCF. Complements the visual-asset notes
> in [assets/dubins/README.md](assets/dubins/README.md) and the integration spec
> in [mujoco.md](mujoco.md).
>
> **Related:** FR-SIM-11 unit sandboxes · [robots/README.md](robots/README.md)

FRET keeps **thin custom MJCF** for foundational unit robots (`ppp_unit`,
`diffdrive_unit`) and showcase worlds (`ppp_warehouse`, `dubins_race`). External
packs are preferred for **meshes**, **validated wheel/arm kinematics**, and
**reference actuators** — not as a replacement for the FRET bridge/ROS layer.

---

## How to read the tables

| Column | Meaning |
|---|---|
| **FRET use today** | Already vendored, cited, or wired in this repo |
| **Fit** | How well the project matches FRET's Cartesian / diff-drive / arm roadmap |
| **Adopt?** | Recommendation for the next physics-foundation work |

---

## Model collections (MJCF libraries)

| Project | License | What it provides | FRET use today | Fit | Adopt? |
|---|---|---|---|---|---|
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Per-model (often Apache-2.0 / BSD) | Arms, humanoids, mobile manipulators, props | Planned for 6-DOF meshes ([robots/six_dof.md](robots/six_dof.md)); Stretch/TidyBot cited in Dubins asset notes | High for arms / props; no simple PPP gantry | **Yes** — import meshes for v1.4; do not take whole stack |
| [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) (`robotis_tb3`) | Apache-2.0 | TurtleBot3 Burger / Waffle with **real wheel joints + actuators** | TB3 **meshes** vendored via `scripts/import_dubins_assets.py`; race MJCF still holonomic SE(2) | **Best** for true diff-drive | **Yes** — prefer TB3 physics MJCF (or patterns) when upgrading race agents beyond `diffdrive_unit` |
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
| AWS RoboMaker warehouse meshes + ground | AWS RoboMaker | MIT-0 | `mjcf/assets/aws_warehouse/` | Used by PPP + Dubins showcase; import via `scripts/import_aws_warehouse_assets.py` |
| TurtleBot3 Burger chassis STL | ROBOTIS Menagerie | Apache-2.0 | `mjcf/assets/dubins/turtlebot3/` | Visual only on race agents today |
| Procedural PPP gantry | FRET | — | `mjcf/ppp_warehouse.xml`, `mjcf/ppp_unit.xml` | No good Menagerie gantry; keep custom |
| Unit diff-drive | FRET | — | `mjcf/diffdrive_unit.xml` | Minimal freejoint + 2 wheels; foundation before race upgrade |

---

## Mapping to FRET robots

| FRET robot | Preferred external source | Current FRET MJCF | Gap |
|---|---|---|---|
| Cartesian / PPP | None (custom 3-slide) | `ppp_unit.xml` (physics OK), `ppp_warehouse.xml` (showcase) | Warehouse Z hold still needs gravcomp/mass cleanup if open-loop physics is required there |
| Diff-drive / “Dubins” mobile | ROBOTIS TB3 Menagerie | `diffdrive_unit.xml` (true wheels); `dubins_race.xml` (holonomic slides + visual wheels) | Race scene not yet on true wheel actuators |
| RRP / SCARA (v1.3) | URDF→MJCF or custom | Planned | — |
| 6-DOF (v1.4) | MuJoCo Menagerie (UR / similar) | Planned | Mesh import only |

---

## Decision rules

1. **Unit physics first** — prove motion on `*_unit.xml` (`fret.simulation`) before changing showcase worlds.
2. **Import meshes, keep FRET control** — vendor OBJ/STL under `mjcf/assets/`; do not pull foreign ROS bridges as the SITL runtime.
3. **True wheels for mobile bases** — prefer hinge-wheel MJCF (TB3 or `diffdrive_unit`) over SE(2) slides + `qvel` projection for any new mobile work.
4. **Cartesian stays custom** — three prismatic slides with explicit mass / `gravcomp` beat hunting for a Menagerie gantry.

---

## Regeneration / import scripts

| Script | Purpose |
|---|---|
| `scripts/import_aws_warehouse_assets.py` | AWS warehouse meshes + textures |
| `scripts/import_dubins_assets.py` | TurtleBot3 meshes from ROBOTIS Menagerie |
| `scripts/generate_dubins_race_mjcf.py` | Rebuild race obstacle visual block |

---

## References

- [MuJoCo model gallery](https://mujoco.readthedocs.io/en/stable/models.html)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- FRET Dubins visual benchmark: [assets/dubins/README.md](assets/dubins/README.md)
- FRET PPP asset notes: [assets/ppp/README.md](assets/ppp/README.md)
