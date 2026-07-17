# Dubins Race Visual Assets (SC-v11)

Free, copy-friendly assets for the compact Dubins dual-race showcase
(`dubins_race.xml`) — **real TurtleBot3 Burger** on a **10 m × 10 m** lab floor.

Mesh **sources of truth** are git submodules under `third_party/` (see
[third_party/README.md](../../../third_party/README.md)).

---

## Git submodules

| Submodule | Upstream | License |
| --- | --- | --- |
| `third_party/robotis_mujoco_menagerie` | [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) | Apache-2.0 |
| `third_party/aws-robomaker-small-warehouse-world` | [AWS RoboMaker Small Warehouse](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) | MIT-0 |

```bash
git submodule update --init --recursive
python3 scripts/import_aws_warehouse_assets.py
python3 scripts/import_turtlebot3_assets.py
python3 scripts/generate_dubins_warehouse_layout.py
python3 scripts/generate_dubins_race_mjcf.py
```

---

## Imported AWS assets used in the race scenario

Converted from the AWS submodule into `src/fret/mjcf/assets/aws_warehouse/`
(see also `IMPORTED_ASSETS.md` there):

| Local file | Upstream model | Role in scenario |
| --- | --- | --- |
| `meshes/shelf.obj` | `ShelfD_01` | Diagonal cheek shelves (collision-matched scale) |
| `meshes/shelf_e.obj` | `ShelfE_01` | Outer flank shelves |
| `meshes/clutter_a.obj` | `ClutteringA_01` | Optional crate visual |
| `meshes/clutter_c.obj` | `ClutteringC_01` | On-diagonal dummy foil block |
| `meshes/clutter_d.obj` | `ClutteringD_01` | Optional crate visual |
| `meshes/bucket.obj` | `Bucket_01` | Aisle prop |
| `meshes/desk.obj` | `DeskC_01` | Aisle prop |
| `meshes/lamp.obj` | `Lamp_01` | Aisle prop |
| `meshes/pallet.obj` | `PalletJackB_01` | Aisle prop |
| `meshes/trash.obj` | `TrashCanC_01` | Aisle prop |
| `textures/ground.png` | `GroundB_01` | Floor |
| `textures/wall.png` | `WallB_01` | Available wall texture |

**Not imported:** `ShelfF_01` (degenerate authored extents), `RoofB_01` (not useful indoors).

---

## TurtleBot3 (Menagerie submodule — no local STL copies)

| Mesh | Path |
| --- | --- |
| `burger_base.stl` | `third_party/robotis_mujoco_menagerie/robotis_tb3/assets/` |
| `left_tire.stl` / `right_tire.stl` | same |
| `lds.stl` | same |

Referenced by `dubins_race.xml` and `turtlebot3_burger.xml` via relative `meshdir` /
mesh `file=` paths. Dead vendored trees `mjcf/assets/turtlebot3/` and
`mjcf/assets/dubins/turtlebot3/` were removed.

---

## Lab sizing (real TB3)

| Parameter | Value |
| --- | --- |
| Workspace | 10 m × 10 m |
| Start → goal | (1.2, 1.2) → (8.8, 8.8) |
| Vehicle width / radius / clearance | 0.18 m / 0.12 m / 0.18 m |
| Min corridor gap | `width + 4×clearance` (+0.24 m PP slack) ≈ **1.14 m** |
| Wheel ctrl limit | ±6.67 rad/s (≈ 0.22 m/s) |
| Dummy | Pure Pursuit on the straight start→goal segment (foil) |

Per-instance mesh `scale=` is written into MJCF by
`scripts/generate_dubins_race_mjcf.py` (MuJoCo ignores `geom size` on meshes —
that bug previously rendered native ~2 m clutter as one impassable blob).

---

## References

- [ROBOTIS MuJoCo Menagerie (TurtleBot3)](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
