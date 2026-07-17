# Dubins Race Visual Asset Benchmark (SC-v11)

Free, copy-friendly assets for the compact Dubins dual-race showcase
(`dubins_race.xml`) — **real TurtleBot3 Burger** on a **10 m × 10 m** lab floor.

---

## Selection criteria

| Criterion | Requirement |
|---|---|
| License | MIT, Apache-2.0, BSD, or MIT-0 |
| Format | OBJ/STL/DAE usable in MuJoCo `<mesh>` |
| Scale | Real TB3 (~0.14 m radius); arena finishable at ≈ 0.22 m/s |
| Integration | Meshes vendored under `src/fret/mjcf/assets/` |

---

## Vehicle candidates

| Asset | Source | License | Fit | Notes |
|---|---|---|---|---|
| **TurtleBot3 Burger** | [ROBOTIS Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) | Apache-2.0 | **In use** | Real wheel hinges + actuators; unit + race |
| TurtleBot3 Waffle Pi | ROBOTIS Menagerie | Apache-2.0 | Optional | Wider; slower in narrow aisles |
| Hakoniwa TB3 + LiDAR | [hakoniwa-mujoco-robots](https://github.com/hakoniwalab/hakoniwa-mujoco-robots) | MIT | Reference | Good LiDAR ideas; heavier runtime |
| mujoco-maze worlds | [kngwyu/mujoco-maze](https://github.com/kngwyu/mujoco-maze) | Apache-2.0 | Ideas only | Procedural mazes — no new meshes needed |
| Stretch / TidyBot bases | MuJoCo Menagerie | BSD / MIT | Too large | Keep for mobile-manipulator demos later |

**Pick:** real-scale TurtleBot3 Burger (already vendored via
`scripts/import_turtlebot3_assets.py`).

---

## Obstacle / floor candidates

| Asset | Source | License | Use | Notes |
|---|---|---|---|---|
| **AWS RoboMaker clutter + ground** | Already in `mjcf/assets/aws_warehouse/` | MIT-0 | **In use** | Same meshes, scaled down to lab structures |
| Procedural boxes | `dubins_race_obstacles.yml` | — | Planning + collision | Analytic `RectStructureOccupancy` |
| AWS small-warehouse Gazebo world | aws-robotics | MIT-0 | Not adopted | Full Gazebo pack is heavier than needed |

**Pick:** keep hybrid AWS visuals on a compact analytic layout. No new mesh
vendoring required — regenerate layout + MJCF:

```bash
python3 scripts/generate_dubins_warehouse_layout.py
python3 scripts/generate_dubins_race_mjcf.py
```

### Lab sizing (real TB3)

| Parameter | Value |
|---|---|
| Workspace | 10 m × 10 m |
| Start → goal | (1.2, 1.2) → (8.8, 8.8) |
| Vehicle radius / clearance | 0.12 m / 0.18 m |
| Wheel ctrl limit | ±6.67 rad/s (≈ 0.22 m/s) |
| Cruise / max speed | 0.16 / 0.22 m/s |
| Race timeout | 180 s |

Corridor face-to-face gaps are intentionally narrow (~0.7–0.9 m) but stay
above `2 × (radius + clearance)` at planning time.

---

## Implementation status

| Item | Status |
|---|---|
| Real TB3 unit sandbox (`turtlebot3_unit`) | **Done** |
| Compact race world + Menagerie wheel limits | **Done** |
| AWS clutter visuals on analytic boxes | **Done** |
| Showcase-scale fake wheel speeds | **Removed** |

---

## References

- [ROBOTIS MuJoCo Menagerie (TurtleBot3)](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
- [mujoco-maze](https://github.com/kngwyu/mujoco-maze) (layout ideas)
