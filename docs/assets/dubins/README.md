# Dubins Race Visual Asset Benchmark (SC-v11)

This document benchmarks **free, copy-friendly** mobile-robot and warehouse
assets for the Dubins dual-race showcase (`dubins_race.xml`). It mirrors the
PPP warehouse workflow: vendored meshes for visuals, analytic boxes for
planning/collision, AWS ground texture for the floor.

---

## Selection criteria

| Criterion | Requirement |
|---|---|
| License | MIT, Apache-2.0, BSD, or MIT-0 (commercial-friendly) |
| Format | OBJ/STL/DAE usable in MuJoCo `<mesh>` |
| Kinematics | Car-like / differential or Ackermann base suitable for Dubins SE(2) |
| Scale | Footprint roughly 0.5–1.0 m long after scaling (matches `vehicle.radius`) |
| Integration | No absolute host paths; meshes vendored under `src/fret/mjcf/assets/` |

---

## Vehicle candidates (free / copiable)

| Asset | Source | License | Footprint (nominal) | Dubins fit | Notes |
|---|---|---|---|---|---|
| **TurtleBot3 Burger** | [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) | Apache-2.0 | ~0.138 m radius | **Best** | Classic diff-drive; meshes + MJCF; scale ≈3× to match 0.42 m planning radius |
| **TurtleBot3 Waffle Pi** | ROBOTIS Menagerie | Apache-2.0 | ~0.20 m radius | Good | Wider base; slower turns in tight aisles |
| **Hello Robot Stretch 2 base** | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/hello_robot_stretch) | BSD-3-Clause-Clear | ~0.34 m wide | Fair | Mobile manipulator; visually rich but oversized for a race |
| **Stanford TidyBot base** | MuJoCo Menagerie | MIT | ~0.35 m | Fair | Research mobile manipulator; good meshes |
| **Fetch Freight base** | [fetch_freight_mujoco](https://github.com/JChunX/fetch_freight_mujoco) | Check repo | ~0.40 m | Good | Freight holonomic base; needs wheel cleanup |
| **Robot soccer kit (omni)** | MuJoCo Menagerie | MIT | Small | Poor | Omnidirectional — not Dubins-like |
| **Procedural box + wheels (current)** | FRET `dubins_race.xml` | N/A | 0.72 × 0.44 m | Baseline | Matches planning envelope exactly; blob-like visuals |

### Pick: **TurtleBot3 Burger (ROBOTIS Menagerie)**

**Why:** smallest well-maintained diff-drive MJCF, Apache-2.0, widely recognized
as a warehouse / lab robot, easy to scale and tint (blue RRT*, green SST).

**Planned integration:**

1. Vendor `burger_base.obj` (+ wheels) under `mjcf/assets/dubins/turtlebot3/`
2. Replace procedural chassis geoms with mesh visuals (`contype=0`)
3. Keep invisible box collision matching `vehicle.radius` + `clearance_margin`
4. Scale mesh so outer footprint ≈ 0.84 m (2 × 0.42 m radius)

---

## Obstacle / warehouse candidates

| Asset | Source | License | Use in race | Notes |
|---|---|---|---|---|
| **AWS RoboMaker warehouse** (current) | Already in `mjcf/assets/aws_warehouse/` | MIT-0 | Floor + clutter | `ground.png`, `clutter_*.obj`, `shelf.obj` |
| **MuJoCo Menagerie shelves** | Menagerie props | Per-model | Optional back-wall | Heavier meshes; AWS shelf already used |
| **Procedural boxes** (current) | `dubins_race_obstacles.yml` | N/A | Planning + MJCF collision | Analytic `RectStructureOccupancy` |
| **Clutter meshes as posts** | AWS `clutter_a/c/d.obj` | MIT-0 | Visual-only replacements for `str_*` boxes | Scale to match YAML half-extents |

### Pick: **Hybrid AWS warehouse (same as PPP)**

**Floor:** `assets/aws_warehouse/textures/ground.png` on an 80 × 80 m plane.

**Structures:** keep YAML rectangles for planning; swap procedural column
visuals for scaled AWS clutter meshes (cardboard / crate / pallet) keyed to
structure height tiers:

| Tier | Height range | Visual mesh |
|---|---|---|
| Low | ≤ 2.2 m | `clutter_a` (cardboard) |
| Mid | 2.2 – 3.0 m | `clutter_c` (crate) |
| Tall | > 3.0 m | `clutter_d` (pallet stack) |

**Density rule:** corridor width ≥ `2 × (vehicle.radius + clearance_margin)`
= **1.68 m** minimum between structure faces at planning time. Visual meshes
may be slightly smaller than collision boxes so contact never appears tighter
than the analytic map.

**Back wall:** retain `shelf.obj` along Y ≈ 76 m for depth cues (visual only).

---

## Implementation status

| Item | Status |
|---|---|
| Analytic rectangle occupancy (`RectStructureOccupancy`) | **Done** — replaces sparse KD-tree samples |
| Clearance = vehicle radius + margin (0.42 + 0.42 m) | **Done** |
| Repulsive APF layer (`repulsion_gain`) | **Active** — tuned to 3.0 in `dubins.yml` |
| Executed-path clearance metric | **Done** — `min_obstacle_clearance_m` in E2E runner |
| TurtleBot3 mesh import script | Planned (`scripts/import_dubins_assets.py`) |
| Clutter mesh column visuals | Planned |

---

## Regenerate AWS assets

```bash
python3 scripts/import_aws_warehouse_assets.py
```

## References

- [MuJoCo Menagerie model gallery](https://mujoco.readthedocs.io/en/stable/models.html)
- [ROBOTIS MuJoCo Menagerie (TurtleBot3)](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- FRET PPP asset pipeline: `docs/assets/ppp/README.md`
