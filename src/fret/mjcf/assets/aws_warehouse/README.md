# AWS RoboMaker warehouse assets (MIT-0)

Visual meshes and textures for FRET Dubins race MuJoCo scenes.

| File | Source model | Use in `dubins_race.xml` |
|---|---|---|
| `meshes/shelf.obj` | `aws_robomaker_warehouse_ShelfD_01` | Back-wall storage racks (visual) |
| `meshes/clutter_a.obj` | `aws_robomaker_warehouse_ClutteringA_01` | Structure / column visuals |
| `meshes/clutter_c.obj` | `aws_robomaker_warehouse_ClutteringC_01` | Structure / column visuals |
| `meshes/clutter_d.obj` | `aws_robomaker_warehouse_ClutteringD_01` | Structure / column visuals |
| `textures/ground.png` | `aws_robomaker_warehouse_GroundB_01` | Warehouse floor |
| `textures/wall.png` | `aws_robomaker_warehouse_WallB_01` | Back wall |

**License:** [MIT-0](LICENSE) — [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

**Regenerate:**

```bash
python3 scripts/import_aws_warehouse_assets.py
```

Collision geometry for planning remains analytic rectangles in
`src/fret/config/worlds/dubins_race_obstacles.yml`.
