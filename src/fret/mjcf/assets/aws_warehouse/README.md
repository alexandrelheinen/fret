# AWS RoboMaker warehouse assets (MIT-0)

Visual meshes and textures for the FRET PPP warehouse MuJoCo scene.

| File | Source model | Use in `ppp_warehouse.xml` |
|---|---|---|
| `meshes/shelf.obj` | `aws_robomaker_warehouse_ShelfD_01` | Back-wall storage racks (visual) |
| `meshes/clutter_a.obj` | `aws_robomaker_warehouse_ClutteringA_01` | Obstacle `obs_a` visual |
| `meshes/clutter_c.obj` | `aws_robomaker_warehouse_ClutteringC_01` | Obstacle `obs_b` visual |
| `meshes/clutter_d.obj` | `aws_robomaker_warehouse_ClutteringD_01` | Obstacle `obs_c` visual |
| `textures/ground.png` | `aws_robomaker_warehouse_GroundB_01` | Warehouse floor |
| `textures/wall.png` | `aws_robomaker_warehouse_WallB_01` | Back wall |

**License:** [MIT-0](LICENSE) — [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

**Regenerate:**

```bash
python3 scripts/import_aws_warehouse_assets.py
```

Collision geometry for planning remains box primitives aligned with
`src/fret/config/perception_ppp_warehouse.yaml`. The gantry stays procedural.
