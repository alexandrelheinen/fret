# FRET Milestones — Perception and Autonomous Obstacle Avoidance

> **Context:** This document extends the three-milestone SITL delivery plan in
> [`docs/delivery_plan.md`](docs/delivery_plan.md).  Milestones 1–3 of that plan
> bring the SCARA arm from a straight-line controller test to a full ARCO-planned
> trajectory executed in an empty Gazebo world.  Milestones 4 and 5 below add
> real workspace-level perception and then close the loop with autonomous
> collision-aware motion.
>
> **Stop condition:** end of Milestone 5 — the SCARA arm starts from a rest pose,
> discovers two cylindrical pillars via its occupancy map, plans a collision-free
> path at constant z using ARCO, and executes it in Gazebo.

---

## Milestone 4 — 3-D workspace occupancy from Gazebo + ARCO container + matplotlib validation

### Goal

Build a dense occupancy map of the full SCARA reachable workspace by sampling a
regular 3-D grid at **20 cm resolution** and asking Gazebo whether each voxel
centre is occupied.  Store the result in an ARCO `KDTreeOccupancy` container.
Provide a standalone validation script that:

1. Renders the occupancy data with **matplotlib** (3-D scatter: occupied voxels
   vs. free voxels, colour-coded).
2. Captures a **screenshot of the Gazebo scene** around the robot.
3. Exposes a programmatic `is_occupied(x, y, z) → bool` query so that the
   planning layer (MS-2/MS-3) can use the workspace map as a collision predicate.

### SCARA workspace bounds used for the grid

The grid must cover the full reachable envelope of the SCARA (derived from
`src/fret/control/kinematics.py`):

| Axis | Lower | Upper | # cells at 20 cm |
|------|-------|-------|-----------------|
| X    | −0.60 m | +0.60 m | 7 |
| Y    | −0.60 m | +0.60 m | 7 |
| Z    | 0.00 m  | +0.40 m | 3 |

Total grid size: 7 × 7 × 3 = **147 voxels**.  Only voxels whose centre falls
within the annular reachable workspace (`|L1 − L2|` ≤ r ≤ `L1 + L2` =
0.05 m ≤ r ≤ 0.60 m) are evaluated; the rest are unconditionally free.

### How occupancy is determined from Gazebo

FRET already publishes obstacle geometry via `PerceptionBridgeNode` on
`/obstacle_cloud` (`sensor_msgs/PointCloud2`, 1 Hz).  The workspace occupancy
builder **reuses this live cloud** rather than adding a new Gazebo interface:

```
/obstacle_cloud  ──►  WorkspaceOccupancyBuilder.build()
                            │  sample 20-cm grid
                            │  for each voxel centre p:
                            │      nearest = KDTreeOccupancy.clearance(p)
                            │      occupied = nearest < voxel_half_diagonal (≈ 0.17 m)
                            ▼
                        KDTreeOccupancy (occupied centres only)
```

`WorkspaceOccupancyBuilder` is a pure-Python class (no ROS dependency) that
receives an `OccupancyUpdatePayload` and returns the sampled map.  The owning
ROS node feeds it the payload obtained from `SceneAcquisition`.

### Acceptance criteria

1. `scripts/validate_occupancy.py` runs without error on a live Gazebo session
   (or on a saved ROS bag).
2. The matplotlib figure shows occupied voxels (red) and free voxels (light grey)
   as a 3-D scatter plot, with correct axis labels and a title that includes the
   sampling resolution.
3. A PNG screenshot of the Gazebo viewport is saved alongside the matplotlib
   figure with filename `gazebo_scene_<timestamp>.png`.
4. `WorkspaceOccupancyBuilder.is_occupied(x, y, z)` returns `True` for a point
   known to be inside an obstacle and `False` for a point in free space.
5. Unit test coverage for `WorkspaceOccupancyBuilder` ≥ 90 %.
6. The `KDTreeOccupancy` built by the builder passes all existing
   `OccupancyAdapter` unit tests when injected as a mock occupancy.

### Deliverables (V-cycle ordered)

**Level 1 — Spec (done above as acceptance criteria)**

**Level 2 — Architecture**

- Add `src/fret/scene/workspace_occupancy.py`:
  `WorkspaceOccupancyBuilder` class — pure Python, no ROS imports.
  Public API:
  ```python
  class WorkspaceOccupancyBuilder:
      def __init__(
          self,
          resolution: float = 0.20,          # voxel edge [m]
          x_bounds: tuple[float, float] = (-0.60, 0.60),
          y_bounds: tuple[float, float] = (-0.60, 0.60),
          z_bounds: tuple[float, float] = (0.00, 0.40),
      ) -> None: ...

      def build(self, payload: OccupancyUpdatePayload) -> KDTreeOccupancy:
          """Sample the grid and return the occupied-voxel KDTreeOccupancy."""
          ...

      def is_occupied(self, x: float, y: float, z: float) -> bool:
          """True if the voxel containing (x, y, z) is occupied."""
          ...

      def occupied_centres(self) -> np.ndarray:
          """Shape (N, 3) array of occupied voxel centres."""
          ...

      def free_centres(self) -> np.ndarray:
          """Shape (M, 3) array of free voxel centres."""
          ...
  ```
- Add `scripts/validate_occupancy.py`:
  standalone script; accepts `--bag` (path to a ROS bag) or connects to a live
  ROS session; builds the occupancy map; renders the matplotlib figure; saves
  the Gazebo screenshot via `ros2 service call /gazebo/screenshot` or an
  `imageio` grab of the Gazebo window; saves both outputs to `outputs/`.
- No changes to `OccupancyAdapter` or `SceneAcquisition` — `WorkspaceOccupancyBuilder`
  is an additional helper, not a replacement.

**Level 3 — Test files and stubs (write tests first)**

- `tests/scene/test_workspace_occupancy.py`:
  - `test_grid_dimensions`: assert grid has 7 × 7 × 3 = 147 cells at 20 cm.
  - `test_is_occupied_inside_obstacle`: inject a payload with a single point at
    `(0.30, 0.10, 0.20)`; assert `is_occupied(0.30, 0.10, 0.20)` is `True`.
  - `test_is_occupied_free_space`: inject empty payload; assert all voxels free.
  - `test_build_returns_kdtree`: assert return type is `arco.mapping.KDTreeOccupancy`
    (`pytest.importorskip("arco")`).
  - `test_occupied_free_centres_partition`: assert occupied + free = all cells in
    the annular reachable region.
  - `test_resolution_parameter`: build with 10 cm resolution; assert ≥ 147 cells.

**Level 4 — Implementation**

- Implement `WorkspaceOccupancyBuilder`:
  - Generate the full grid (`np.meshgrid` + flatten).
  - Mask out cells outside the annular reachable workspace.
  - Build a temporary `KDTreeOccupancy` from the payload obstacle points.
  - Classify each remaining voxel: occupied if nearest obstacle point is within
    `resolution * sqrt(3) / 2` (voxel circumradius).
  - Store occupied/free arrays; build and return the occupied `KDTreeOccupancy`.
- Implement `scripts/validate_occupancy.py`:
  - Argument `--bag <path>` (optional): replay bag to feed `/obstacle_cloud`.
  - Subscribe to `/obstacle_cloud`, call `SceneAcquisition.get_latest_payload()`,
    pass to `WorkspaceOccupancyBuilder.build()`.
  - 3-D matplotlib scatter (occupied red, free light-grey), axis labels
    `X [m]`, `Y [m]`, `Z [m]`, title `FRET workspace occupancy — 20 cm grid`.
  - Save figure as `outputs/occupancy_<timestamp>.png`.
  - Save Gazebo screenshot: call `ros2 service call /gui/screenshot` if available,
    else use `subprocess` to call `grim` or `scrot` on the Gazebo window; save
    as `outputs/gazebo_scene_<timestamp>.png`.
- Export `WorkspaceOccupancyBuilder` from `fret.scene.__init__`.
- Install `scripts/validate_occupancy.py` in `CMakeLists.txt`
  (`install(PROGRAMS scripts/validate_occupancy.py DESTINATION lib/fret)`).

**Quality gates**

- `pytest tests/scene/test_workspace_occupancy.py -v` — all tests pass.
- `black --check src/ scripts/ && isort --check-only src/ scripts/` passes.
- Manual smoke: `python scripts/validate_occupancy.py` on a live Gazebo session
  produces `outputs/occupancy_*.png` and `outputs/gazebo_scene_*.png`.

---

## Milestone 5 — Two-pillar world and autonomous collision-aware motion at constant z

### Goal

Place two cylindrical pillar obstacles in the Gazebo world, run the full ARCO
planning pipeline (built in MS-2/MS-3), and have the SCARA arm plan and execute
a collision-free path at **constant z** from a start configuration to a goal
configuration, autonomously avoiding the pillars.  No hardcoded path — the path
is computed by ARCO at runtime using the occupancy map from Milestone 4.

### World definition

Two static cylindrical pillars in the SCARA horizontal workspace:

| Pillar | Centre (world frame) | Radius | Height |
|--------|---------------------|--------|--------|
| `pillar_a` | `(0.25, 0.10, 0.0)` | 0.04 m | 0.40 m |
| `pillar_b` | `(−0.15, 0.30, 0.0)` | 0.04 m | 0.40 m |

Both pillars stand on the floor (`z = 0`) and are tall enough to intersect the
arm's constant-z operating plane.  They are added to a new Gazebo world file
`src/fret/worlds/pillar_scenario.sdf` (based on the existing
`arco_scenario.sdf` structure).

### Motion parameters

| Parameter | Value |
|-----------|-------|
| Start configuration | `[0.0, 0.0, 0.10]` (joint space, home) |
| Goal configuration | `[1.047, 0.0, 0.10]` (≈ 60°, 0°, 5 cm extension) |
| Fixed joint extension `q3` | 0.10 m throughout (constant z plane) |
| Planning algorithm | ARCO SST with `CSpaceChecker` + workspace occupancy from MS-4 |
| Max planning time | 30 s |
| Trajectory duration | ≤ 20 s |

Constant-z constraint is enforced by:
1. Locking `q3 = 0.10` in the scenario YAML (`start_configuration[2] == goal_configuration[2]`).
2. Confirming ARCO SST samples only configurations with `q3 = 0.10` (achieved by
   setting the lower and upper bound for the prismatic joint to the same value in
   the SST bounding box passed to the planner).

### Acceptance criteria

1. `ros2 launch fret sitl.py scenario:=pillar_avoidance model:=scara` starts
   without errors; both pillars are visible in Gazebo and RViz.
2. `WorkspaceOccupancyBuilder` correctly marks voxels overlapping the pillars as
   occupied (verified by `is_occupied` calls in the integration test).
3. `PlannerNode` finds a collision-free path within 30 s; no waypoint's FK position
   is within 0.05 m of either pillar surface.
4. `ControllerNode` tracks the trajectory; EE error ≤ 5 mm throughout.
5. The EE reaches the goal configuration within the trajectory duration.
6. No `/fault` message is published.
7. Scenario SC-02 (Obstacle Avoidance) passes end-to-end with the pillar world
   (replaces the box obstacle in the original scenario YAML).
8. The ROS bag for the run contains `/joint_states`, `/joint_commands`,
   `/joint_trajectory`, and `/obstacle_cloud` with no gaps.

### Deliverables (V-cycle ordered)

**Level 1 — Spec (done above as acceptance criteria)**

**Level 2 — Architecture**

- `src/fret/worlds/pillar_scenario.sdf`: new Gazebo world with ground plane, two
  cylinders, lighting, and the SCARA spawn point.  No gate or box structures from
  the old `arco_scenario.sdf`; the SDF must be self-contained.
- `src/fret/config/scenarios/pillar_avoidance.yml`: new scenario YAML with
  `world.file: worlds/pillar_scenario.sdf`, two obstacle definitions for
  `PerceptionBridgeNode` (so the static pillars are injected into `/obstacle_cloud`
  without requiring a depth sensor), and goal/start as above.
- `src/fret/config/perception.yaml` (update): add pillar definitions
  (`type: cylinder`, `radius`, `height`, `pose`) alongside the existing box entries.
- `PerceptionBridgeNode` (extend): add cylinder surface sampling in addition to
  the existing box sampling, so cylindrical obstacles are correctly represented in
  `/obstacle_cloud`.
- The occupancy map from MS-4 (`WorkspaceOccupancyBuilder`) is used as the
  collision predicate for ARCO SST; `CSpaceChecker` is updated to use it when
  available, falling back to `KDTreeOccupancy` directly from `OccupancyAdapter`.

**Level 3 — Test files and stubs (write tests first)**

- `tests/scene/test_workspace_occupancy_pillars.py`:
  - `test_pillars_marked_occupied`: inject payload points sampled from pillar
    surfaces; assert voxels at pillar centres are occupied.
  - `test_clearance_around_pillars`: assert voxels ≥ 0.20 m from pillar surfaces
    are free.
- `tests/integration/test_scenario_pillar_avoidance.py`:
  `launch_testing`-based test — asserts acceptance criteria 1–7 with a 45 s
  timeout (30 s planning + 15 s tracking margin).
- `tests/scene/test_perception_bridge_cylinder.py`:
  unit test — inject a cylinder obstacle definition; assert the resulting point
  cloud contains points on the lateral surface and top cap.

**Level 4 — Implementation**

- Create `src/fret/worlds/pillar_scenario.sdf`:
  copy the SCARA SDF base (ground plane + lighting + SCARA URDF include); add two
  `<model>` blocks for `pillar_a` and `pillar_b` using `<geometry><cylinder>`.
- Update `src/fret/config/perception.yaml`: add cylinder entries for both pillars
  with their world-frame poses matching the SDF definition.
- Extend `PerceptionBridgeNode._sample_obstacle()` (or equivalent method) to
  handle `type: cylinder`: sample lateral surface points at uniform azimuth and
  height intervals; add top-cap points.
- Create `src/fret/config/scenarios/pillar_avoidance.yml` following the scenario
  YAML schema from `docs/scenarios.md`; set `q3` to 0.10 in both start and goal;
  set `planner.planning_timeout: 30.0`.
- Update `CSpaceChecker` to accept a `WorkspaceOccupancyBuilder` instance (optional
  constructor argument); if provided, delegate `is_collision_free` to
  `builder.is_occupied(FK(q).position)` in addition to the `KDTreeOccupancy`
  point-distance check.
- Install `src/fret/worlds/` in `CMakeLists.txt`:
  `install(DIRECTORY worlds DESTINATION share/fret)`.

**Quality gates**

- `pytest tests/ -v` — all tests pass (≥ 90 % coverage).
- `black --check src/ && isort --check-only src/` passes.
- `ros2 launch fret sitl.py scenario:=pillar_avoidance` smoke test:
  - Both pillars appear in Gazebo.
  - `/obstacle_cloud` contains points at known pillar positions.
  - ARCO returns `SUCCEEDED` within 30 s.
  - SCARA EE reaches the goal within 20 s without entering a 0.05 m exclusion
    zone around either pillar.
- All GitHub workflows (`formatting.yml`, `tests.yml`, `type_check.yml`) pass.

---

## Milestone sequence — full picture

```
MS-1  ControllerNode ROS wiring + straight-line (3 s, constant z, no ARCO)
  ▼
MS-2  ARCO pipeline → /joint_trajectory (publish only, empty world)
  ▼
MS-3  Full SITL — ARCO trajectory executed by controller (20 s, empty world)
  ▼
MS-4  Workspace occupancy map (20-cm grid, ARCO KDTreeOccupancy, matplotlib validation)
  │   ∘ WorkspaceOccupancyBuilder (pure Python, no ROS)
  │   ∘ scripts/validate_occupancy.py (scatter plot + Gazebo screenshot)
  │   ∘ is_occupied(x, y, z) → bool query interface
  ▼
MS-5  Pillar world + autonomous collision-aware motion at constant z
        ∘ pillar_scenario.sdf (two cylinders)
        ∘ PerceptionBridgeNode cylinder sampling
        ∘ pillar_avoidance.yml (constant q3 = 0.10)
        ∘ ARCO SST + WorkspaceOccupancyBuilder as collision predicate
        ∘ SC-02 end-to-end integration test (pillar variant)
```

---

## Dependency and risk notes

| Risk | Milestone | Mitigation |
|------|-----------|------------|
| 20-cm grid is too coarse to detect 4-cm pillar radii | MS-4, MS-5 | `WorkspaceOccupancyBuilder` uses voxel circumradius (≈ 17 cm) as occupancy threshold; pillars at 4 cm radius will mark ≥ 1 neighbouring voxel; add safety margin of 1 voxel around obstacles in `CSpaceChecker` |
| Gazebo screenshot API varies by version | MS-4 | Try `/gui/screenshot` service first; fall back to `scrot`/`grim`; document both in `validate_occupancy.py` header |
| Cylinder sampling in `PerceptionBridgeNode` may be slow at high density | MS-5 | Sample at 5° azimuth × 5 cm height intervals (≈ 72 × 8 = 576 pts per pillar); acceptable for 1 Hz publication |
| ARCO SST may time out finding a path between two pillars | MS-5 | Ensure pillar centres are ≥ 0.30 m from each other and from start/goal FK positions; validate clearance manually before running SITL |
| Constant-z constraint via joint-bound clamping may conflict with ARCO SST sampler | MS-5 | Set SST bounding box `q3_lo = q3_hi = 0.10`; test in isolation before full SITL run |

---

## References

- Full 3-milestone SITL delivery plan: [`docs/delivery_plan.md`](docs/delivery_plan.md)
- Scenario library and pass criteria: [`docs/scenarios.md`](docs/scenarios.md)
- Interface contracts: [`docs/interfaces.md`](docs/interfaces.md)
- ARCO integration boundary: [`docs/arco.md`](docs/arco.md)
- Coding standards and V-cycle: [`docs/guidelines.md`](docs/guidelines.md)
