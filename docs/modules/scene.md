# Scene Module

**Package:** `fret.scene`  
**Source:** `src/fret/scene/`  
**Tests:** `tests/scene/`

> Dubins race structures and bootstrap SCARA obstacles feed this module from
> YAML/MuJoCo. See [releases.md](../releases.md).

---

## Responsibility

The scene module acquires obstacle geometry, transforms it to the world frame, and
constructs the ARCO `KDTreeOccupancy` model for planning.

---

## Components

### `acquisition.py` — SceneAcquisition

Subscribes to `/obstacle_cloud` (`sensor_msgs/PointCloud2`) published by
`PerceptionBridgeNode`. Transforms received point clouds to the `world` frame
and packages them into an `OccupancyUpdatePayload`.

Key invariants:
- All points are in the `world` frame before leaving this module.
- `OccupancyUpdatePayload.frame_id` is always `"world"`.

**Tests:** `tests/scene/test_acquisition.py`, `tests/integration/test_scene_acquisition.py`

---

### `acquisition_node.py` — SceneAcquisitionNode (Level 4)

ROS 2 node wrapper for `SceneAcquisition`. Subscribes to `/obstacle_cloud` and
maintains the latest `OccupancyUpdatePayload` for consumption by the planner.

---

### `occupancy_adapter.py` — OccupancyAdapter

Adapts an `OccupancyUpdatePayload` into the ARCO `KDTreeOccupancy` interface.
Acts as the FRET-side bridge between the scene acquisition layer and ARCO's planner.

```python
adapter = OccupancyAdapter()
adapter.update(payload)           # update with new obstacle cloud
checker = CSpaceChecker(adapter)  # pass to planner
```

Duck-typed: no direct ARCO import required. When ARCO is absent, stores points
internally and provides a distance-based query fallback.

**Tests:** `tests/scene/test_occupancy_adapter.py`

---

### `workspace_occupancy.py` — WorkspaceOccupancyBuilder

Builds a dense voxel-grid occupancy map of the full SCARA reachable workspace
by sampling a regular 3-D grid at configurable resolution (default 20 cm).

**API:**

```python
builder = WorkspaceOccupancyBuilder(resolution=0.20)
kdtree = builder.build(payload)         # classify all voxels
builder.is_occupied(x, y, z)            # True if voxel is occupied
builder.occupied_centres()              # (N, 3) array of occupied centres
builder.free_centres()                  # (M, 3) array of free centres
builder.clearance(position)             # signed distance (negative = inside obstacle)
```

**Default grid bounds** (SCARA reachable envelope):

| Axis | Lower | Upper | Cells at 20 cm |
|---|---|---|---|
| X | −0.60 m | +0.60 m | 7 |
| Y | −0.60 m | +0.60 m | 7 |
| Z | 0.00 m | +0.40 m | 3 |

Total: **147 voxels**. Only voxels within the annular reachable workspace
(`|L1 − L2|` ≤ r ≤ `L1 + L2` = 0.05 m ≤ r ≤ 0.60 m) are evaluated; others
are unconditionally free.

**Voxel occupancy criterion:** a voxel is occupied if the nearest obstacle point
is within the voxel circumradius (`resolution × √3 / 2 ≈ 0.17 m` at 20 cm).

**Tests:** `tests/scene/test_workspace_occupancy.py` — 20 unit tests

---

## Data Flow

```
PerceptionBridgeNode  ──► /obstacle_cloud (PointCloud2, 1 Hz)
    │
    ▼
SceneAcquisition
    │  OccupancyUpdatePayload (world frame, (N, 3) float64)
    ▼
OccupancyAdapter
    │  KDTreeOccupancy (ARCO)
    ▼
CSpaceChecker (planning module)
```

---

## Satisfies Requirements

| Requirement | Description |
|---|---|
| FR-SCN-01 | Subscribe to `/world_state` (via `/obstacle_cloud` bridge) |
| FR-SCN-02 | Transform all point cloud data to `world` frame |
| FR-SCN-03 | Maintain a `KDTreeOccupancy` from obstacle point cloud |
| FR-SCN-04 | Update occupancy without blocking planning or control |
