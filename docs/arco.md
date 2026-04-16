# ARCO Integration in FRET

## What is ARCO

ARCO (Autonomous Routing, Control, and Observation) is a pure-Python library
for autonomous navigation building blocks. It provides the motion planning
layer that FRET consumes: given an obstacle model of the workspace, ARCO
computes a collision-free path that FRET's control stack then executes.

ARCO repository: `../arco/`

---

## ARCO Capabilities

ARCO is organized in three layers.

### Mapping

Environment representation used by planners to query obstacle proximity.

| Structure | Description |
|---|---|
| `ManhattanGrid` | 4-connected discrete grid, $L_1$ metric |
| `EuclideanGrid` | 8-connected discrete grid, $L_2$ metric |
| `Graph → WeightedGraph → CartesianGraph → RoadGraph` | Layered graph hierarchy for network routing |
| `KDTreeOccupancy` | KD-tree obstacle set answering nearest-obstacle and clearance queries in N-dimensional spaces |

`KDTreeOccupancy` is the primary interface for continuous-space planning and
the one FRET uses: FRET acquires obstacle geometry from Gazebo, transforms
it to the world frame, and constructs a `KDTreeOccupancy` from the resulting
point set.

### Planning

| Algorithm | Type | Notes |
|---|---|---|
| A* | Discrete (grid / graph) | Optimal; used for grid and road-network routing |
| Route planning | Discrete (road graph) | A* with nearest-node projection and waypoint smoothing |
| RRT* | Sampling-based | Asymptotically optimal; simpler fallback for continuous spaces |
| SST | Sampling-based, kinodynamic | Stable Sparse Trees; asymptotically near-optimal, memory-efficient, designed for systems with dynamics |
| `TrajectoryOptimizer` | Post-processing | Two-stage time-optimal refinement; collision-aware; accepts an IK callable |
| `TrajectoryPruner` | Post-processing | Removes redundant waypoints from a raw planner path |

SST is the preferred algorithm for manipulator planning: it handles kinodynamic
constraints, scales to N-dimensional joint spaces, and converges to a
near-optimal solution while keeping memory bounded.

### Guidance

Post-planning components that shape raw paths into executable trajectories.

| Component | Description |
|---|---|
| `BSplineInterpolator` | C² smooth trajectory from discrete waypoints, arc-length parameterized |
| `PIDController` | Classic setpoint tracking with derivative filtering and anti-windup |
| `PurePursuitController` | Geometric look-ahead path tracker for car-like vehicles |
| `MPCController` | Optimization-based controller with preview horizon and constraint handling |
| `TrackingLoop` | Integration wrapper: timing, state management, consistent interface across controllers |
| `DubinsPrimitive` | Shortest-path kinematic steering for forward-only vehicles (Dubins 1957) |
| `DubinsVehicle` | Car-like kinematic model (used internally by ARCO's 2-D scenarios) |

> Note: the guidance layer's vehicle models are currently 2-D. FRET must
> supply its own kinematics (Jacobian-based control) and use ARCO's controllers
> only if they fit the manipulator model.

### Entity model

A typed, JSON-serializable hierarchy for physical entities in a planning scene:

```
Entity
├── Agent (DubinsAgent, CartesianAgent)
├── Link
├── Joint (RevoluteJoint, PrismaticJoint)
├── EndEffector
└── Object
```

`KinematicChain` assembles Links, Joints, and an EndEffector into a manipulator
description. For the SCARA (RRP topology) this maps to two `RevoluteJoint`s and
one `PrismaticJoint`.

### Middleware and pipeline

ARCO's `arco.middleware` package provides an in-process typed message bus
(`Bus` / `InMemoryBus`) with arc dataclasses (`MappingFrame`, `PlanFrame`,
`GuidanceFrame`) and a `PipelineRunner` that wires nodes to a shared bus.
The pipeline is designed around a file-I/O discipline: each stage reads one
artifact and writes the next, making stages independently restartable.

---

## Integration Architecture

### Ownership boundary

| Responsibility | Owner |
|---|---|
| Occupancy representation (KD-tree construction, update, query API) | ARCO |
| Planner logic (sampling, collision checking, path search, diagnostics) | ARCO |
| Scene acquisition (point cloud capture, frame transform pipeline) | FRET |
| Robot state (joint positions, velocities, forward kinematics) | FRET |
| Control and execution (trajectory interpolation, controller dispatch, feedback) | FRET |
| TF broadcast (`world → base_link → sensor`) | FRET |

ARCO must not read robot state from ROS topics.
FRET must not call ARCO planner internals directly; it only calls the planning
request interface.

### Data flow

```
FRET                                ARCO
────────────────────                ────────────────────────
SceneAcquisition                    OccupancyModel
  │  point cloud (world frame)        │  KD-tree
  ▼                                   ▼
OccupancyUpdatePayload ──────────► occupancy_update()
                                        │
RobotState (joint positions,            │
  TF snapshot)                          ▼
PlanningRequest ────────────────► plan()
                                        │
                                        ▼
PlanningResult ◄────────────────  (path, status, diagnostics)
  │
  ▼
TrajectoryConverter ─────────────► ControllerDispatch
  │  JointTrajectory (ROS message)
  ▼
ExecutionFeedback ◄─────────────── TrackingMonitor
```

### Frame convention

All data crossing the integration boundary must be expressed in the `world`
frame. ARCO never performs TF lookups. FRET is responsible for resolving all
transforms before sending any payload to ARCO.

| Frame | Owner | Rate |
|---|---|---|
| `world → base_link` | FRET | 100 Hz |
| `world → sensor_frame` | FRET | 100 Hz |
| `base_link → tool0` | FRET (FK) | 100 Hz; not sent to ARCO |

---

## Integration Potential Assessment

### What maps cleanly

**Occupancy model.** `KDTreeOccupancy` is a direct fit: FRET acquires a Gazebo
point cloud, transforms it to `world`, and hands it to ARCO's constructor.
No adaptation is needed at the interface level.

**Sampling planners.** SST and RRT* accept an N-dimensional bounding box and
an `Occupancy` instance. As long as FRET supplies an occupancy model — either
in Cartesian task space or in joint configuration space — the planners are
immediately usable.

**Trajectory post-processing.** `TrajectoryOptimizer` accepts a callable IK
function and a feasibility predicate, making it model-agnostic. Combined with
`BSplineInterpolator`, the chain raw-path → optimized → smooth is directly
applicable before handing off to FRET's controller.

**Entity model.** `KinematicChain` with `RevoluteJoint` and `PrismaticJoint`
matches the SCARA RRP topology and provides a planning-side robot description
decoupled from URDF.

### The central open question: planning space

ARCO planners operate in whatever N-dimensional space they are given.
The key architectural decision for the refactor is:

**Option A — Task-space planning (default, simpler):** Sample EE positions in
3-D world space; use ARCO's `KDTreeOccupancy` directly on obstacle point clouds.
FRET resolves IK only at execution time. Simple to integrate but incomplete
near singularities and for self-collision avoidance.

**Option B — Configuration-space planning (correct for manipulation):** Sample
joint configurations; collision check by running FK → checking world-space
clearance via `KDTreeOccupancy`. This requires a FRET-side C-space collision
checker adapter, but handles self-collisions and singularities correctly.

The `TrajectoryOptimizer` already accepts an IK callable as a hook — that is
the right seam for bridging the two options.

### What ARCO does not yet provide for FRET

| Gap | Impact | Mitigation |
|---|---|---|
| No 3-D manipulator kinematic primitive | ARCO's `DubinsPrimitive` / `DubinsVehicle` are 2-D car-like | FRET supplies Jacobian-based kinematics; ARCO planners are used without ARCO's guidance primitives |
| No C-space collision checker | Planners operate on point-cloud occupancy, not joint-space | FRET adapter: FK call + `KDTreeOccupancy` query wrapped as a custom `Occupancy` subclass |
| No replanning loop | ARCO is a one-shot planner | FRET implements replanning triggers; ARCO is re-invoked; the `arco.middleware` Bus is the right foundation |

### Summary

ARCO provides the correct primitives — occupancy model, sampling planners,
trajectory optimizer, B-spline interpolation — for FRET's planning stage
(roadmap step 5). The integration boundary is clean and the ownership split is
well-defined. The main refactoring work is:

1. Decide on planning space (task-space vs. C-space) and implement the
   corresponding collision-check adapter in FRET.
2. Wire ARCO's one-shot planning output into FRET's ROS 2 execution cycle
   with proper replanning triggers and scene-update logic.
3. Clarify which ARCO guidance components (interpolation, controllers) are
   reused in FRET and which are replaced by FRET's own Jacobian-based stack.
