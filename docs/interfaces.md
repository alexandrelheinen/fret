# FRET Interface Contracts

This document defines the typed data structures and behavioral contracts at every
module boundary in FRET. It is the **Level 2** artifact: produced alongside the
architecture and consumed directly by Level 3 module stubs and integration tests.

All Python types below are dataclasses. Physical quantity naming follows
[docs/guidelines.md](guidelines.md). All data crossing the FRET/ARCO boundary must
be in the `world` frame — ARCO performs no TF lookups.

---

## Enumeration Types

### `PlanningStatus`

```python
import enum

class PlanningStatus(enum.IntEnum):
    SUCCESS   = 0
    ABORTED   = 1
    CANCELLED = 2
```

### `ErrorCode`

```python
class ErrorCode(enum.IntEnum):
    NONE                 = 0
    TIMEOUT              = 1  # Planning exceeded the time budget (FR-PLN-04)
    NO_PATH_FOUND        = 2  # Planner exhausted iterations without finding a path
    INVALID_CONFIGURATION = 3  # Start or goal violates joint limits (FR-PLN-07)
    POST_PROCESS_FAILED  = 4  # Trajectory post-processing raised an exception
    INTERNAL_ERROR       = 99
```

---

## Scene → Planning Boundary

### `OccupancyUpdatePayload`

Carries the obstacle representation from the Scene Acquisition layer to the Planning
layer. Constructed by `scene/occupancy_adapter.py`; consumed by `planning/cspace_checker.py`.

```python
@dataclass
class OccupancyUpdatePayload:
    obstacle_points: np.ndarray  # shape (N, 3), dtype float64, world frame
    timestamp: float             # POSIX seconds
    frame_id: str                # Must always be "world"
```

**Invariants:**
- `obstacle_points.ndim == 2` and `obstacle_points.shape[1] == 3`
- `frame_id == "world"` — enforced by the scene layer before construction
- `N >= 0` (empty array is valid: no obstacles in scene)

---

## Control → Planning Boundary

### `RobotState`

Snapshot of joint state at a given instant. Produced by `control/state_estimator.py`;
consumed by `planning/cspace_checker.py` at planning request time.

```python
@dataclass
class RobotState:
    joint_positions: np.ndarray   # shape (DOF,), radians or meters
    joint_velocities: np.ndarray  # shape (DOF,), rad/s or m/s
    joint_names: list[str]        # length DOF, matches URDF joint name order
    timestamp: float              # POSIX seconds
```

**Invariants:**
- `len(joint_positions) == len(joint_velocities) == len(joint_names)`
- Each `joint_positions[i]` is within the declared limit for `joint_names[i]`

---

## Task → Planning Boundary (Action Goal / Result)

### `PlanningRequest`

Input to the Planning layer. Maps one-to-one to the Action goal fields.

```python
@dataclass
class PlanningRequest:
    start_configuration: np.ndarray  # shape (DOF,)
    goal_configuration: np.ndarray   # shape (DOF,)
    planning_timeout: float          # seconds; must be > 0
    scenario_id: str                 # informational; used in logs and feedback
```

**Invariants:**
- Both configurations within the operational envelope joint limits
- `planning_timeout > 0`
- `start_configuration.shape == goal_configuration.shape == (DOF,)`

### `PlanningResult`

Output of the Planning layer. Maps one-to-one to the Action result fields.

```python
@dataclass
class PlanningResult:
    status: PlanningStatus
    path: list[np.ndarray]    # each element shape (DOF,); empty if status != SUCCESS
    error_code: ErrorCode
    planning_duration: float  # seconds
    iteration_count: int
```

**Invariants:**
- `status == SUCCESS` implies `len(path) >= 2`
- `status != SUCCESS` implies `len(path) == 0`
- `error_code == ErrorCode.NONE` iff `status == SUCCESS`

---

## ROS 2 Action Definition: `PlanRequest.action`

File location: `src/fret/action/PlanRequest.action`

```
# Goal — mirrors PlanningRequest
float64[] start_configuration
float64[] goal_configuration
float64   planning_timeout
string    scenario_id
---
# Result — mirrors PlanningResult
uint8     status            # PlanningStatus enum value
uint8     error_code        # ErrorCode enum value
float64[] path_flat         # row-major flattened path; length = waypoint_count * dof
int32     waypoint_count
int32     dof
float64   planning_duration
int32     iteration_count
---
# Feedback
int32     iteration_count
float64   current_cost
float64   elapsed_time
```

---

## ROS 2 Topic QoS Assignments

Full profile specifications for every topic in the system. Implementation must
use exactly these profiles to ensure correct publisher/subscriber matching.

| Topic | Message Type | Reliability | Durability | Depth | Rationale |
|---|---|---|---|---|---|
| `/world_state` | `sensor_msgs/PointCloud2` | RELIABLE | TRANSIENT_LOCAL | 1 | Latched: late-joining nodes receive current world state |
| `/joint_states` | `sensor_msgs/JointState` | BEST_EFFORT | VOLATILE | 10 | High-rate sensor; losing a sample is acceptable |
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | RELIABLE | VOLATILE | 1 | Must not be dropped; stale trajectories are discarded |
| `/joint_commands` | `std_msgs/Float64MultiArray` | BEST_EFFORT | VOLATILE | 1 | Real-time; always prefer the freshest sample |
| `/replan_trigger` | `std_msgs/Bool` | RELIABLE | VOLATILE | 1 | Event-driven; exactly-once delivery required |
| `/fault` | `std_msgs/String` | RELIABLE | TRANSIENT_LOCAL | 10 | Diagnostic; late subscribers must see recent faults |
| `/tf` and `/tf_static` | `tf2_msgs/TFMessage` | BEST_EFFORT | VOLATILE | 100 | Standard ROS 2 TF2 convention |

---

## Node State Machines

### `PlannerNode` — Action Server FSM

**States:** `IDLE`, `PLANNING`, `POST_PROCESSING`, `SUCCEEDED`, `ABORTED`, `CANCELLED`

```
                    ┌─────────────────────────────────────────────────────┐
                    │                      IDLE                           │
                    │  Waiting for an Action goal                         │
                    └────────────────────────┬────────────────────────────┘
                                             │ Goal received
                              ┌──────────────┴──────────────────┐
                    config    │                                 │ config
                    invalid   │                                 │ valid
                              ▼                                 ▼
                    ┌──────────────────┐          ┌──────────────────────────┐
                    │    ABORTED       │          │        PLANNING          │
                    │ INVALID_CONFIG   │          │  ARCO running; feedback  │
                    └──────────────────┘          │  published each iteration│
                                                  └──┬──────────┬────────────┘
                                                     │          │          │
                                           path OK   │  timeout │  cancel  │
                                                     │  or fail │  request │
                                                     ▼          ▼          ▼
                                        ┌───────────────┐  ┌─────────┐  ┌──────────┐
                                        │POST_PROCESSING│  │ ABORTED │  │CANCELLED │
                                        │ pruner +      │  │ error   │  │ path=[]  │
                                        │ optimizer +   │  │ code    │  │          │
                                        │ B-spline      │  │ path=[] │  │          │
                                        └──────┬───┬────┘  └────┬────┘  └────┬─────┘
                                               │   │ error      │            │
                                           OK  │   └────────────┴────────────┘
                                               ▼              all → IDLE
                                       ┌───────────────┐
                                       │  SUCCEEDED    │
                                       │ traj published│
                                       │  → IDLE       │
                                       └───────────────┘
```

**Transition table:**

| From | Event | To | Side effect |
|---|---|---|---|
| IDLE | Goal received; configuration valid | PLANNING | Invoke ARCO planner asynchronously |
| IDLE | Goal received; configuration invalid | ABORTED | Return `INVALID_CONFIGURATION`; no ARCO call |
| PLANNING | ARCO returns non-empty path | POST_PROCESSING | Run post-processing chain |
| PLANNING | Timeout elapsed | ABORTED | Return `TIMEOUT`; `path=[]` |
| PLANNING | ARCO returns empty path | ABORTED | Return `NO_PATH_FOUND`; `path=[]` |
| PLANNING | Cancel received from client | CANCELLED | Interrupt ARCO; `path=[]` |
| POST_PROCESSING | Chain completes without exception | SUCCEEDED | Publish `JointTrajectory`; return path |
| POST_PROCESSING | Chain raises an exception | ABORTED | Return `POST_PROCESS_FAILED`; `path=[]` |
| SUCCEEDED / ABORTED / CANCELLED | — | IDLE | Clear state; ready for next goal |

---

### `ControllerNode` — Periodic Node FSM

**States:** `IDLE`, `TRACKING`, `HALTED`

```
         ┌─────────────────────────────────────────────────┐
         │                   IDLE                          │
         │  No active trajectory; no commands published    │
         └──────────────────────┬──────────────────────────┘
                                │ /joint_trajectory received
                                ▼
         ┌─────────────────────────────────────────────────┐
         │                 TRACKING                        │
         │  50 Hz: error → Jacobian pseudoinverse →        │
         │  /joint_commands; TF2 broadcast maintained      │
         └──────┬──────────────────────────┬───────────────┘
                │ Last waypoint reached     │ EE error > 20 mm
                │                          │ for > 0.5 s
                ▼                          ▼
         ┌─────────────┐         ┌──────────────────────────┐
         │    IDLE     │         │         HALTED           │
         │  (complete) │         │  /fault published        │
         └─────────────┘         │  all joint commands = 0  │
                                 └──────────────┬───────────┘
                                                │ New /joint_trajectory received
                                                │ (operator-triggered reset)
                                                ▼
                                            TRACKING
```

**Transition table:**

| From | Event | To | Side effect |
|---|---|---|---|
| IDLE | `/joint_trajectory` received | TRACKING | Load trajectory; start 50 Hz loop |
| TRACKING | Last waypoint reached and error ≤ 5 mm | IDLE | Stop loop; log completion |
| TRACKING | EE error > 20 mm for > 0.5 s | HALTED | Publish `/fault`; zero all commands |
| HALTED | New `/joint_trajectory` received | TRACKING | Clear fault state; restart loop |

---

## Error Propagation

| Error origin | Detection point | Propagation path | Observable effect |
|---|---|---|---|
| Invalid goal configuration | `PlannerNode` at goal receipt | Action returns `ABORTED` before ARCO is called | `error_code = INVALID_CONFIGURATION` |
| ARCO planning timeout | `PlannerNode` wall-clock monitor | Action returns `ABORTED` | `error_code = TIMEOUT`; `/joint_trajectory` not published |
| ARCO returns no path | `PlannerNode` inspects result | Action returns `ABORTED` | `error_code = NO_PATH_FOUND`; `/joint_trajectory` not published |
| Post-processing exception | `PlannerNode` try/except around chain | Action returns `ABORTED` | `error_code = POST_PROCESS_FAILED` |
| Tracking error threshold exceeded | `ControllerNode` error monitor | Transitions to HALTED | `/fault` published; commands zeroed |
| Simulator disconnect (no `/joint_states`) | `ControllerNode` message age check | Transitions to HALTED if last stamp > 0.5 s old | `/fault` published |

---

## Simulation launch parameters (v1.2)

Passed to `ros2 launch fret sitl.py` or declared in scenario YAML under
`ros__parameters`. Full schema: [config.md](config.md#simulation-physics-v12).

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `model` | string | `scara` | Robot model (`ppp`, `dubins`, …) |
| `scenario` | string | `static_reach` | Scenario YAML stem |
| `physics_mode` | bool | `false` | Enable MuJoCo actuator + `mj_step` SITL (FR-SIM-09) |

When `physics_mode:=true`, `/joint_states` must reflect simulated `qpos`/`qvel`
only — no open-loop pose injection (FR-SIM-07). See
[mujoco_physics_v1.2.md](mujoco_physics_v1.2.md).

---

## ARCO Dependency Contract

**Integration mechanism:** editable local install.

```bash
pip install -e ../arco/
```

**Declared in** `pyproject.toml`:

```toml
[project.optional-dependencies]
arco = ["arco @ file:../arco"]
```

**Minimum API surface FRET depends on:**

| ARCO symbol | Used by FRET module | Purpose |
|---|---|---|
| `arco.mapping.KDTreeOccupancy` | `scene/occupancy_adapter.py` | Obstacle proximity and clearance queries |
| `arco.planning.SST` | `planning/cspace_checker.py` | Primary C-space kinodynamic planner |
| `arco.planning.RRTStar` | `planning/cspace_checker.py` | Fallback asymptotically-optimal planner |
| `arco.planning.TrajectoryPruner` | `planning/trajectory_generator.py` | Redundant waypoint removal |
| `arco.planning.TrajectoryOptimizer` | `planning/trajectory_generator.py` | Time-optimal refinement |
| `arco.guidance.BSplineInterpolator` | `planning/trajectory_generator.py` | C² trajectory smoothing |
| `arco.entity.KinematicChain` | `control/kinematics.py` | Planning-side robot description (optional) |

**CI import validation step** (in `tests.yml`):

```bash
pip install -e ../arco/
python -c "
from arco.mapping import KDTreeOccupancy
from arco.planning import SST, RRTStar, TrajectoryPruner, TrajectoryOptimizer
from arco.guidance import BSplineInterpolator
print('ARCO API surface OK')
"
```

**Version pinning:** ARCO is co-developed with FRET. No version pin is enforced.
The CI checkout step is responsible for obtaining a compatible ARCO snapshot.
The CI job that requires ARCO must check out `../arco/` before installing it.
The exact CI checkout configuration is tracked in `.github/workflows/tests.yml`.
