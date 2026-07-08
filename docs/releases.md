# FRET Release Specification (v1.0 → v1.3)

> **Authoritative product roadmap.** All requirements, scenarios, and milestones trace
> to this document.
>
> **Related:** [roadmap.md](roadmap.md) · [requirements.md](requirements.md) ·
> [scenarios.md](scenarios.md) · [architecture.md](architecture.md)

---

## Product vision

FRET is a **ROS 2 full-stack robotics framework** that connects the **ARCO** motion-
planning library to simulators and (eventually) hardware. Each minor release adds one
robot class, one showcase scenario, and one article-ready visual demo.

| Release | Robot | Scenario | Visual backend |
|---|---|---|---|
| **v1.0** | PPP gantry (3 prismatic) | Pick-and-place box through warehouse obstacles | **MuJoCo** (primary) |
| **v1.1** | Dubins mobile × 2 | Dual-robot race A→B through column forest | MuJoCo + ARCO race renderer |
| **v1.2** | RRP / SCARA (3-DOF) | Reproduce ARCO `rrp` / `rr` scenarios in FRET | Gazebo + MuJoCo |
| **v1.3** | 6-DOF manipulator | Final challenge — full C-space planning + execution | Gazebo + MuJoCo |

**Platform stack (all releases):**

- **Planning:** ARCO (RRT*, SST, KDTree occupancy, trajectory pruner)
- **Middleware:** ROS 2 Jazzy
- **Engineering SITL:** Gazebo Harmonic (where applicable)
- **Showcase visuals:** MuJoCo

---

## Engineering foundation (v0.x — complete)

Before v1.0, the project validated the pipeline on a **3-DOF SCARA bootstrap robot**
(MS-1–5): Jacobian control, C-space planning, occupancy bridge, pillar avoidance in
pure-Python CI. That code remains as shared infrastructure; it is **not** a release
target. v1.2 reuses and extends it for RRP.

---

## v1.0 — PPP gantry warehouse pick-and-place

### Goal

A **PPP gantry robot** (three prismatic joints: X, Y, Z) moves a **cargo box** from a
start pose to a goal pose through a **warehouse of static box obstacles**, rendered in
**MuJoCo** with publication-quality visuals.

### Grasp model (v1.0 simplification)

Full gripper simulation is **out of scope**. The cargo box uses **magnetic grasping**:

1. When the gantry end-effector enters a capture zone, the box **welds** to the EE frame.
2. The welded box is included in the planner collision predicate (EE + box envelope).
3. On goal arrival, the weld **releases** and the box remains at the goal pose.

This avoids manipulator DOF while still demonstrating pick-and-place semantics.

### Environment

Based on ARCO `map/ppp.yml` (industrial high-bay warehouse):

| Axis | Range |
|---|---|
| X (aisle length) | 0 – 60 m |
| Y (bay width) | 0 – 20 m |
| Z (height) | 0 – 6 m |

Static box obstacles block corridors; the planner must find a 3-D C-space path. Scatter
boxes and width-crossing barriers create detours (see ARCO PPP scene for reference layout).

### Robot

| Property | Value |
|---|---|
| Model name | `ppp` |
| DOF | 3 (prismatic X, Y, Z) |
| EE geometry | 0.5 m × 0.5 m square face |
| Planning space | C-space (qx, qy, qz) ≡ world position |
| ARCO reference | `arco/simulator/scenes/ppp.py`, `map/ppp.yml` |

### Scenario ID

**SC-v10** — `config/scenarios/ppp_warehouse.yml`

### Acceptance criteria

| # | Criterion |
|---|---|
| V10-1 | `ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco` runs without error |
| V10-2 | ARCO RRT* finds collision-free path within 30 s (EE + welded box, MuJoCo contacts) |
| V10-3 | Gantry tracks trajectory; EE position error ≤ 10 mm in MuJoCo |
| V10-4 | Cargo box picked at start zone, released at goal zone |
| V10-5 | No collision between welded box and static obstacles along executed path |
| V10-6 | Headless MuJoCo render produces MP4 ≥ 30 s suitable for README / article |
| V10-7 | Unit tests for PPP kinematics, magnetic weld FSM, and C-space checker |

### Implementation tasks

| ID | Task |
|---|---|
| T10-01 | PPP kinematics module (`fret/control/kinematics_ppp.py`) |
| T10-02 | MJCF model: gantry + warehouse boxes (`src/fret/mjcf/ppp_warehouse.xml`) |
| T10-03 | MuJoCo backend adapter (`fret/ros/mujoco_bridge.py`) |
| T10-04 | Magnetic grasp FSM (`fret/control/grasp_magnet.py`) |
| T10-05 | PPP C-space checker (EE box + cargo envelope) |
| T10-06 | Scenario YAML + launch (`sitl.py backend:=mujoco`) |
| T10-07 | Headless video render script + CI artifact |
| T10-08 | Port/adapt ARCO `ppp.yml` obstacle layout |

---

## v1.1 — Dubins dual-robot race

### Goal

Two **Dubins vehicles** race from **A → B** through a world of **columns with varied
heights**, reproducing the ARCO `vehicle` race format (RRT* vs SST or dual-agent race).

### Environment

| Element | Description |
|---|---|
| Floor | Bounded 2-D plane (≈ 50 m × 50 m) |
| Columns | Circular or rectangular posts, **varied height** for visual depth in MuJoCo |
| Start A | Lower-left region |
| Goal B | Upper-right region |
| Walls | Optional maze walls (ARCO `vehicle.yml` style) |

### Robots

| Property | Value |
|---|---|
| Model name | `dubins` |
| DOF | 3 (x, y, θ) — SE(2) |
| Kinematics | ARCO `DubinsVehicle` |
| Control | ARCO Pure Pursuit |
| Planning | ARCO SST (per robot) |
| ARCO reference | `map/vehicle.yml`, `scenes/vehicle.py` |

### Scenario ID

**SC-v11** — `config/scenarios/dubins_race.yml`

### Acceptance criteria

| # | Criterion |
|---|---|
| V11-1 | Two robots plan independently; both reach B without collision with columns |
| V11-2 | Race video shows simultaneous motion (split-screen or shared world) |
| V11-3 | Dubins curvature constraint respected (min turning radius) |
| V11-4 | MP4 artifact uploaded on release tag |

### Implementation tasks

| ID | Task |
|---|---|
| T11-01 | SE(2) kinematics + Dubins validity adapter |
| T11-02 | Column forest world (MJCF + optional Gazebo SDF) |
| T11-03 | Dual-agent launch + race orchestration |
| T11-04 | Integrate ARCO Pure Pursuit tracking loop |
| T11-05 | Race metrics (time-to-goal, path length) |

---

## v1.2 — RRP / SCARA (ARCO reproduction)

### Goal

Reproduce ARCO **`rrp`** and **`rr`** scenarios inside the FRET ROS 2 pipeline — the
same pillar/slab obstacles, joint-space planning, and race-style execution that ARCO
CI already validates.

### Robots

| Model | DOF | ARCO map |
|---|---|---|
| `rrp` | 3 (θ₁, θ₂, z) | `map/rrp.yml` |
| `rr` | 2 (θ₁, θ₂) | `map/rr.yml` |

Leverages existing FRET SCARA bootstrap (`src/fret/urdf/scara.xacro`, MS-1–5 code).

### Scenario IDs

| ID | File | Description |
|---|---|---|
| SC-v12a | `rrp_pillars.yml` | 3-D pillars + slabs (ARCO rrp) |
| SC-v12b | `rr_planar.yml` | 2-D planar arm (ARCO rr) |

### Acceptance criteria

| # | Criterion |
|---|---|
| V12-1 | RRP scenario matches ARCO `rrp.yml` obstacle layout and pass/fail semantics |
| V12-2 | EE tracking error ≤ 5 mm (RRP) on Gazebo and MuJoCo |
| V12-3 | RR planar scenario passes in pure-Python CI |
| V12-4 | Side-by-side video: ARCO arcosim vs FRET sitl (same scenario) |

---

## v1.3 — 6-DOF manipulator (final challenge)

### Goal

A **6-DOF revolute manipulator** (e.g. UR5 or custom arm) performs C-space planning
and trajectory execution in a cluttered environment — the capstone release.

### Scope (high level)

| Item | Detail |
|---|---|
| Model | 6 revolute joints; URDF + MJCF |
| IK | Numerical IK (Jacobian pseudoinverse or analytic where available) |
| Planning | ARCO SST in 6-D C-space |
| Collision | Per-link FK + KDTree clearance |
| Environment | Configurable obstacle field (tabletop or cell) |

### Scenario ID

**SC-v13** — `config/scenarios/six_dof_challenge.yml` *(to be created)*

### Acceptance criteria

| # | Criterion |
|---|---|
| V13-1 | Collision-free 6-D path planned within 60 s |
| V13-2 | EE reaches goal with ≤ 5 mm error |
| V13-3 | Self-collision checking enabled |
| V13-4 | Demo video + benchmark table (planning time, path length) |

*Detailed task breakdown will be written when v1.2 is complete.*

---

## Version tagging

| Tag | Content | Prerequisite |
|---|---|---|
| `v0.9.0` | Bootstrap SCARA pipeline (MS-1–5) | ✅ Done |
| `v1.0.0` | PPP warehouse + magnetic grasp + MuJoCo video | T10-* |
| `v1.1.0` | Dubins dual race | T11-* |
| `v1.2.0` | RRP + RR ARCO reproduction | T12-* |
| `v1.3.0` | 6-DOF challenge | T13-* |

---

## Deprecated / removed

The following documents and goals are **superseded** by this release spec:

- MS-6 / MS-7 milestone framing (MuJoCo + TBD showcase)
- SCARA-as-v1.0-showcase
- SC-01 – SC-05 as primary release scenarios (retained as regression tests only)
- Platform study 2026 Q3 dual-demo proposal

Regression scenarios (SC-01 – SC-05) remain in CI for the bootstrap SCARA pipeline
until v1.2 replaces them as the canonical arm tests.
