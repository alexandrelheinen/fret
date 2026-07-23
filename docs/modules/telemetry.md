# Telemetry Module (specification)

**Package:** `fret.telemetry`  
**Source:** `src/fret/telemetry/`  
**Layouts:** `src/fret/telemetry/layouts/` (PlotJuggler `.xml`, not under `docs/`)  
**Tests:** `tests/telemetry/`  
**Debug / plot tool:** [PlotJuggler](https://github.com/facontidavide/PlotJuggler) (CSV loader)  
**Status:** Implemented (CSV writer + scenario hooks + release R2 layout + layouts).

> **V-cycle:** Level 1–2 design artifact. Implementation must not start until
> acceptance criteria below are agreed. Related requirement: proposed
> **FR-SIM-12** (time-series telemetry export for sim/debug).

---

## 1. Intent

FRET needs an opt-in, ROS-independent telemetry logger that records
simulation / control state **over time** so engineers can diagnose motion
quality (fluidity, kink tracking, lag, accel spikes) in PlotJuggler.

This is **not** a replacement for:

- ROS `get_logger()` console messages
- MuJoCo contact JSONL / `metrics.json` ([mujoco_physics_v1.2.md](../mujoco_physics_v1.2.md) §4)
- In-memory `pose_history` used only by tests / render scripts

It **is** a durable, PlotJuggler-first time-series export under `/tmp/fret_telemetry/`.

---

## 2. Scope

### In scope (MVP → v1)

| Item | Notes |
|---|---|
| Pure-Python logger API usable from scenario runners and MuJoCo bridge core | No ROS runtime required |
| Opt-in enable flag + output directory | Default **off** |
| Wide CSV export compatible with PlotJuggler | One row per sample tick |
| Strict series naming grammar | `agent.quantity_frame.component` |
| Sidecar manifest (units, frames, agents, schema version) | JSON next to the CSV |
| First producer: Dubins race session (kinematic + physics) | SE(2) + body rates + commands |
| Documented PlotJuggler load procedure | Time axis = `t` |

### Out of scope (v1)

| Item | Reason |
|---|---|
| rosbag / MCAP as primary format | Couples debug path to ROS; optional later export |
| MuJoCo-plugin or native MJB dump as primary logger | Misses kinematic + MPC quantities |
| Real-time PlotJuggler streaming over UDP/WebSocket | Nice-to-have after file path is solid |
| Automatic upload to CI artifacts for every PR | Opt-in / evidence jobs only |
| Replacing contact JSONL | Keep FR-SIM-08 path unchanged |

---

## 3. Architectural placement

```
Scenario / bridge tick
        │
        ▼
 fret.telemetry.TelemetrySession   ← pure Python (Level 3)
        │  append sample dict keyed by series id
        ▼
 /tmp/fret_telemetry/<run_id>/telemetry.csv
 /tmp/fret_telemetry/<run_id>/manifest.json
        │
        ▼
 PlotJuggler  →  DataLoad CSV  →  select time column `t`
```

| Layer | Owns |
|---|---|
| `fret.telemetry` | Naming validation, buffering, CSV/manifest writers |
| `fret.scenario.*` / bridge core | Decides *what* to sample each tick; calls `record()` |
| `fret.ros.*` | Optional: enable via ROS param; **must not** be required to log |
| ARCO | Unchanged — FRET reads public tracker/vehicle state only |
| PlotJuggler | Offline consumer of CSV |

Rationale: matches FRET ownership (algorithms ROS-agnostic; ROS is thin I/O).
Same pattern as `fret.ros.mujoco_physics_log` but for continuous state, not contacts.

---

## 4. Naming convention (normative)

### 4.1 Grammar

Every numeric series id **shall** match:

```text
{agent}.{quantity}_{frame}.{component}
```

| Segment | Alphabet | Rules |
|---|---|---|
| `agent` | `[a-z][a-z0-9_]*` | Stable run identity; `_` allowed (`tb3_sst`, `tb3_rrt`, `tb3_dummy`, `omy`, `joint_arm_0` as agent when logging a single joint stream). |
| `quantity` | `[a-z][a-z0-9_]*` | Physical meaning **without** frame (`position`, `velocity`, `acceleration`, `orientation`, `omega`, `cmd_velocity`, …). |
| `frame` | see §4.2 | **Required** on every quantity that is spatially represented. Appended to quantity with a single `_`. |
| `component` | see §4.3 | Vector / quaternion / scalar channel. |

**Separators**

- `.` separates the three top-level segments (`agent` / `quantity_frame` / `component`).
- `_` is reserved **inside** a segment (agent tokens, multi-word quantities, frame tags).
- `/` is **forbidden** in series ids (reserved for possible future ROS topic export).
- `,` `"` spaces and uppercase letters are **forbidden** in series ids (PlotJuggler CSV + consistency).

**Why dots (not only underscores):** PlotJuggler shows each CSV header as a
series name; dotted ids stay human-readable and map cleanly to a 3-level mental
model. PlotJuggler’s CSV loader treats headers as flat names (no mandatory tree
split); dots remain the canonical FRET form. Do **not** flatten to
`agent_quantity_frame_component` — that loses structure and collides with
`_` inside agent names.

### 4.2 Frames of reference (normative catalog)

`frame` **shall** be one of:

| Token | Meaning |
|---|---|
| `enu` | East-North-Up world / map frame (FRET default warehouse / MuJoCo world XY≈EN, Z up) |
| `ned` | North-East-Down (only when a producer explicitly uses NED) |
| `body` | Vehicle / link body frame (forward–left–up unless a model doc says otherwise) |
| `base` | Robot `base_link` frame |
| `map` | Occupancy / planning map frame when distinct from `enu` |
| `joint` | Joint / configuration space (rad, m); not a Cartesian frame |
| `sensor_<name>` | Quantity expressed in a named sensor frame (`sensor_lidar`, …) |
| `ctrl` | Controller command space (setpoints, not a spatial frame) |

Producers **shall not** invent ad-hoc frame tokens without updating this table.

Every Cartesian / attitude quantity **must** include a frame token.
Joint-space scalars use `_joint` (e.g. `omy.position_joint.arm_1` — see §4.4).

### 4.3 Components (axis / channels)

| Quantity kind | Allowed `component` values |
|---|---|
| 3-vector | `x`, `y`, `z` |
| 2-vector (SE2 planar) | `x`, `y` |
| Quaternion (Hamilton, `wxyz`) | `qw`, `qx`, `qy`, `qz` |
| Angle / yaw (scalar) | `yaw` (radians) or `rad` |
| Scalar with no axis | `val` |
| Matrix / covariance (v2+) | `m00`, `m01`, … (row-major) |

Yaw-only Dubins pose **shall** be logged as:

- `tb3_sst.position_enu.x`, `.y` (and `.z` if available; else omit `z`)
- `tb3_sst.orientation_enu.yaw`

not as a partial quaternion unless a full attitude is actually available.

### 4.4 Examples

| Series id | Meaning |
|---|---|
| `tb3_sst.position_enu.x` | SST TurtleBot easting [m] |
| `tb3_sst.velocity_body.x` | Forward body speed [m/s] |
| `tb3_sst.acceleration_body.x` | Forward body accel [m/s²] |
| `tb3_sst.omega_body.z` | Yaw rate in body frame [rad/s] |
| `tb3_sst.cmd_velocity_ctrl.val` | Commanded scalar cruise/track speed [m/s] |
| `tb3_sst.cmd_omega_ctrl.val` | Commanded yaw rate [rad/s] |
| `tb3_sst.cross_track_map.val` | Signed lateral error vs reference path [m] |
| `tb3_sst.progress_map.val` | Contouring arc-length `s` [m] |
| `tb3_rrt.orientation_enu.yaw` | RRT* agent heading [rad] |
| `omy.position_joint.arm_1` | OMX joint position (agent=`omy`, frame=`joint`, component=`arm_1`) |
| `omy.velocity_enu.x` | End-effector velocity in ENU [m/s] |

**Joint special case:** when the “axis” is a joint name, that name occupies the
`component` segment (`arm_1`, `gripper`). The frame remains `joint`.

### 4.5 Time column (special)

| Column | Type | Rule |
|---|---|---|
| `t` | float64 | **Mandatory** first column. Monotonic simulation time [s]. Not an `agent.*` series. |

Optional extra timebases (not for PlotJuggler primary axis):

| Column | Meaning |
|---|---|
| `t_wall` | Wall-clock seconds since session start |
| `tick` | Integer sample index |

---

## 5. Output artifacts

### 5.1 Directory layout

```text
/tmp/fret_telemetry/<run_id>/
  <basename>.csv     # PlotJuggler primary input
  <basename>.json    # schema, units, agents, frames used
```

`<run_id>` default: `{scenario_id}_{YYYYMMDD_HHMMSS}_{pid}`  
Configurable override via config / env `FRET_TELEMETRY_DIR`.

Release / R2 layout (same basename as the overview video):

```text
releases/<tag>/<scenario>/
  <scenario>_overview.mp4
  <scenario>_overview.csv
  <scenario>_overview.json
  <scenario>_follow.mp4          # when present
latest/<scenario>/...
```

### 5.2 CSV format (PlotJuggler contract)

Normative rules so `DataLoad CSV` succeeds:

1. **Delimiter:** `,` (comma). No other delimiter.
2. **Header row 1:** unique column names; first column **must** be `t`.
3. **No commas** inside header names; no quoting tricks.
4. **Numeric cells only** in data rows (IEEE floats). Use empty cell for
   “not sampled this tick” — do **not** write `nan` strings if they break a
   consumer; prefer omitting the series from that run’s schema, or write a
   constant empty and document in manifest.
5. **One row per sample** of the session clock (shared `t` for all agents).
6. **Column order:** `t`, then optionally `t_wall`, `tick`, then series ids
   sorted lexicographically (stable across runs with the same schema).
7. **No duplicate headers.**
8. **UTF-8**, Unix `\n` line endings.
9. **No `#` comment preamble** in v1 (PlotJuggler can skip rows, but a clean
   header on line 1 avoids operator error). Put metadata in `manifest.json`.

Example (`telemetry.csv`):

```csv
t,tb3_rrt.orientation_enu.yaw,tb3_rrt.position_enu.x,tb3_rrt.position_enu.y,tb3_sst.orientation_enu.yaw,tb3_sst.position_enu.x,tb3_sst.position_enu.y
0.00,0.56,1.39,1.01,0.49,1.01,1.39
0.05,0.58,1.41,1.02,0.51,1.03,1.41
```

### 5.3 PlotJuggler load procedure

1. Open PlotJuggler → **Data Loader** → **CSV**.
2. Select `/tmp/fret_telemetry/<run_id>/telemetry.csv`.
3. Choose time axis column **`t`** (not index).
4. Confirm delimiter `,`.
5. Drag series (e.g. `tb3_sst.velocity_body.x`) onto plots — **or** load a
   checked-in layout (below).

Compatibility target: PlotJuggler ≥ 3.x CSV plugin (`DataLoadCSV`).

### 5.3.1 Checked-in layouts (location)

PlotJuggler layout XML lives next to the telemetry producer:

```text
src/fret/telemetry/layouts/
  index.yaml          # scenario_id → layout basename
  dubins_race.xml
  omx_arm.xml         # shared by OMX pick-place / maze / clutter / reach
  omy_arm.xml         # shared by OMY pick-place / clutter / reach
  README.md           # why not docs/; regenerate notes
```

**Not under `docs/`:** docs are prose and static images; layouts are
operational UI assets loaded by PlotJuggler / `scripts/plotjuggler.sh`.
**Not under `config/`:** they mirror the FR-SIM-12 series schema (owned by
`fret.telemetry`), not scenario YAML parameters.

```bash
bash scripts/plotjuggler.sh \
  --csv /tmp/fret_telemetry/<run_id>/telemetry.csv \
  --scenario dubins_race

# or
plotjuggler -d /path/to/telemetry.csv \
  -l src/fret/telemetry/layouts/dubins_race.xml
```

Resolve in Python: `fret.telemetry.layout_path_for_scenario("dubins_race")`.
Regenerate hand-authored baselines with
`python3 scripts/gen_plotjuggler_layouts.py`, or overwrite after
**File → Save Layout** in PlotJuggler against a real CSV.

### 5.4 `manifest.json`

```json
{
  "schema_version": 1,
  "format": "fret.telemetry.csv",
  "run_id": "dubins_race_20260723_170000_12345",
  "scenario_id": "dubins_race",
  "created_utc": "2026-07-23T17:00:00Z",
  "dt_nominal_s": 0.05,
  "time_column": "t",
  "time_unit": "s",
  "agents": ["tb3_rrt", "tb3_sst", "tb3_dummy"],
  "frames_used": ["enu", "body", "ctrl", "map"],
  "series": {
    "tb3_sst.position_enu.x": {"unit": "m", "quantity": "position", "frame": "enu", "component": "x"},
    "tb3_sst.velocity_body.x": {"unit": "m/s", "quantity": "velocity", "frame": "body", "component": "x"}
  },
  "plotjuggler": {
    "recommended_time_axis": "t",
    "delimiter": ","
  }
}
```

The CSV remains usable **without** the manifest; the manifest is authoritative
for units and for validating naming.

---

## 6. Public API (Level 3 sketch)

```python
# Planned: src/fret/telemetry/session.py

class TelemetrySession:
    def __init__(self, run_id: str, output_dir: Path, *, enabled: bool) -> None: ...
    def register_series(self, series_id: str, *, unit: str) -> None: ...
    def record(self, t: float, values: Mapping[str, float]) -> None: ...
    def close(self) -> Path:
        """Flush CSV + manifest; return path to telemetry.csv."""

def parse_series_id(series_id: str) -> SeriesId: ...
def validate_series_id(series_id: str) -> None:
    """Raise ValueError if grammar / catalog rules fail."""
```

Behavioral rules:

- When `enabled=False`, all calls are no-ops (zero file I/O).
- `register_series` **shall** validate ids before the first `record`.
- `record` **shall** reject unknown keys (fail fast in tests; in production
  runners, configurable `strict=True` default for CI).
- `close()` is idempotent.

---

## 7. Configuration

Proposed keys (scenario YAML or `mujoco.yml` / controller-adjacent debug block):

```yaml
telemetry:
  enabled: false
  output_dir: ""   # empty → /tmp/fret_telemetry/<run_id>
  dt_policy: session_tick   # one CSV row per control/sim tick
  agents:
    rrt_star: tb3_rrt
    sst: tb3_sst
    dummy: tb3_dummy
```

Environment override: `FRET_TELEMETRY_ENABLED=1`.

---

## 8. MVP series set (Dubins race)

When telemetry is enabled on `DubinsRaceRunner` / race session, producers
**shall** sample at least:

| Agent | Series |
|---|---|
| `tb3_rrt`, `tb3_sst`, `tb3_dummy` | `position_enu.{x,y}`, `orientation_enu.yaw` |
| planner agents | `velocity_body.x`, `omega_body.z` |
| planner agents | `cmd_velocity_ctrl.val`, `cmd_omega_ctrl.val` |
| planner agents (if available from tracker) | `cross_track_map.val`, `progress_map.val` |

Acceleration: if not directly available, v1 may omit it or derive
finite-difference `acceleration_body.x` and tag `derived: true` in the
manifest.

---

## 9. Development plan (V-cycle)

| Stage | Deliverable | Validation |
|---|---|---|
| **L1 Spec** | This document + FR-SIM-12 in [requirements.md](../requirements.md) | Review / AC checklist |
| **L2 Interfaces** | Series grammar + CSV/manifest contract frozen | Fixture CSV opens in PlotJuggler |
| **L3 Module** | `fret.telemetry` stubs + unit tests for `validate_series_id`, CSV writer | `pytest tests/telemetry/` |
| **L4 Integration** | Wire Dubins race session behind `telemetry.enabled` | Scenario test writes CSV; columns match grammar |
| **Evidence** | One PlotJuggler screenshot or exported layout note in PR when diagnosing motion | Manual / agent computer-use |

### Implementation task breakdown (after AC sign-off)

1. Add `FR-SIM-12` text to `docs/requirements.md` (pointer to this spec).
2. Create `src/fret/telemetry/` with `naming.py`, `session.py`, `csv_writer.py`.
3. Unit tests: grammar accept/reject tables; CSV round-trip; PlotJuggler constraints (unique headers, `t` first).
4. Hook `DubinsRaceSession.step` / `step_physics` behind config flag.
5. Document operator steps in [tutorial.md](../tutorial.md) / [simulation.md](../simulation.md) (short subsection).
6. Optional: helper script `scripts/telemetry_check.py` that validates a CSV against the grammar.

---

## 10. Acceptance criteria

1. [ ] Spec defines a single normative grammar `agent.quantity_frame.component` with frame catalog and component tables.
2. [ ] Spec mandates PlotJuggler-compatible CSV (`t` first, comma delimiter, unique numeric columns) under `/tmp/fret_telemetry/<run_id>/`.
3. [ ] Spec places the module in pure-Python `fret.telemetry` (not ROS-bag-primary, not MuJoCo-plugin-primary).
4. [ ] Spec lists MVP Dubins series and agent ids (`tb3_rrt` / `tb3_sst` / `tb3_dummy`).
5. [ ] Spec includes V-cycle task breakdown and out-of-scope list.
6. [ ] When implemented: enabling telemetry on a Dubins race write `telemetry.csv` + `manifest.json`; a human can open the CSV in PlotJuggler with time axis `t` and plot `tb3_sst.position_enu.x` vs `t`.
7. [ ] When implemented: unit tests reject illegal series ids (uppercase, missing frame, `/` separators, duplicate headers).

---

## 11. Traceability

| ID | Statement |
|---|---|
| **FR-SIM-12** *(proposed)* | FRET shall provide an opt-in time-series telemetry export of simulation/control state to a PlotJuggler-compatible CSV under `/tmp/fret_telemetry/`, using the naming grammar in this document. |
| Related | FR-SIM-02 (algorithm layers simulator-agnostic), FR-SIM-08 (contact log remains separate) |

---

## 12. Open questions (resolve before L3 coding)

1. Default world frame token: confirm `enu` for all current MuJoCo worlds (vs introducing `world` as an alias).
2. Whether derived finite-difference acceleration is allowed in MVP or deferred.
3. Whether manipulator joint components use MJCF joint names verbatim (`joint_arm_0`) or shortened aliases.
4. Max CSV column count / down-sample policy for long physics runs (e.g. >15 min).

---

## References

- Existing contact/metrics pattern: [mujoco_physics_v1.2.md](../mujoco_physics_v1.2.md) §4, `src/fret/ros/mujoco_physics_log.py`
- PlotJuggler CSV loader: [PlotJuggler DataLoadCSV](https://github.com/facontidavide/PlotJuggler)
- Ownership boundary: [arco.md](../arco.md) § Ownership boundary
- Coding standards: [guidelines.md](../guidelines.md)
