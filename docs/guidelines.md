# FRET-specific coding notes

The shared baseline for Python/C++ naming, docstrings/Doxygen, formatting,
type annotations, and error handling lives in
[.guidelines/languages/py.md](../.guidelines/languages/py.md),
[.guidelines/languages/cpp.md](../.guidelines/languages/cpp.md), and
[.guidelines/style/naming.md](../.guidelines/style/naming.md). This file
covers only what is specific to FRET: ROS 2 layout, architecture
invariants, and build/test conventions. Workflow, SDD, V-cycle, and merge
policy live in [CONTRIBUTING.md](../CONTRIBUTING.md).

## 1. File and Package Structure

### Python

Python modules under `src/fret/` that are not ROS nodes are organized by
functional domain.

### ROS 2 Package Structure

FRET follows standard ROS 2 ament_cmake package layout:

```
src/fret/
├── CMakeLists.txt           ← build configuration
├── package.xml              ← ROS 2 package manifest
├── include/fret/            ← public C++ headers
├── src/                     ← C++ source files and private headers
├── urdf/                    ← robot description files (XACRO/URDF)
├── mesh/                    ← mesh generator scripts
├── mjcf/                    ← MuJoCo scene files (v1.0+)
├── launch/                  ← sitl.py, mujoco.py, sim.py, hardware.py
└── config/                  ← parameter files (YAML)
```

### Launch Files

- Launch files live in `src/fret/launch/` and use the `.py` extension
  (not XML) for programmatic flexibility.
- Descriptive names: `mujoco.py` (MuJoCo bridge), `sitl.py` (SITL
  orchestration), `hardware.py` (hardware nodes).
- Visual inspection uses MuJoCo only: `scripts/view.sh` and
  `scripts/video.sh` (all showcase flags required, see
  [mujoco.md § Required flags](mujoco.md#required-flags)).

### URDF/XACRO Files

- URDF descriptions live in `src/fret/urdf/`; use XACRO for parametric
  descriptions, URDF files are build-generated.
- Each robot model is a single `.xacro` file named after the model (e.g.
  `open_manipulator_x.xacro`).
- Optional mesh generators in `src/fret/mesh/<model>.py` produce STL
  files at build time.

## 2. ROS 2 Resources

- **Topics**: `/` hierarchy with `snake_case` (e.g. `/joint_states`,
  `/cmd_vel`).
- **Services**: `snake_case` (e.g. `/get_jacobian`).
- **Actions**: `PascalCase` (e.g. `ExecuteTrajectory`).
- **Parameters**: `snake_case` with `.` for nesting (e.g.
  `controller.max_velocity`).
- **Nodes**: `snake_case` (e.g. `controller_node`, `planner_node`).

Physical-variable naming (`who_what`) follows
[.guidelines/style/naming.md](../.guidelines/style/naming.md); non-SI
config parameters expressed for human readability (e.g.
`max_turn_rate_deg_s` in a YAML file) are exempt, and the exemption must
be documented with a comment in the config file.

## 3. Code Formatting

Formatting is enforced on **production code only** (`src/` and
`scripts/`). Test files (`tests/`) are excluded, they are not production
code and do not need to be perfectly formatted or documented.

FRET pins Black/isort to `line-length = 79` (an accepted per-project
override, see [.guidelines/languages/py.md](../.guidelines/languages/py.md)):

```bash
python -m black --target-version py312 --line-length 79 src/
python -m isort --line-length 79 src/
find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
```

CI enforces these rules on `src/` only.

## 4. Testing and the V-Cycle

Development workflow (SDD, V-cycle, TDD, and merge policy) is defined in
[CONTRIBUTING.md](../CONTRIBUTING.md). This section covers **test
conventions** only.

### Python test conventions

- Framework: `pytest` (not `unittest`).
- Location: `tests/` mirroring `src/fret/` (e.g.
  `tests/control/test_kinematics.py`).
- Pure-Python simulation: `tests/simulation/` (no ROS runtime).
- Integration tests: `tests/integration/` using `launch_testing`.
- Mocks: `unittest.mock` or `pytest-mock` to avoid ROS 2 runtime
  dependencies in unit tests; integration tests may launch real nodes.

```bash
pytest tests/ --ignore=tests/integration -v
pytest tests/ --ignore=tests/integration --cov=src/fret --cov-fail-under=90
```

### C++ test conventions

- Framework: GTest for unit tests; `launch_testing` for integration
  tests.
- Location: `src/fret/test/` for C++ unit tests.
- `GTEST_SKIP()` with a descriptive message for not-yet-implemented
  tests.

## 5. Configuration Parameters

Tunable parameters belong in YAML files under `config/`, hierarchical
structure, comments explaining parameter meanings and units,
`snake_case` with `.` for nesting in ROS 2 node access:

```yaml
controller:
  # Maximum joint velocity in rad/s
  max_joint_velocity: 1.57
  # Jacobian pseudoinverse damping factor (dimensionless)
  damping_factor: 0.01
```

Declare all ROS 2 parameters in node constructors with default values and
validation; use parameter callbacks for runtime updates when appropriate.

## 6. Architecture Invariants

- **Nodes are executables**: each ROS 2 node is a standalone executable
  with a clear responsibility.
- **Library layers use namespaces**: reusable C++ code belongs in
  namespaced libraries (`fret::control`, `fret::planning`).
- **Separation of concerns**: keep ROS 2 communication (topics/services)
  separate from core algorithms; core algorithms should have non-ROS
  interfaces for unit testing.
- **Kinematics layer**: forward/inverse kinematics are pure functions
  independent of ROS.
- **Jacobian computation**: numerical or analytical methods in
  `fret::control`.
- **Planning**: trajectory generation produces waypoints in joint or task
  space; planners respect joint limits and velocity constraints.

## 7. ROS 2 Best Practices

- **Composition**: component-based nodes (`rclcpp::Node` or
  `rclcpp_components`); support both standalone executables and
  component composition.
- **QoS**: `rclcpp::SensorDataQoS()` for real-time control (lossy, low
  latency), `rclcpp::ServicesQoS()` for state updates (reliable),
  `rclcpp::ParametersQoS()` for configuration (reliable, transient
  local).
- **Logging**: `RCLCPP_DEBUG/INFO/WARN/ERROR/FATAL`, include node name
  and operation context, avoid excessive logging in high-frequency
  loops.
- **Timing**: `rclcpp::Rate` for periodic loops, `rclcpp::Time` for
  timestamps (not `std::chrono` directly); consider real-time executors
  for real-time nodes.

## 8. Build and Dependency Management

- Follow ament_cmake conventions; declare all dependencies in
  `package.xml` and `CMakeLists.txt`; use `find_package()`.
- Install all necessary files (launch, config, URDF, meshes) to
  `share/fret/`.
- Minimize dependencies; prefer well-maintained ROS 2 packages; document
  non-standard dependencies in README; use `rosdep` for resolution.

## 9. Pre-flight Checklist

Before finishing any implementation task, run the consolidated gate
script or equivalent steps:

```bash
bash scripts/check/pre_push.sh
```

With `--skip-ros`, smoke and integration gates are omitted (useful
without a local ROS 2 install). Individual steps:

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
bash scripts/tests/unit.sh              # requires built workspace + ROS
bash scripts/tests/smoke.sh             # requires ROS + xvfb
bash scripts/tests/integration.sh       # requires ROS + xvfb
```

CI workflows (all required on pull requests): `formatting.yml` (Black,
isort, clang-format), `type_check.yml` (mypy strict on `src/`),
`tests.yml` (parallel unit shards, coverage gate, smoke, integration).
