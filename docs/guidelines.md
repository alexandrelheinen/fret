# FRET Coding Guidelines

This document is the single authoritative reference for coding conventions in the FRET project. All contributors and AI agents should follow these rules rigorously — they are **not optional**.

FRET is a ROS 2 workspace with Python and C++ components. These guidelines integrate ROS 2 best practices with rigorous software engineering standards.

## 1. File and Package Structure

### Python Files

- **One main class per `.py` file.** Small, tightly-coupled auxiliary classes may coexist in the same file.
- **Use folders with `__init__.py` instead of `_`-separated suffixes.** If several files share a common suffix, turn that suffix into a sub-package and drop it from the filenames.
- **`__init__.py` files are re-export-only.** They must not contain class or function definitions; they only import and expose the public API of the package.
- Python modules under `src/fret/` that are not ROS nodes should be organized by functional domain.

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

- Launch files live in `src/fret/launch/` and use the `.py` extension.
- Launch files must be Python-based (not XML) for programmatic flexibility.
- Use descriptive names: `sim.py` (Gazebo simulation backend), `mujoco.py` (MuJoCo bridge), `hardware.py` (hardware nodes).
- Visual inspection uses MuJoCo only: `scripts/view.sh` (interactive) and `scripts/video.sh` (MP4).
- Launch files should support command-line arguments for model selection and configuration.

### C++ Files

- **Namespace mirrors directory**: A file in `src/fret/src/control/` belongs to namespace `fret::control`.
- **Public headers** live in `src/fret/include/fret/<namespace>/`.
- **Private headers** are co-located with their `.cpp` file in `src/fret/src/<namespace>/`.
- **Nodes** (ROS 2 executables) live in `src/fret/src/` directly and belong to the base `fret` namespace.

### URDF/XACRO Files

- URDF descriptions live in `src/fret/urdf/`.
- Use XACRO for parametric descriptions; URDF files should be build-generated.
- Each robot model is a single `.xacro` file named after the model (e.g., `scara.xacro`).
- Optional mesh generators in `src/fret/mesh/<model>.py` produce STL files at build time.

## 2. Naming Conventions

### Python

- **Modules and packages**: `snake_case`
- **Classes**: `PascalCase`
- **Functions and methods**: `snake_case`
- **Constants**: `UPPER_CASE`
- **Private members**: `_leading_underscore`

### C++

Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

- **Classes and types**: `PascalCase`
- **Functions and methods**: `snake_case`
- **Variables**: `snake_case`
- **Private/protected members**: `trailing_underscore_`
- **Constants and macros**: `UPPER_CASE`
- **Namespaces**: `snake_case`
- **Files**: `snake_case.cpp`, `snake_case.hpp`

### ROS 2 Resources

- **Topics**: Use `/` hierarchy with `snake_case` (e.g., `/joint_states`, `/cmd_vel`)
- **Services**: `snake_case` (e.g., `/get_jacobian`)
- **Actions**: `PascalCase` (e.g., `ExecuteTrajectory`)
- **Parameters**: `snake_case` with `.` for nesting (e.g., `controller.max_velocity`)
- **Nodes**: `snake_case` (e.g., `controller_node`, `planner_node`)

### Physical Variable Naming

All physical variables must follow the `who_what` (or `who_what_where` when frame of reference matters) convention. The rules below are **mandatory** for all layers.

**Physical quantity, not unit.** Never suffix a variable with its SI unit. Use the quantity name instead.

| Wrong | Correct |
|---|---|
| `size_m` | `physical_size` or `cell_size` |
| `radius_m` | `lookahead_distance` |
| `speed_ms` | `max_speed` |
| `angle_rad` | `joint_angle` or `shoulder_angle` |

Non-SI config parameters are explicitly exempt: a configuration value expressed in deg/s for human readability may keep its unit suffix (e.g., `max_turn_rate_deg_s` in a YAML file). The exemption must be documented with a comment in the config file.

**Qualifiers are prefixes.** `max`, `min`, `avg`, and similar qualifiers always come first.

| Wrong | Correct |
|---|---|
| `speed_max` | `max_speed` |
| `value_avg` | `avg_value` |
| `torque_desired` | `desired_torque` |

**Integers are suffixed with `_count`.** Never use a `num_` prefix or leave an integer qualifier implicit.

| Wrong | Correct |
|---|---|
| `num_joints` | `joint_count` |
| `waypoints_per_edge` | `waypoints_per_edge_count` |
| `num_iterations` | `iteration_count` |

**Lists use the plural form**, unless the list represents a coordinate sequence.

| Wrong | Correct |
|---|---|
| `joint_position` (list) | `joint_positions` |
| `link_length` (list) | `link_lengths` |

**Two-word quantity names are valid** and should not be collapsed. `turn_rate`, `cell_size`, and `joint_velocity` are all acceptable base names.

## 3. Documentation

### Python — Google Style

All public classes and methods must have Google-style docstrings with the appropriate sections.

```python
def compute_jacobian(self, joint_positions: np.ndarray) -> np.ndarray:
    """Compute the Jacobian matrix for given joint positions.

    Args:
        joint_positions: Array of joint angles in radians, shape (n,).

    Returns:
        Jacobian matrix of shape (6, n) mapping joint velocities to
        end-effector twist.

    Raises:
        ValueError: If joint_positions has incorrect shape.
    """
```

Required sections:
- `Args` — for every parameter (skip only when there are none).
- `Returns` — for every non-`None` return value.
- `Raises` — whenever the method raises an exception intentionally.
- `Yields` — for generator methods.

References: [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html), [PEP 8](https://peps.python.org/pep-0008/).

### C++ — Doxygen

Public headers must be documented with Doxygen using `/** */` block comments.

```cpp
/**
 * @brief Compute forward kinematics for the given joint configuration.
 * 
 * @param joint_positions Vector of joint angles in radians.
 * @return Homogeneous transformation matrix (4x4) of end-effector pose.
 * @throws std::invalid_argument if joint_positions size mismatches DOF.
 */
Eigen::Matrix4d compute_forward_kinematics(
    const std::vector<double>& joint_positions) const;
```

Required tags:
- `@brief` — concise one-line description
- `@param` — for each parameter
- `@return` — for non-void returns
- `@throws` — for intentional exceptions

## 4. Code Formatting

### Python

Formatting is enforced on **production code only** (`src/` and `scripts/`).
Test files (`tests/`) are excluded — they are not production code and do not
need to be perfectly formatted or documented.

Run **both** formatters before every commit:

```bash
python -m black --target-version py312 --line-length 79 src/
python -m isort --line-length 79 src/
```

- `black` target version: `py312`, line length: `79`.
- `isort` default profile (no extra configuration needed).
- CI enforces these rules on `src/` only.

### C++

All C++ files are formatted with `clang-format` using the `.clang-format` configuration at the repository root.

```bash
find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
```

The configuration follows ROS 2 conventions (Google C++ style with adaptations).

## 5. Type Annotations

### Python

Variable and parameter typing is **strongly enforced**. Every public method signature must include full type annotations. Use `from __future__ import annotations` at the top of each file.

```python
from __future__ import annotations

from typing import Optional, List
import numpy as np

def plan_trajectory(
    self,
    start: np.ndarray,
    goal: np.ndarray,
    constraints: Optional[dict] = None
) -> List[np.ndarray]:
    """Plan a trajectory from start to goal."""
    ...
```

### C++

Use modern C++ type practices:

- Prefer `auto` for type deduction when the type is obvious from context.
- Use `const` references for read-only parameters.
- Use smart pointers (`std::shared_ptr`, `std::unique_ptr`) for ownership semantics.
- Avoid raw pointers except for non-owning references.

## 6. Testing and the V-Cycle

Development workflow (SDD, 4-level V-cycle stages, constraint layers for AI agents,
Definition of Ready/Done, and merge policy) is defined in
[CONTRIBUTING.md](../CONTRIBUTING.md). **Do not duplicate** that material here.

This section covers **test conventions** only.

### Python test conventions

- Framework: `pytest` (not `unittest`).
- Location: `tests/` mirroring `src/fret/` (e.g., `tests/control/test_kinematics.py`).
- Pure-Python simulation: `tests/simulation/` (no ROS runtime).
- Integration tests: `tests/integration/` using `launch_testing`.
- Mocks: use `unittest.mock` or `pytest-mock` to avoid ROS 2 runtime dependencies in
  unit tests. Integration tests may launch real nodes.
- Run unit tests:
  ```bash
  pytest tests/ --ignore=tests/integration -v
  ```
- Run with coverage:
  ```bash
  pytest tests/ --ignore=tests/integration --cov=src/fret --cov-fail-under=90
  ```

### C++ test conventions

- Framework: GTest for unit tests; `launch_testing` for integration tests.
- Location: `src/fret/test/` for C++ unit tests.
- Use `GTEST_SKIP()` with a descriptive message for not-yet-implemented tests.

### Stub marking convention (Level 3 → Level 4)

```python
# Level 3 — not yet implemented:
@pytest.mark.xfail(strict=True, raises=NotImplementedError)
def test_compute_jacobian_known_configuration():
    km = Kinematics(model="scara")
    result = km.compute_jacobian(np.array([0.0, 0.0, 0.1]))
    assert result.shape == (6, 3)

# Level 4 — remove xfail when implemented:
def test_compute_jacobian_known_configuration():
    km = Kinematics(model="scara")
    result = km.compute_jacobian(np.array([0.0, 0.0, 0.1]))
    assert result.shape == (6, 3)
    assert np.allclose(result[2, :], [0.0, 0.0, 1.0])
```

## 7. Configuration Parameters

### YAML Configuration Files

Tunable parameters belong in YAML files under `config/`.

- Use hierarchical structure with meaningful nesting.
- Include comments explaining parameter meanings and units.
- Parameter names: `snake_case` with `.` for nesting in ROS 2 node access.

Example:

```yaml
controller:
  # Maximum joint velocity in rad/s
  max_joint_velocity: 1.57
  
  # Jacobian pseudoinverse damping factor (dimensionless)
  damping_factor: 0.01
  
  # Control loop frequency in Hz
  update_rate: 100.0
```

### ROS 2 Parameters

- Declare all parameters in node constructors.
- Provide default values and validation.
- Use parameter callbacks for runtime updates when appropriate.

```cpp
this->declare_parameter("controller.max_joint_velocity", 1.57);
max_joint_velocity_ = this->get_parameter("controller.max_joint_velocity")
    .as_double();
```

## 8. Architecture Invariants

### ROS 2 Node Architecture

- **Nodes are executables**: Each ROS 2 node is a standalone executable with a clear responsibility.
- **Library layers use namespaces**: Reusable C++ code belongs in namespaced libraries (`fret::control`, `fret::planning`).
- **Separation of concerns**: Keep ROS 2 communication (topics/services) separate from core algorithms.
- **Testability**: Core algorithms should have non-ROS interfaces for unit testing.

### Control Architecture

- **Kinematics layer**: Forward/inverse kinematics are pure functions independent of ROS.
- **Jacobian computation**: Numerical or analytical Jacobian methods in `fret::control`.
- **Feedback control**: Controllers (PID, Jacobian-based) operate on error signals.
- **Command interface**: Controllers publish to standard ROS 2 `joint_trajectory_controller` or custom velocity commands.

### Planning Architecture

- **Trajectory generation**: Path planning produces waypoints in joint or task space.
- **Kinematic feasibility**: Planners respect joint limits and velocity constraints.
- **Closed-loop planning**: Future integration with vision for dynamic replanning.

## 9. ROS 2 Best Practices

### Composition

- Use component-based nodes (`rclcpp::Node` or `rclcpp_components`).
- Support both standalone executables and component composition.

### Quality of Service (QoS)

- Use appropriate QoS profiles for topics:
  - Real-time control: `rclcpp::SensorDataQoS()` (lossy, low latency)
  - State updates: `rclcpp::ServicesQoS()` (reliable)
  - Configuration: `rclcpp::ParametersQoS()` (reliable, transient local)

### Logging

- Use ROS 2 logging macros:
  - `RCLCPP_DEBUG`, `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`, `RCLCPP_FATAL`
- Include context in log messages: node name, operation, relevant values.
- Avoid excessive logging in high-frequency loops.

### Timing

- Use `rclcpp::Rate` for periodic loops.
- Use `rclcpp::Time` for timestamps, not `std::chrono` directly.
- For real-time nodes, consider using real-time executors.

## 10. Language — US English Only

All source code identifiers, comments, docstrings, documentation (Markdown),
configuration files (YAML/JSON), and commit messages **must use American
English** spelling. This rule applies to all human contributors and AI agents
without exception.

Common corrections:

| Use | Not |
|-----|-----|
| color | colour |
| behavior | behaviour |
| initialize / initialized | initialise / initialised |
| optimize / optimized / optimizer | optimise / optimised / optimiser |
| center / centered / centerline | centre / centred / centreline |
| meters | metres |
| normalize / normalized | normalise / normalised |
| analyze | analyse |
| finalize | finalise |
| visualize | visualise |
| minimize | minimise |
| maximize | maximise |

**Exception:** External library API parameters that use UK spelling must be left unchanged to avoid breaking calls.

## 11. Build and Dependency Management

### CMakeLists.txt

- Follow ament_cmake conventions.
- Declare all dependencies in `package.xml` and `CMakeLists.txt`.
- Use `find_package()` for all ROS 2 and external dependencies.
- Install all necessary files (launch, config, URDF, meshes) to `share/fret/`.

### Package Dependencies

- Minimize dependencies; prefer well-maintained ROS 2 packages.
- Document any non-standard dependencies in README.
- Use `rosdep` for dependency resolution.

## 12. Pre-flight Checklist

Before finishing **any** implementation task, run the consolidated gate script or
equivalent steps from [CONTRIBUTING.md](../CONTRIBUTING.md):

```bash
bash scripts/check/pre_push.sh
```

With `--skip-ros`, smoke and integration gates are omitted (useful without a local
ROS 2 install).

Individual steps:

```bash
bash scripts/check/formatting.sh
bash scripts/check/types.sh
bash scripts/tests/unit.sh              # requires built workspace + ROS
bash scripts/tests/smoke.sh             # requires ROS + xvfb
bash scripts/tests/integration.sh       # requires ROS + xvfb
```

CI workflows (all required on pull requests):

| Workflow | Trigger | Gate |
|---|---|---|
| `formatting.yml` | PR | Black, isort, clang-format |
| `type_check.yml` | PR | mypy strict on `src/` |
| `tests.yml` | PR | pytest + smoke tests |
| `integration.yml` | PR | `launch_testing` inter-node scenarios |

## 13. Git and Version Control

Commit messages, branch strategy, and pull request expectations are defined in
[CONTRIBUTING.md](../CONTRIBUTING.md). Summary:

- Conventional Commits format (`feat`, `fix`, `docs`, `test`, …)
- `main` is protected; changes via pull request
- Focused branches; all CI checks must pass before merge

---

These guidelines are mandatory for all development in FRET. When in doubt, refer to
[CONTRIBUTING.md](../CONTRIBUTING.md) for workflow, then this file for coding style,
then ROS 2 official documentation.
