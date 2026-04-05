# Contributing to FRET

Thank you for contributing to the Full-stack Robotic Effector Trajectories project.

## Prerequisites

- Ubuntu 24.04 (for ROS 2 Jazzy)
- ROS 2 Jazzy Desktop
- Python 3.12+
- Git
- Basic familiarity with ROS 2 concepts (nodes, topics, services, launch files)

## Setup

```bash
git clone https://github.com/alexandrelheinen/fret.git
cd fret
./scripts/install.sh -y
./scripts/setup.sh -y
./scripts/build.sh
```

## Development Workflow

1. Create a feature branch from main.
2. Implement the change with focused commits following the V-cycle.
3. Add or update tests (Python unit tests required, integration tests recommended).
4. Run local validation before opening a pull request.
5. Ensure all CI checks pass.

## Local Validation

### Build and Source

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Run Tests

```bash
# Python unit tests
python3 -m unittest discover tests -v
```

### Format Code

```bash
# Python formatting
python -m black --check src/
python -m isort --check-only src/

# C++ formatting
find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format --dry-run --Werror
```

### Smoke Tests

Verify that launch files start without errors:

```bash
# RViz visualization (terminate after visual confirmation)
ros2 launch fret view.py model:=scara

# Gazebo simulation (terminate after visual confirmation)
ros2 launch fret sim.py model:=scara
```

## Coding Rules

The authoritative coding standard is [docs/guidelines.md](docs/guidelines.md).

All contributions, including AI-assisted changes, must follow this file.

### Key Conventions

- **Python**: Google-style docstrings, type annotations, `black` + `isort` formatting
- **C++**: Doxygen comments, ROS 2 C++ style guide, `clang-format` formatting
- **ROS 2**: Standard message types, proper QoS profiles, parameter-based configuration
- **Testing**: V-cycle development, unit tests required, aim for 90%+ coverage
- **Physical variables**: `who_what` naming (e.g., `max_joint_velocity`, not `velocity_max` or `vel_max`)

## V-Cycle Development

FRET follows a rigorous V-cycle development process. Every task must include:

1. **Requirements**: Document goals and acceptance criteria (GitHub issue or PR description)
2. **Architecture + Functional Tests**: Design interfaces and write tests **before** implementation
3. **Implementation + Fine Tests**: Fill stubs and add private function tests
4. **Validation**: Run tests (target 100% coverage, minimum 90%)
5. **Integration**: Update documentation, verify launches work, ensure CI passes

**Never** submit code without corresponding tests. See [docs/guidelines.md](docs/guidelines.md) section 6 for full details.

## Pull Request Checklist

Before opening a PR, verify:

- [ ] Behavior changes are covered by tests
- [ ] Public APIs are typed and documented
- [ ] Python formatting passes (`black --check` and `isort --check-only`)
- [ ] C++ formatting passes (`clang-format --dry-run`)
- [ ] Unit tests pass (`python3 -m unittest`)
- [ ] Launch files start without errors
- [ ] Documentation updated (README, guidelines, or docs/ as appropriate)
- [ ] Commit messages follow conventional commits format
- [ ] Branch is up-to-date with main

## Documentation

Update documentation when adding or changing behavior:

- [README.md](README.md) for user-facing changes (installation, usage, examples)
- [docs/guidelines.md](docs/guidelines.md) for new coding rules or conventions
- [docs/roadmap.md](docs/roadmap.md) for future work updates
- Inline code documentation (docstrings/Doxygen) for all public APIs

## ROS 2 Specific Contributions

When contributing ROS 2 components:

### Adding a New Node

1. Create C++ source in `src/fret/src/<node_name>.cpp` or Python script
2. Update `CMakeLists.txt` to build and install the executable
3. Add corresponding launch file in `src/fret/launch/`
4. Document node's topics, parameters, and purpose

### Adding URDF/XACRO Models

1. Create XACRO file in `src/fret/urdf/<model>.xacro`
2. Optional: Add mesh generator in `src/fret/mesh/<model>.py`
3. Optional: Add RViz config in `src/fret/rviz/<model>.rviz`
4. Update CMakeLists.txt to install generated files
5. Test with `ros2 launch fret view.py model:=<model>`

### Adding Configuration

1. Create YAML file in `src/fret/config/`
2. Add comments explaining each parameter
3. Update CMakeLists.txt to install config file
4. Reference from launch file or node

### Adding Dependencies

1. Add to `package.xml` (both `<depend>` tag and rosdep key)
2. Add `find_package()` call in `CMakeLists.txt`
3. Document in README if it's not a standard ROS 2 package

## Commit Messages

Follow conventional commits format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Example**:
```
feat(control): add Jacobian pseudoinverse controller

Implement numerical Jacobian computation with damped least squares
for singularity robustness. Controller tested with SCARA model
in simulation.

Closes #23
```

## Getting Help

- Review existing code and tests for examples
- Check [docs/guidelines.md](docs/guidelines.md) for detailed conventions
- Open a GitHub issue for questions or clarifications
- Reference [ROS 2 documentation](https://docs.ros.org/) for ROS-specific topics

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
