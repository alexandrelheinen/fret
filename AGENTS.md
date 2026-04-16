# AGENTS Instructions

This file defines repository-wide instructions for coding agents.

## Scope

Applies to all automated agents working in this repository.

## Source of Truth

Follow [docs/guidelines.md](docs/guidelines.md) as the authoritative standard for:

- Architecture and package structure (ROS 2 ament_cmake layout)
- Python and C++ naming conventions
- ROS 2 resource naming (topics, services, parameters, nodes)
- Docstring and documentation style
- Formatting and typing
- Testing and quality gates (V-cycle)

## Agent Policy

- Do not introduce patterns that violate [docs/guidelines.md](docs/guidelines.md).
- Keep edits narrow and relevant to the task.
- When changing behavior, update tests in the mirrored tests structure.
- When adding public APIs, include type annotations and proper documentation (Google-style for Python, Doxygen for C++).
- Prefer project-local conventions over generic defaults.
- Respect ROS 2 ecosystem standards and best practices.

An imperative order (do, implement, make, add...) is not only about writing the code. It must include all the V-cycle.

### Respect the V-cycle

As indicated above, all work of an AI must include all the descending and ascending steps of the cycle. For each row numbered below, the two actions (descend and ascend) must be done **at the same time**:

1. **Add documentation of the work, feature, bug fix**: Use GitHub issues if possible, otherwise go directly into the GitHub PR and document every step in comments. This includes: goal/objectives and acceptance criteria.

2. **Implement the architecture** of the code (classes, public interface, file organization, dependencies) and the **(functional) unit tests at the same time**. The testing must come first then coding: the performance of the algorithm is independent of its implementation. By reading the acceptance criteria above, you must already know which values to expect.

3. **Do the coding**: This is the 3rd step: Fill the stubs left by the architecture definition. Implement algorithms, data structures, and private/local utilities. Add unit testing for private functions as well (fine testing/non-functional tests).

4. **Then, run the tests**: Ideally proving 100% coverage (at least 90% would be great!). If this step fails, go back to step 2: Review your architecture, your functional tests, and go back to the cycle.

5. **Implement high-level simulations** if all the testing are passing. Add visual inspection (either images or videos) if applicable. Add material for the presentation and documentation of the tool. Add the appropriate documentation of the newly implemented feature, or fix the lines affected by the changes. All GitHub workflows must pass: both at push and release! If something is wrong in this step, go back to step number 1.

This completes the V-cycle. Once the acceptance criteria are met and all the GitHub workflows (autotests) are passing (both at push and release, test them all locally or add the tooling to test it), you can push your branch and trigger the review.

## ROS 2 Specific Requirements

When working with ROS 2 components:

- **Build system**: Update both `CMakeLists.txt` and `package.xml` when adding dependencies or files.
- **Installation**: Ensure new files (launch, config, URDF, meshes) are installed via `install()` directives.
- **Namespaces**: C++ code must follow namespace-mirrors-directory convention.
- **Nodes**: Each node has a clear, single responsibility.
- **Parameters**: All configurable values must be ROS 2 parameters or YAML config files.
- **Testing**: Unit tests use mocks to avoid ROS runtime; integration tests use launch_testing.

## Pre-commit Validation

Before completing any task, run the pre-flight checklist from [docs/guidelines.md](docs/guidelines.md):

1. Build workspace: `./scripts/build.sh`
2. Source environment: `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
3. Run unit tests: `pytest tests/ -v`
4. Check Python formatting: `black --check src/ && isort --check-only src/`
5. Check C++ formatting: `find src -name '*.cpp' -o -name '*.hpp' | xargs clang-format --dry-run --Werror`
6. Launch smoke tests: Verify nodes start without errors

## Conflict Resolution

When instructions conflict, resolve in this order:

1. Direct maintainer request in the active task
2. [docs/guidelines.md](docs/guidelines.md)
3. ROS 2 official documentation
4. This file
