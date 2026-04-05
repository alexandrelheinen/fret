# CLAUDE Instructions

This repository uses this file as persistent guidance for Claude-based coding agents.

## Mandatory Standard

Use [docs/guidelines.md](docs/guidelines.md) as the required coding and architecture reference for all changes.

FRET is a ROS 2 workspace with Python and C++ components. Respect ROS 2 conventions and the architectural boundaries defined in the guidelines.

## Required Practices

- Follow ROS 2 best practices for node design, topic naming, and QoS settings.
- Respect the namespace-mirrors-directory convention for C++ code.
- Keep launch files Python-based and parametric.
- Add tests with behavior changes and keep test layout mirrored with source layout.
- Write Google-style docstrings for Python and Doxygen comments for C++.
- Use explicit typing for public methods and APIs.
- Keep changes small, composable, and aligned with existing style.
- Ensure URDF/XACRO files are properly formatted and use parametric descriptions.

## ROS 2 Specific Requirements

- **Nodes**: Each node has a single, well-defined responsibility.
- **Topics**: Use standard ROS 2 message types when possible; custom messages only when necessary.
- **Parameters**: Declare all parameters with defaults and validation.
- **Launch files**: Support command-line arguments for model selection and configuration.
- **Build system**: Use ament_cmake; update both `CMakeLists.txt` and `package.xml` for new dependencies.
- **Transforms**: Use TF2 for coordinate frame transformations.

## Delivery Quality

Before finishing a task, verify:

- Relevant tests pass (Python unit tests at minimum).
- New public APIs are documented and typed.
- File organization and imports comply with [docs/guidelines.md](docs/guidelines.md).
- ROS 2 nodes launch without errors (`ros2 launch` smoke test).
- An imperative order (do, implement, make, add...) is not only about writing the code. It must include all the V-cycle.

### Respect the V-cycle

As indicated above, all work of an AI must include all the descending and ascending steps of the cycle. For each row numbered below, the two actions (descend and ascend) must be done **at the same time**:

1. **Add documentation of the work, feature, bug fix**: Use GitHub issues if possible, otherwise go directly into the GitHub PR and document every step in comments. This includes: goal/objectives and acceptance criteria.

2. **Implement the architecture** of the code (classes, public interface, file organization, dependencies) and the **(functional) unit tests at the same time**. The testing must come first then coding: the performance of the algorithm is independent of its implementation. By reading the acceptance criteria above, you must already know which values to expect.

3. **Do the coding**: This is the 3rd step: Fill the stubs left by the architecture definition. Implement algorithms, data structures, and private/local utilities. Add unit testing for private functions as well (fine testing/non-functional tests).

4. **Then, run the tests**: Ideally proving 100% coverage (at least 90% would be great!). If this step fails, go back to step 2: Review your architecture, your functional tests, and go back to the cycle.

5. **Implement high-level simulations** if all the testing are passing. Add visual inspection (either images or videos) if applicable. Add material for the presentation and documentation of the tool. Add the appropriate documentation of the newly implemented feature, or fix the lines affected by the changes. All GitHub workflows must pass: both at push and release! If something is wrong in this step, go back to step number 1.

This completes the V-cycle. Once the acceptance criteria are met and all the GitHub workflows (autotests) are passing (both at push and release, test them all locally or add the tooling to test it), you can push your branch and trigger the review.

## ROS 2 Workspace Workflow

When making changes that affect the ROS 2 workspace:

1. **Build**: `./scripts/build.sh`
2. **Source**: `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
3. **Test**: Run unit tests and launch smoke tests
4. **Validate**: Check that nodes start without errors

## Common Pitfalls to Avoid

- Don't mix ROS 2 communication logic with core algorithms (keep algorithms testable).
- Don't forget to install new files (URDF, config, launch) in `CMakeLists.txt`.
- Don't use blocking calls in high-frequency control loops.
- Don't hardcode parameters; use ROS 2 parameter system or config files.
- Don't skip the V-cycle steps; all code must have tests.
- Don't violate namespace conventions for C++ code.
