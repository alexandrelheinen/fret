# Copilot Instructions for FRET

These instructions apply to all GitHub Copilot chat and coding-agent interactions in this repository.

## Mandatory Rule

Always follow [docs/guidelines.md](../docs/guidelines.md) as the authoritative coding standard.

If there is any conflict between an ad hoc request and the guidelines, prioritize the guidelines unless the user explicitly asks to change the guidelines themselves.

## ROS 2 Context

FRET is a ROS 2 workspace with Python and C++ components for robotic manipulation. Respect ROS 2 conventions and best practices throughout all contributions.

## Required Behaviors

- Preserve ROS 2 architecture: nodes have single responsibilities, library code is separate from ROS communication.
- Follow namespace-mirrors-directory convention for C++ code.
- Keep launch files Python-based and parametric.
- Add or update tests alongside behavior changes (Python unit tests required, integration tests recommended).
- Use Google-style docstrings for Python and Doxygen comments for C++.
- Keep formatting compliant with Black, isort, and clang-format.
- Prefer minimal, focused diffs and avoid unrelated refactors.
- Update CMakeLists.txt and package.xml when adding dependencies or files.
- Ensure new files (URDF, config, launch) are installed via CMake install() directives.

## ROS 2 Specific Requirements

- **Nodes**: Each node has a well-defined, single responsibility.
- **Topics/Services**: Use standard ROS 2 message types when possible.
- **Parameters**: All configurable values must be ROS 2 parameters or YAML config files.
- **QoS**: Use appropriate QoS profiles (sensor data, services, parameters).
- **Transforms**: Use TF2 for coordinate transformations.
- **Namespace conventions**: C++ namespaces mirror directory structure (`fret::control`, `fret::planning`, etc.).

## Validation Checklist Before Finalizing

- Tests relevant to the change pass locally.
- Public APIs have typing and documentation.
- Python formatting validated: `black --check src/ && isort --check-only src/`
- C++ formatting validated: `clang-format --dry-run --Werror`
- Launch files start without errors (smoke test).
- Changes remain consistent with [docs/guidelines.md](../docs/guidelines.md).
- An imperative order (do, implement, make, add...) is not only about writing the code. It must include all the V-cycle.

### Respect the V-cycle

As indicated above, all work of an AI must include all the descending and ascending steps of the cycle. For each row numbered below, the two actions (descend and ascend) must be done **at the same time**:

1. **Add documentation of the work, feature, bug fix**: Use GitHub issues if possible, otherwise go directly into the GitHub PR and document every step in comments. This includes: goal/objectives and acceptance criteria.

2. **Implement the architecture** of the code (classes, public interface, file organization, dependencies) and the **(functional) unit tests at the same time**. The testing must come first then coding: the performance of the algorithm is independent of its implementation. By reading the acceptance criteria above, you must already know which values to expect.

3. **Do the coding**: This is the 3rd step: Fill the stubs left by the architecture definition. Implement algorithms, data structures, and private/local utilities. Add unit testing for private functions as well (fine testing/non-functional tests).

4. **Then, run the tests**: Ideally proving 100% coverage (at least 90% would be great!). If this step fails, go back to step 2: Review your architecture, your functional tests, and go back to the cycle.

5. **Implement high-level simulations** if all the testing are passing. Add visual inspection (either images or videos) if applicable. Add material for the presentation and documentation of the tool. Add the appropriate documentation of the newly implemented feature, or fix the lines affected by the changes. All GitHub workflows must pass: both at push and release! If something is wrong in this step, go back to step number 1.

This completes the V-cycle. Once the acceptance criteria are met and all the GitHub workflows (autotests) are passing (both at push and release, test them all locally or add the tooling to test it), you can push your branch and trigger the review.

## Common Pitfalls to Avoid

- Don't mix ROS 2 communication logic with core algorithms (keep algorithms testable).
- Don't forget to install new files in CMakeLists.txt.
- Don't use blocking calls in high-frequency control loops.
- Don't hardcode parameters; use ROS 2 parameter system or config files.
- Don't skip the V-cycle steps; all code must have tests.
- Don't violate C++ namespace conventions.
