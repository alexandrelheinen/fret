# Development environment notes

Durable, non-obvious notes for working in this repository, especially in
cloud/sandboxed agent environments. Standard build/test/run commands live
in [README.md](../README.md) and [CONTRIBUTING.md](../CONTRIBUTING.md).

## Cloud agent base image

The base VM snapshot already has ROS 2 Jazzy Desktop, headless-GL
libraries, `xvfb`, and a complete C/C++ toolchain installed system-wide.
The startup update script only refreshes Python deps
(`pip install -e ".[dev,sim]"`, installed to `~/.local`; this also
downloads the pinned ARCO release wheel, so it needs network access).

## Sourcing ROS

ROS is not sourced by default. Run `source /opt/ros/jazzy/setup.bash` and,
after building, `source install/setup.bash`. `scripts/build.sh` and
`scripts/tests/*.sh` source these themselves.

## pytest vs. ROS plugins (important)

The pip `pytest` (>=8, currently 9.x) is incompatible with ROS Jazzy's
`launch_testing` / `launch_ros` pytest plugins (they use the removed
`path` hook arg). With the ROS overlay sourced, plain `pytest`, and thus
`scripts/tests/unit.sh` as written, aborts with a `PluginValidationError`.
Run unit tests with those plugins disabled:

```bash
python3 -m pytest tests/ --ignore=tests/integration -p no:launch_testing -p no:launch_ros
```

The same incompatibility blocks `tests/integration/` (launch_testing
based) under pytest 9.

## Headless MuJoCo

Viewer/render scripts need EGL. Export
`MUJOCO_GL=egl PYOPENGL_PLATFORM=egl` (the system EGL libs are in the
snapshot), e.g. for `scripts/render_mujoco.py` / `scripts/video.sh`.

## render_mujoco motion

The default controller-tracked showcase path
(`simulate_tracked_trajectory`) barely advances with the current
controller config and renders a near-static clip; pass `--no-tracking` to
render visible planned-path (ARCO RRT*) motion.

## Known pre-existing breakages (not environment issues)

The pinned ARCO v0.5.0 wheel provides the compiled classical
path-following MPCC and joint-space MPC. It carries the abi3 extension,
so `pip install -e ".[sim]"` needs no Rust toolchain on Linux x86_64,
macOS arm64 or Windows x86_64; any other platform falls back to the
source distribution and does need one. CasADi is gone with the `mpc`
extra, and ARCO now pulls SciPy. Pre-existing: `mypy` reports a missing `Any`
import in `planner_node_ros.py`; and the ROS `planner_node` crashes at
launch because `declare_parameter("start_configuration", [])` is inferred
as `BYTE_ARRAY` under rclpy Jazzy. Other nodes (mujoco_bridge, controller,
perception, scene) launch cleanly.
