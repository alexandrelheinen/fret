"""V10-1 integration smoke: PPP warehouse MuJoCo SITL launch.

Validates releases.md acceptance criterion V10-1:

    ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco

The launch must start all pipeline nodes and remain alive without fatal
errors long enough for perception, planning, and control to initialise.

Requires a built ROS 2 workspace and sourced overlay (see
``scripts/tests/integration.sh``).
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from collections.abc import Iterator

import pytest

_REQUIRED_NODES: frozenset[str] = frozenset(
    {
        "/mujoco_bridge_node",
        "/planner_node",
        "/controller_node",
        "/perception_bridge_node",
        "/scene_acquisition_node",
    }
)

_FATAL_PATTERNS: tuple[str, ...] = (
    "process has died",
    "process has finished",
    "No executable found",
    "package 'fret' not found",
    "Failed to load",
    "Traceback (most recent call last)",
)


def _ros2_node_names() -> set[str]:
    """Return the current ROS 2 node name list."""
    proc = subprocess.run(
        ["ros2", "node", "list"],
        capture_output=True,
        text=True,
        check=True,
        timeout=15,
    )
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def _terminate_process_group(proc: subprocess.Popen[str]) -> None:
    """Send SIGTERM to the launch process group and wait for exit."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=10)


@pytest.fixture()
def ppp_warehouse_sitl_launch() -> Iterator[subprocess.Popen[str]]:
    """Launch SC-v10 SITL and tear it down after the test."""
    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl")
    env.setdefault("PYOPENGL_PLATFORM", "egl")

    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "fret",
            "sitl.py",
            "scenario:=ppp_warehouse",
            "model:=ppp",
            "backend:=mujoco",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    try:
        yield proc
    finally:
        _terminate_process_group(proc)
        if proc.stdout is not None:
            proc.stdout.close()


@pytest.mark.timeout(120)
def test_v10_1_ppp_warehouse_mujoco_sitl_starts(
    ros_context: None,
    ppp_warehouse_sitl_launch: subprocess.Popen[str],
) -> None:
    """V10-1: MuJoCo PPP SITL launch stays up with all core nodes present."""
    del ros_context  # fixture ensures rclpy is initialised for ros2 CLI
    proc = ppp_warehouse_sitl_launch

    deadline = time.monotonic() + 45.0
    nodes_ok = False
    log_lines: list[str] = []

    while time.monotonic() < deadline:
        if proc.poll() is not None:
            break
        if proc.stdout is not None and proc.stdout.readable():
            line = proc.stdout.readline()
            if line:
                log_lines.append(line.rstrip())
        nodes = _ros2_node_names()
        if _REQUIRED_NODES.issubset(nodes):
            nodes_ok = True
            break
        time.sleep(0.5)

    log_tail = "\n".join(log_lines[-40:])
    assert proc.poll() is None, (
        "SITL launch exited before all nodes were ready.\n" f"{log_tail}"
    )
    assert nodes_ok, (
        "Timed out waiting for PPP warehouse SITL nodes.\n"
        f"Expected: {sorted(_REQUIRED_NODES)}\n"
        f"Seen: {sorted(_ros2_node_names())}\n"
        f"{log_tail}"
    )

    combined_log = "\n".join(log_lines)
    for pattern in _FATAL_PATTERNS:
        assert (
            pattern not in combined_log
        ), f"Launch log contains fatal pattern {pattern!r}.\n{log_tail}"
