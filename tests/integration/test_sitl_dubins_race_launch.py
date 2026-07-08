"""V11-1 integration smoke: Dubins race MuJoCo SITL launch.

Validates releases.md acceptance criterion V11-1 launch path:

    ros2 launch fret sitl.py scenario:=dubins_race model:=dubins backend:=mujoco

Requires a built ROS 2 workspace and sourced overlay (see
``scripts/tests/integration.sh``).
"""

from __future__ import annotations

import io
import os
import select
import signal
import subprocess
import time
from collections.abc import Iterator

import pytest

_REQUIRED_NODES: frozenset[str] = frozenset(
    {
        "/dubins_race_node",
    }
)

_FATAL_PATTERNS: tuple[str, ...] = (
    "process has died",
    "process has finished",
    "No executable found",
    "package 'fret' not found",
    "Failed to load",
    "Traceback (most recent call last)",
    "Dubins race planning failed",
)


def _ros2_node_names() -> set[str]:
    proc = subprocess.run(
        ["ros2", "node", "list"],
        capture_output=True,
        text=True,
        check=True,
        timeout=15,
    )
    return {line.strip() for line in proc.stdout.splitlines() if line.strip()}


def _terminate_process_group(proc: subprocess.Popen[str]) -> None:
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


def _drain_launch_output(stdout: io.TextIO | None) -> list[str]:
    """Return any complete log lines currently buffered on the launch pipe."""
    if stdout is None:
        return []
    lines: list[str] = []
    while True:
        ready, _, _ = select.select([stdout], [], [], 0)
        if not ready:
            break
        line = stdout.readline()
        if not line:
            break
        lines.append(line.rstrip())
    return lines


@pytest.fixture()
def dubins_race_sitl_launch() -> Iterator[subprocess.Popen[str]]:
    """Launch SC-v11 SITL and tear it down after the test."""
    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl")
    env.setdefault("PYOPENGL_PLATFORM", "egl")

    proc = subprocess.Popen(
        [
            "ros2",
            "launch",
            "fret",
            "sitl.py",
            "scenario:=dubins_race",
            "model:=dubins",
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


@pytest.mark.timeout(180)
def test_v11_1_dubins_race_mujoco_sitl_starts(
    ros_context: None,
    dubins_race_sitl_launch: subprocess.Popen[str],
) -> None:
    """V11-1: MuJoCo Dubins race SITL launch stays up with race node present."""
    del ros_context
    proc = dubins_race_sitl_launch

    deadline = time.monotonic() + 120.0
    nodes_ok = False
    log_lines: list[str] = []

    while time.monotonic() < deadline:
        if proc.poll() is not None:
            break
        log_lines.extend(_drain_launch_output(proc.stdout))
        nodes = _ros2_node_names()
        if _REQUIRED_NODES.issubset(nodes):
            nodes_ok = True
            break
        time.sleep(0.5)

    log_lines.extend(_drain_launch_output(proc.stdout))
    log_tail = "\n".join(log_lines[-40:])
    assert proc.poll() is None, (
        "SITL launch exited before the race node was ready.\n" f"{log_tail}"
    )
    assert nodes_ok, (
        "Timed out waiting for Dubins race SITL nodes.\n"
        f"Expected: {sorted(_REQUIRED_NODES)}\n"
        f"Seen: {sorted(_ros2_node_names())}\n"
        f"{log_tail}"
    )

    combined_log = "\n".join(log_lines)
    for pattern in _FATAL_PATTERNS:
        assert (
            pattern not in combined_log
        ), f"Launch log contains fatal pattern {pattern!r}.\n{log_tail}"
