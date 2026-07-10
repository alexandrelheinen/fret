"""Shared fixtures for integration tests.

All tests in this directory require a live ROS 2 context.  The
``ros_context`` fixture initialises ``rclpy`` once for the entire session
and shuts it down afterwards.  Individual tests can create and destroy
``rclpy.node.Node`` objects using the ``test_node`` fixture.
"""

from __future__ import annotations

from collections.abc import Generator

import numpy as np
import pytest
import rclpy
from rclpy.node import Node

# ARC planners call ``np.random.default_rng()`` without a seed; pin for CI.
_PLANNER_RNG_SEED = 11


@pytest.fixture(scope="session", autouse=True)
def _deterministic_planner_rng() -> Generator[None, None, None]:
    """Seed ARC planner RNG so physics integration gates are reproducible."""
    original_default_rng = np.random.default_rng

    def _seeded_default_rng(seed: int | None = None) -> np.random.Generator:
        return original_default_rng(
            _PLANNER_RNG_SEED if seed is None else seed
        )

    np.random.default_rng = _seeded_default_rng
    yield
    np.random.default_rng = original_default_rng


@pytest.fixture(scope="session")
def ros_context() -> Generator[None, None, None]:
    """Initialise rclpy once per test-session (matches CI lifecycle)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def test_node(ros_context: None) -> Generator[Node, None, None]:
    """Create a throwaway ROS 2 node for a single test, then destroy it."""
    node = Node("fret_integration_test")
    yield node
    node.destroy_node()
