"""Shared fixtures for integration tests.

All tests in this directory require a live ROS 2 context.  The
``ros_context`` fixture initialises ``rclpy`` once for the entire session
and shuts it down afterwards.  Individual tests can create and destroy
``rclpy.node.Node`` objects using the ``test_node`` fixture.
"""

from __future__ import annotations

import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope="session")
def ros_context() -> object:  # type: ignore[return]
    """Initialise rclpy once per test-session (matches CI lifecycle)."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def test_node(ros_context: object) -> object:  # type: ignore[return]
    """Create a throwaway ROS 2 node for a single test, then destroy it."""
    node = Node("fret_integration_test")
    yield node
    node.destroy_node()
