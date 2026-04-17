"""Shared fixtures for integration tests.

All tests in this directory require a live ROS 2 context.  The
``ros_context`` fixture initialises ``rclpy`` once for the entire session
and shuts it down afterwards.  Individual tests can create and destroy
``rclpy.node.Node`` objects using the ``test_node`` fixture.
"""

from __future__ import annotations

from collections.abc import Generator

import pytest
import rclpy
from rclpy.node import Node


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
