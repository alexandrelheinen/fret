"""Jacobian-based trajectory tracking controller at 50 Hz.

Subscribes to the ``/planned_trajectory`` topic (``JointTrajectory``) and
drives the simulated / physical robot at a fixed 50 Hz timer.  Tracks the
active trajectory segment using a Jacobian pseudo-inverse velocity controller
with configurable damping factor.

FSM states: ``IDLE → TRACKING → HALTED`` — see docs/interfaces.md for the
full transition table.

Publishes:
    /joint_commands (``trajectory_msgs/JointTrajectory``)
    /controller_fault (``std_msgs/Bool``) — ``True`` when EE error > 20 mm.

Satisfies requirements FR-CTL-01 through FR-CTL-06.
"""

from __future__ import annotations


class ControllerNode:
    """ROS 2 node implementing Jacobian-based 50 Hz trajectory tracking.

    Args:
        model: Robot model name used to instantiate the ``Kinematics`` engine.
        config_path: Absolute path to the controller YAML config file
            (e.g. ``config/controllers/jacobian.yml``).

    Note:
        The full ROS 2 node constructor (``super().__init__``) and timer/
        subscription setup are part of the Level 4 implementation.  This
        stub defines the public interface only.
    """

    def __init__(self, model: str, config_path: str) -> None:
        raise NotImplementedError
