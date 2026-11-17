"""Serial bridge to the robot hardware for hardware-in-the-loop (v2.x).

Subscribes to ``/joint_commands``, serialises each setpoint, and forwards it
over a serial link to the robot. Also reads feedback and re-publishes it as
``/joint_states``.

The v2.x hardware target (the actual robot, bus, and protocol) is defined
starting v2.0 (see ``docs/hardware/README.md``); FR-HW-01..03 below are
superseded in v2.0, pending redefinition, and are not renumbered.

This node is only active in a future ``hardware.py`` launch configuration.
It is **not** part of the v1.x simulation / CV line — see
``docs/releases.md`` § v2.x. Satisfies requirements FR-HW-01 through FR-HW-03
(superseded in v2.0, pending redefinition).
"""

from __future__ import annotations


class BridgeNode:
    """ROS 2 bridge node between the ROS graph and the robot hardware.

    Args:
        port: Serial port path, e.g. ``"/dev/ttyUSB0"``.
        baud_rate: Serial baud rate; must match the hardware configuration.

    Note:
        The full ROS 2 node constructor and serial-port setup are part of
        the Level 4 implementation, which depends on the v2.0 hardware
        target definition. This stub defines the public interface only.
    """

    def __init__(self, port: str, baud_rate: int) -> None:
        raise NotImplementedError
