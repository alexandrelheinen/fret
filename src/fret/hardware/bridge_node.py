"""Serial / Micro-ROS bridge to Arduino for hardware-in-the-loop (v2.x).

Subscribes to ``/joint_commands``, serialises each setpoint, and forwards it
over a serial link to embedded firmware.  Also reads encoder feedback and
re-publishes it as ``/joint_states``.

This node is only active in a future ``hardware.py`` launch configuration.
It is **not** part of the v1.3–v1.5 simulation / CV line — see
``docs/releases.md`` § v2.x. Satisfies requirements FR-HW-01 through FR-HW-03.
"""

from __future__ import annotations


class BridgeNode:
    """ROS 2 bridge node between the ROS graph and Arduino hardware.

    Args:
        port: Serial port path, e.g. ``"/dev/ttyUSB0"``.
        baud_rate: Serial baud rate; must match firmware configuration.

    Note:
        The full ROS 2 node constructor and serial-port setup are part of
        the Level 4 implementation.  This stub defines the public interface.
    """

    def __init__(self, port: str, baud_rate: int) -> None:
        raise NotImplementedError
