"""Robot kinematic models for the ARCO-FRET planning pipeline.

Provides the abstract :class:`RobotModel` base class and the concrete
:class:`ScaraModel` implementation.  All classes are pure Python and
independent of the ROS 2 runtime — the ROS bridge lives in
:mod:`fret.ros`.

Exported symbols:
    - :class:`RobotModel` — abstract base for all robot kinematic models.
    - :class:`ScaraModel` — SCARA R-R-P-R (4-DOF) kinematic model.
"""

from fret.robots.robot_model import RobotModel
from fret.robots.scara_model import ScaraModel

__all__ = ["RobotModel", "ScaraModel"]
