"""ARCO obstacle scenario launcher.

Launches the full ARCO planning-and-tracking validation scenario:
- Gazebo world: arco_scenario.sdf (floor + 5 static obstacles + region markers)
- Robot: UR3 (6-DOF industrial manipulator, reach ≈ 500 mm)
- Nodes: robot_state_publisher, gz_bridge, controller

Single-command usage::

    ros2 launch fret arco_scenario.py

All world and robot arguments can be overridden on the command line;
defaults match the scenario specification in
docs/arco/spec-issue-02-world-scenario.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate the ARCO obstacle scenario launch description."""
    fret_share = get_package_share_directory("fret")
    # sim.py is the authoritative generic simulation launcher; arco_scenario.py
    # is an opinionated composition of it.  If sim.py is ever renamed, update
    # this path and the IncludeLaunchDescription below accordingly.
    sim_launch = os.path.join(fret_share, "launch", "sim.py")
    world_file = os.path.join(fret_share, "worlds", "arco_scenario.sdf")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "model": "ur3",
                    "world": world_file,
                }.items(),
            ),
        ]
    )
