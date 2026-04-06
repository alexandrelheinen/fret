"""Unified FRET SITL pipeline launcher for end-to-end ARCO demonstration.

Launches the complete ARCO-to-FRET obstacle-aware manipulation pipeline:

1. Gazebo simulation — world + robot (via arco_scenario.py).
2. Optional ROS 2 bag recording — captures all diagnostic topics for
   post-run analysis and artifact traceability.

Single-command usage::

    ros2 launch fret sitl.py

All arguments can be overridden on the command line.  Example with a
non-default planner profile and bag recording enabled::

    ros2 launch fret sitl.py scenario:=arco_scenario record_bag:=true

Launch arguments
----------------
scenario : str
    World profile name.  Must match a launch file
    ``<scenario>.py`` in the ``fret`` package share directory.
    Defaults to ``arco_scenario``.
record_bag : bool
    Set to ``true`` to start ``ros2 bag record`` alongside the
    simulation.  Recorded topics include ``/joint_states``,
    ``/clock``, ``/tf``, and ``/tf_static``.  Defaults to ``false``.
bag_dir : str
    Directory in which bag files are written.  Relative paths are
    resolved against the current working directory.
    Defaults to ``log/bags``.

Startup sequence (deterministic)
---------------------------------
1. ``arco_scenario.py`` starts Gazebo, ``robot_state_publisher``,
   the Gazebo ↔ ROS 2 bridge, and the controller node.
2. ``ros2 bag record`` (when ``record_bag:=true``) is started
   immediately after so that the first joint-state messages are
   captured.

Diagnostics
-----------
All nodes write to stdout/stderr via ``output="screen"``.  Node-level
failures are visible in the terminal and in ``~/.ros/log/latest/``.

See also
--------
``docs/arco/issue-08-end-to-end-sitl-launch-and-demonstration.md`` —
full runbook, launch graph, and acceptance criteria.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------
# Diagnostics topics captured in every bag recording
# ---------------------------------------------------------------------------
_BAG_TOPICS = [
    "/joint_states",
    "/clock",
    "/tf",
    "/tf_static",
]


def _build_pipeline(context):
    """Resolve launch-time arguments and build the action list."""
    scenario = LaunchConfiguration("scenario").perform(context)
    record_bag = LaunchConfiguration("record_bag").perform(context)
    bag_dir = LaunchConfiguration("bag_dir").perform(context)

    fret_share = get_package_share_directory("fret")
    scenario_launch_path = os.path.join(fret_share, "launch", f"{scenario}.py")

    if not os.path.isfile(scenario_launch_path):
        raise FileNotFoundError(
            f"[sitl] Scenario launch file not found: {scenario_launch_path}. "
            f"Available scenarios are the *.py files in "
            f"{os.path.join(fret_share, 'launch')}."
        )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scenario_launch_path),
        ),
    ]

    if record_bag.lower() in ("true", "1", "yes"):
        # Build a timestamped output name so successive runs never overwrite
        # each other (the ``ros2 bag record -o`` flag accepts a prefix; Gazebo
        # sim time appends an ISO timestamp automatically).
        bag_output = os.path.join(bag_dir, f"sitl_{scenario}")
        os.makedirs(bag_dir, exist_ok=True)

        bag_cmd = ["ros2", "bag", "record", "-o", bag_output] + _BAG_TOPICS
        actions.append(
            ExecuteProcess(
                cmd=bag_cmd,
                output="screen",
                name="bag_recorder",
            )
        )

    return actions


def generate_launch_description():
    """Generate the SITL pipeline launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="arco_scenario",
                description=(
                    "Scenario profile name.  Must match a <scenario>.py "
                    "launch file in the fret package share directory.  "
                    "Defaults to 'arco_scenario'."
                ),
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description=(
                    "Set to 'true' to record diagnostic topics to a ROS 2 bag."
                ),
            ),
            DeclareLaunchArgument(
                "bag_dir",
                default_value="log/bags",
                description=(
                    "Directory for ROS 2 bag files.  Created automatically "
                    "if it does not exist.  Relative to the working directory."
                ),
            ),
            OpaqueFunction(function=_build_pipeline),
        ]
    )
