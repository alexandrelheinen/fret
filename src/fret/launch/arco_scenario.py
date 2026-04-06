"""ARCO obstacle scenario launcher.

Launches the full ARCO planning-and-tracking validation scenario:
- Gazebo world: arco_scenario.sdf (floor + 3 static box obstacles + region markers)
- Robot: SCARA (R-R-P-R, 4-DOF tabletop manipulator, reach 600 mm)
- Nodes: robot_state_publisher, gz_bridge, controller, perception_bridge
- Static TF: obstacle_box_a/b/c frames published as static transforms

Single-command usage::

    ros2 launch fret arco_scenario.py

All world and robot arguments can be overridden on the command line;
defaults match the scenario specification in
docs/arco/spec-issue-02-world-scenario.md.

Obstacle TF frames
------------------
The three box obstacles are static, so their poses are published as
``tf2_ros/static_transform_publisher`` nodes rather than via the Gazebo
bridge.  Positions are taken directly from ``arco_scenario.sdf``; they
are expressed in the ``world`` frame (which coincides with the robot
``base_link`` origin per REP-105).

  obstacle_box_a : (0.25,  0.00, 0.05)  centre block
  obstacle_box_b : (0.35,  0.12, 0.05)  left block
  obstacle_box_c : (0.35, -0.12, 0.05)  right block
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Obstacle TF frames (must match arco_scenario.sdf <pose> entries exactly)
# ---------------------------------------------------------------------------
# Each entry: (child_frame_id, x, y, z)  – orientation is always identity.
_OBSTACLE_FRAMES = [
    ("obstacle_box_a", "0.25", "0.00", "0.05"),
    ("obstacle_box_b", "0.35", "0.12", "0.05"),
    ("obstacle_box_c", "0.35", "-0.12", "0.05"),
]


def generate_launch_description():
    """Generate the ARCO obstacle scenario launch description."""
    fret_share = get_package_share_directory("fret")
    # sim.py is the authoritative generic simulation launcher; arco_scenario.py
    # is an opinionated composition of it.  If sim.py is ever renamed, update
    # this path and the IncludeLaunchDescription below accordingly.
    sim_launch = os.path.join(fret_share, "launch", "sim.py")
    world_file = os.path.join(fret_share, "worlds", "arco_scenario.sdf")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "model": "scara",
                "world": world_file,
            }.items(),
        ),
    ]

    # Publish static TF frames for each obstacle so that PerceptionBridgeNode
    # can look up obstacle poses via tf2 without requiring the Gazebo
    # Pose_V → TFMessage bridge (which emits invalid empty-frame transforms).
    for frame_id, x, y, z in _OBSTACLE_FRAMES:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{frame_id}",
                output="screen",
                arguments=[
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    frame_id,
                    "--x",
                    x,
                    "--y",
                    y,
                    "--z",
                    z,
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1",
                ],
            )
        )

    return LaunchDescription(actions)
