"""Launch: Gazebo simulation with robot state publisher.

Usage::

    ros2 launch fret sim.py model:=scara

Arguments:
    model (str, default: scara)
        Robot model name.  Must match a XACRO file under
        ``fret/urdf/<model>.xacro``.
    world (str, default: empty.world)
        Gazebo world file name to load.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the Gazebo simulation configuration."""
    pkg_share = FindPackageShare("fret")
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="scara",
        description="Robot model name (must match urdf/<model>.xacro)",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="Gazebo world file name",
    )

    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", [LaunchConfiguration("model"), ".xacro"]]
    )
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros_share, "/launch/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-entity",
            LaunchConfiguration("model"),
            "-topic",
            "robot_description",
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            robot_state_publisher,
            joint_state_publisher,
            gazebo,
            spawn_entity,
        ]
    )
