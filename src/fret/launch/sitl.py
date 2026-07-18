"""Launch: full SITL pipeline (sim + planning + control + scene).

This is the main entry point for ``fretsim``.

Usage::

    ros2 launch fret sitl.py scenario:=dubins_race model:=dubins backend:=mujoco
    ros2 launch fret sitl.py scenario:=omx_reach model:=open_manipulator_x backend:=mujoco

Arguments:
    model (str, default: dubins)
        Robot model name.
    scenario (str, default: dubins_race)
        Scenario YAML stem under ``fret/config/scenarios/<scenario>.yml``.
    physics_mode (str, default: false)
        When ``true``, MuJoCo advances with ``mj_step`` and actuators (v1.2+).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    OrSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the SITL pipeline."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="dubins",
        description="Robot model name",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="dubins_race",
        description="Scenario YAML stem (config/scenarios/<scenario>.yml)",
    )
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="mujoco",
        description="Simulator backend: gazebo | mujoco",
    )
    physics_mode_arg = DeclareLaunchArgument(
        "physics_mode",
        default_value="false",
        description="Enable MuJoCo physics SITL (mj_step + actuators)",
    )

    is_mujoco = EqualsSubstitution(LaunchConfiguration("backend"), "mujoco")
    is_gazebo = UnlessCondition(is_mujoco)
    is_dubins_race = EqualsSubstitution(
        LaunchConfiguration("scenario"), "dubins_race"
    )
    is_omx_model = OrSubstitution(
        EqualsSubstitution(LaunchConfiguration("model"), "open_manipulator_x"),
        EqualsSubstitution(LaunchConfiguration("model"), "omx"),
    )

    is_standard_mujoco = PythonExpression(
        [
            "'",
            LaunchConfiguration("backend"),
            "' == 'mujoco' and '",
            LaunchConfiguration("scenario"),
            "' != 'dubins_race'",
        ]
    )
    is_dubins_mujoco = AndSubstitution(is_mujoco, is_dubins_race)

    scenario_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "scenarios",
            [LaunchConfiguration("scenario"), ".yml"],
        ]
    )

    omx_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "controllers", "open_manipulator_x.yml"]
    )

    default_perception_config = PathJoinSubstitution(
        [pkg_share, "config", "perception.yaml"]
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/sim.py"]),
        launch_arguments={
            "model": LaunchConfiguration("model"),
        }.items(),
        condition=is_gazebo,
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/mujoco.py"]),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "scenario": LaunchConfiguration("scenario"),
            "physics_mode": LaunchConfiguration("physics_mode"),
        }.items(),
        condition=IfCondition(is_standard_mujoco),
    )

    dubins_race_node = Node(
        package="fret",
        executable="dubins_race_node",
        name="dubins_race_node",
        output="screen",
        parameters=[
            {"scenario": LaunchConfiguration("scenario")},
            {"physics_mode": LaunchConfiguration("physics_mode")},
        ],
        condition=IfCondition(is_dubins_mujoco),
    )

    planner_node = Node(
        package="fret",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            scenario_config,
        ],
        condition=UnlessCondition(is_dubins_race),
    )

    scene_acquisition_node = Node(
        package="fret",
        executable="scene_acquisition_node",
        name="scene_acquisition_node",
        output="screen",
        parameters=[{"model": LaunchConfiguration("model")}],
        condition=UnlessCondition(is_dubins_race),
    )

    perception_bridge_default = Node(
        package="fret",
        executable="perception_bridge",
        name="perception_bridge_node",
        output="screen",
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "config_path": default_perception_config,
            }
        ],
        condition=UnlessCondition(is_dubins_race),
    )

    controller_arm = Node(
        package="fret",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            omx_controller_config,
        ],
        condition=IfCondition(is_omx_model),
    )

    return LaunchDescription(
        [
            model_arg,
            scenario_arg,
            backend_arg,
            physics_mode_arg,
            sim_launch,
            mujoco_launch,
            dubins_race_node,
            scene_acquisition_node,
            planner_node,
            perception_bridge_default,
            controller_arm,
        ]
    )
