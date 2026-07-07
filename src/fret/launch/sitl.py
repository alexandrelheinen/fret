"""Launch: full SITL pipeline (sim + planning + control + scene).

This is the main entry point for ``fretsim``.

Usage::

    ros2 launch fret sitl.py model:=scara scenario:=static_reach
    ros2 launch fret sitl.py model:=scara scenario:=straight_line
    ros2 launch fret sitl.py model:=scara scenario:=arc
    ros2 launch fret sitl.py scenario:=ppp_warehouse model:=ppp backend:=mujoco

Arguments:
    model (str, default: scara)
        Robot model name.
    scenario (str, default: static_reach)
        Scenario YAML stem (e.g. ``static_reach``).  Must match a file under
        ``fret/config/scenarios/<scenario>.yml``.
    backend (str, default: gazebo)
        Simulator backend: ``gazebo`` (SCARA regression) or ``mujoco`` (v1.0 PPP).

Scenario behaviour:
    straight_line — Milestone 1.  Launches the ``straight_line_injector``
        node instead of ``planner_node``.  The injector publishes a
        Cartesian straight-line trajectory once; the controller tracks it.
    arc — Launches the ``arc_injector`` node instead of ``planner_node``.
        The injector publishes a circular arc trajectory in Cartesian space
        once; the controller tracks it.  Produces a visible arc in Gazebo.
    ppp_warehouse — v1.0 PPP warehouse pick-and-place with ``backend:=mujoco``.
    static_reach (and others) — standard SITL with ``planner_node`` and
        ``scene_acquisition_node``.  The planner auto-triggers at startup
        and publishes the resulting trajectory; the controller executes it.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import (
    AndCondition,
    IfCondition,
    LaunchConfigurationEquals,
    UnlessCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the SITL pipeline."""
    pkg_share = FindPackageShare("fret")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="scara",
        description="Robot model name",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="static_reach",
        description="Scenario YAML stem (config/scenarios/<scenario>.yml)",
    )
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="gazebo",
        description="Simulator backend: gazebo | mujoco",
    )

    is_mujoco = EqualsSubstitution(LaunchConfiguration("backend"), "mujoco")
    is_gazebo = UnlessCondition(is_mujoco)
    is_ppp_model = LaunchConfigurationEquals("model", "ppp")

    is_straight_line = EqualsSubstitution(
        LaunchConfiguration("scenario"), "straight_line"
    )
    is_arc = EqualsSubstitution(LaunchConfiguration("scenario"), "arc")
    is_injector_scenario = OrSubstitution(is_straight_line, is_arc)

    scenario_config = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "scenarios",
            [LaunchConfiguration("scenario"), ".yml"],
        ]
    )

    scara_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "controllers", "jacobian.yml"]
    )
    ppp_controller_config = PathJoinSubstitution(
        [pkg_share, "config", "controllers", "ppp.yml"]
    )

    default_perception_config = PathJoinSubstitution(
        [pkg_share, "config", "perception.yaml"]
    )
    ppp_perception_config = PathJoinSubstitution(
        [pkg_share, "config", "perception_ppp_warehouse.yaml"]
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/sim.py"]),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "world": "arco_scenario.sdf",
        }.items(),
        condition=is_gazebo,
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/mujoco.py"]),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "scenario": LaunchConfiguration("scenario"),
        }.items(),
        condition=IfCondition(is_mujoco),
    )

    straight_line_injector = Node(
        package="fret",
        executable="straight_line_injector",
        name="straight_line_injector",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            scenario_config,
        ],
        condition=IfCondition(is_straight_line),
    )

    arc_injector = Node(
        package="fret",
        executable="arc_injector",
        name="arc_injector",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            scenario_config,
        ],
        condition=IfCondition(is_arc),
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
        condition=UnlessCondition(is_injector_scenario),
    )

    scene_acquisition_node = Node(
        package="fret",
        executable="scene_acquisition_node",
        name="scene_acquisition_node",
        output="screen",
        parameters=[{"model": LaunchConfiguration("model")}],
        condition=UnlessCondition(is_injector_scenario),
    )

    is_ppp_warehouse = LaunchConfigurationEquals("scenario", "ppp_warehouse")
    is_standard_planner = UnlessCondition(is_injector_scenario)

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
        condition=IfCondition(
            AndCondition(
                is_standard_planner,
                UnlessCondition(is_ppp_warehouse),
            )
        ),
    )

    perception_bridge_ppp = Node(
        package="fret",
        executable="perception_bridge",
        name="perception_bridge_node",
        output="screen",
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "config_path": ppp_perception_config,
            }
        ],
        condition=IfCondition(
            AndCondition(is_standard_planner, is_ppp_warehouse)
        ),
    )

    controller_scara = Node(
        package="fret",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            scara_controller_config,
        ],
        remappings=[
            ("/joint_commands", "/joint_group_velocity_controller/commands")
        ],
        condition=UnlessCondition(is_ppp_model),
    )

    controller_ppp = Node(
        package="fret",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"model": LaunchConfiguration("model")},
            ppp_controller_config,
        ],
        condition=IfCondition(is_ppp_model),
    )

    viz_node = Node(
        package="fret",
        executable="viz_node",
        name="viz_node",
        output="screen",
        parameters=[{"model": LaunchConfiguration("model")}],
        condition=is_gazebo,
    )

    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", [LaunchConfiguration("model"), ".rviz"]]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=is_gazebo,
    )

    return LaunchDescription(
        [
            model_arg,
            scenario_arg,
            backend_arg,
            sim_launch,
            mujoco_launch,
            straight_line_injector,
            arc_injector,
            scene_acquisition_node,
            planner_node,
            perception_bridge_default,
            perception_bridge_ppp,
            controller_scara,
            controller_ppp,
            viz_node,
            rviz2,
        ]
    )
