"""Unified FRET simulation launcher with model selection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from fret.launch.model import resolve_robot_model

logger = logging.get_logger("fret")


def _controller_parameters_for_model(model: str) -> dict:
    """Build controller node parameters that match the selected model."""
    return {
        "robot_model": model,
        "joint_states_topic": "/joint_states",
        "command_topic": "/joint_group_velocity_controller/commands",
        "base_frame": "base_link",
        "ee_frame": "end_effector_link",
        "command_rate_hz": 50.0,
        "joint_names": [
            "joint_arm_0",
            "joint_arm_1",
            "joint_extension",
            "joint_tool_rotate",
        ],
    }


def _launch_selected_model(context):
    model = LaunchConfiguration("model").perform(context)
    if model is None:
        raise ValueError("Missing 'model' launch argument.")

    world = LaunchConfiguration("world").perform(context)

    # Resolve robot model from all fallback sources
    fret_share = get_package_share_directory("fret")
    robot_description = resolve_robot_model(model, fret_share)

    # Derive the Gazebo world name from the world file path.
    # The SDF world name must match the file stem (e.g. arco_scenario.sdf →
    # world name "arco_scenario").  Gazebo Harmonic scopes all model topics
    # under /world/<world_name>/model/<model>/…, so we need this to bridge
    # joint states correctly.
    world_name = os.path.splitext(os.path.basename(world))[0]

    # Gazebo resolves model:// URIs by searching GZ_SIM_RESOURCE_PATH for a
    # directory whose name matches the first path component of the URI.
    # Meshes are installed to share/fret/meshes/, so Gazebo must find a
    # parent directory that contains a "fret/" subdirectory — i.e. share/.
    gz_resource_parent = os.path.dirname(fret_share)
    gz_env = os.environ.copy()
    existing = gz_env.get("GZ_SIM_RESOURCE_PATH", "")
    gz_env["GZ_SIM_RESOURCE_PATH"] = (
        f"{gz_resource_parent}:{existing}" if existing else gz_resource_parent
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_env["GZ_SIM_RESOURCE_PATH"]
        },
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            model,
            "-topic",
            "robot_description",
            "-z",
            "0.0",
            "-Y",
            "3.14159",
        ],
    )

    # Bridge Gazebo joint states directly to /joint_states.
    # Gazebo Harmonic's JointStatePublisher system plugin (injected via the
    # <gazebo> block in scara.xacro) publishes to the world-scoped topic
    # /world/<world_name>/model/<model>/joint_state (gz.msgs.Model).
    # The bridge remaps this to /joint_states for robot_state_publisher.
    #
    # The second bridge entry translates /world/<world_name>/pose/info
    # (gz.msgs.Pose_V) to tf2_msgs/TFMessage on /tf so that the
    # PerceptionBridgeNode can look up obstacle poses via TF.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            f"/world/{world_name}/model/{model}/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/world/{world_name}/pose/info"
            "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            (
                f"/world/{world_name}/model/{model}/joint_state",
                "/joint_states",
            ),
            (
                f"/world/{world_name}/pose/info",
                "/tf",
            ),
        ],
    )

    controller_node = Node(
        package="fret",
        executable="controller",
        output="screen",
        parameters=[
            _controller_parameters_for_model(model),
            {"use_sim_time": True},
        ],
    )

    perception_bridge_node = Node(
        package="fret",
        executable="perception_bridge.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return [
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        gz_bridge,
        controller_node,
        perception_bridge_node,
    ]


def generate_launch_description():
    """Generate the simulation launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("fret"), "worlds", "arco_scenario.sdf"]
                ),
                description=(
                    "Gazebo world file (absolute path or resource name). "
                    "Defaults to the FRET ARCO scenario world."
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="scara",
                description=(
                    "Robot model to simulate. "
                    "Must match a URDF/XACRO file in share/fret/urdf/."
                ),
            ),
            OpaqueFunction(function=_launch_selected_model),
        ]
    )
