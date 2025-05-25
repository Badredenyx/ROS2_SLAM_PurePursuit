# File: src/nav_controller/launch/multi_robot.launch.py

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_robots(context, *args, **kwargs):
    """
    Invoked at launch-time to read robot_count and spawn that many robots.
    Each robot uses the plain URDF (no xacro) and is offset in X by i meters.
    """
    spawn_actions = []

    # 1) Read robot_count from launch arguments
    count_str = context.launch_configurations.get("robot_count", "2")
    try:
        count = int(count_str)
    except ValueError:
        count = 2

    for i in range(count):
        ns = f"robot_{i}"

        # --- a) robot_state_publisher loads the URDF via 'cat' ---
        urdf_path = PathJoinSubstitution(
            [
                FindPackageShare("turtlebot3_description"),
                "urdf",
                "turtlebot3_burger.urdf",
            ]
        )
        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    # Load the raw URDF by cat’ing the file
                    "robot_description": Command(["cat ", urdf_path]),
                }
            ],
        )

        # --- b) spawn_entity to put this robot into Gazebo ---
        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=ns,
            arguments=[
                "-topic",
                "robot_description",  # read from rsp_node under namespace
                "-entity",
                ns,  # name in Gazebo: robot_0, robot_1, etc.
                "-x",
                str(i * 1.0),  # offset each robot i meters in X
                "-y",
                "0",
                "-z",
                "0.01",
            ],
            output="screen",
        )

        # --- c) nav_controller under the same namespace ---
        nav_control = Node(
            package="nav_controller",
            executable="control",
            name="navigation_control",
            namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": True}],
        )

        # Add these three Nodes directly (each has namespace=ns set)
        spawn_actions.extend([rsp_node, spawn_entity, nav_control])

    return spawn_actions


def generate_launch_description():
    # ────────────────────────────────────────────────────
    # 1) Force TurtleBot3 model variable (not strictly needed for plain URDF)
    # ────────────────────────────────────────────────────
    tb3_model_env = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL", value="burger"
    )

    # ────────────────────────────────────────────────────
    # 2) Launch Arguments: world_file, robot_count, rviz_config
    # ────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("turtlebot3_gazebo"),
                "worlds",
                "empty_world.world",
            ]
        ),
        description="Path to the TurtleBot3 empty world",
    )

    robot_count_arg = DeclareLaunchArgument(
        "robot_count",
        default_value="2",
        description="Number of TurtleBot3 instances to spawn",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("nav_controller"),
                "rviz",
                "multi_robot.rviz",
            ]
        ),
        description="Absolute path to RViz config for multi-robot view",
    )

    world_file = LaunchConfiguration("world_file")
    robot_count = LaunchConfiguration("robot_count")
    rviz_config = LaunchConfiguration("rviz_config")

    # ────────────────────────────────────────────────────
    # 3) Start Gazebo with the specified world
    # ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # ────────────────────────────────────────────────────
    # 4) Use OpaqueFunction to spawn N robots at launch-time
    # ────────────────────────────────────────────────────
    spawn_opaque = OpaqueFunction(function=spawn_robots)

    # ────────────────────────────────────────────────────
    # 5) Start a single RViz window
    # ────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # ────────────────────────────────────────────────────
    # 6) Assemble the LaunchDescription
    # ────────────────────────────────────────────────────
    ld = LaunchDescription()

    # A) Force TB3 model (helps xacro-based setups; harmless here)
    ld.add_action(tb3_model_env)

    # B) Declare arguments
    ld.add_action(world_arg)
    ld.add_action(robot_count_arg)
    ld.add_action(rviz_config_arg)

    # C) Launch Gazebo
    ld.add_action(gazebo)

    # D) Spawn robots via OpaqueFunction
    ld.add_action(spawn_opaque)

    # E) Launch RViz
    ld.add_action(rviz)

    return ld
