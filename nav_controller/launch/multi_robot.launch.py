# File: launch/multi_robot.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_package'), 'worlds', 'default.world'
        ]),
        description='Path to the world file'
    )
    robot_count_arg = DeclareLaunchArgument(
        'robot_count', default_value='2',
        description='Number of robot instances to spawn'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_package'), 'rviz', 'multi_robot.rviz'
        ]),
        description='Path to the RViz config file'
    )

    world_file = LaunchConfiguration('world_file')
    robot_count = LaunchConfiguration('robot_count')
    rviz_config = LaunchConfiguration('rviz_config')

    # Start Gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn multiple robot instances
    spawn_groups = []
    for i in range(int(robot_count.perform({}))):
        ns = f"robot_{i}"
        # URDF param name assumed to be robot_description
        spawn = GroupAction([
            PushRosNamespace(ns),
            # spawn entity from gazebo_ros
            Node(
                package='gazebo_ros', executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', ns
                ],
                output='screen'
            ),
            # start the navigation control node in namespace
            Node(
                package='nav_controller', executable='control',
                name='navigation_control',
                namespace=ns,
                output='screen'
            ),
        ])
        spawn_groups.append(spawn)

    # RViz visualization
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        world_arg,
        robot_count_arg,
        rviz_config_arg,
        gazebo,
        *spawn_groups,
        rviz
    ])


