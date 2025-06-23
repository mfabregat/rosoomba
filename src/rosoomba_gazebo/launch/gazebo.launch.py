#!/usr/bin/env python3
# Copyright (c) 2023 TBot Robot
# This file is part of the TBot robot project

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    use_sim_time = True

    # Declare arguments with default values
    declared_arguments = [        
        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),
        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),
        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.1',
            description='Robot spawn position in Z axis'
        ),
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),
        DeclareLaunchArgument(
            name="world_name",
            default_value="empty.world",
            description="Gazebo world name",
        ),
    ]

    world_path = PathJoinSubstitution(
        [
            FindPackageShare("rosoomba_gazebo"),
            "worlds",
            LaunchConfiguration("world_name"),
        ]
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path, " --verbose"],
        }.items(),
    )


    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rosoomba",
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
            ]
    )

    nodes = [
        gazebo,
        gazebo_bridge,
        spawn_entity,
    ]
    
    # Launch all nodes
    return LaunchDescription(declared_arguments + nodes)
