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
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        )
    ]

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
            "gz_args": ["-r ", " --verbose"],
        }.items(),
    )


    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Bridge world services for entity spawning/deletion
            "/world/tennis_court/create@ros_gz_interfaces/srv/SpawnEntity",
            "/world/tennis_court/remove@ros_gz_interfaces/srv/DeleteEntity",
            # Bridge wrench topic for applying forces to entities
            "/world/tennis_court/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
        ],
        remappings=[
            ("/world/tennis_court/create", "/spawn_entity"),
            ("/world/tennis_court/remove", "/delete_entity"),
            ("/world/tennis_court/wrench", "/apply_wrench"),
        ],
    )

    # Spawn robot in Gazebo
    gazebo_tf_publisher = Node(
        package="tbot_gazebo",
        executable="gazebo_tf_publisher_node",
        parameters=[
            {"gz_pose_topic": "/world/tennis_court/dynamic_pose/info"},
            {"base_frame_id": "base_footprint"},
            {"use_sim_time": use_sim_time},
        ],
    )


    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "tbot",
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
        gazebo_tf_publisher,
    ]
    
    # Launch all nodes
    return LaunchDescription(declared_arguments + nodes)
