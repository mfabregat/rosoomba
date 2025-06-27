#!/usr/bin/env python3
# Copyright (c) 2023 rosoomba Rrosoomba
# This file is part of the rosoomba rrosoomba project

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():

    # Declare arguments with default values
    declared_arguments = [        
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run rviz'
        ),

    ]

    
    description_launch_file = PathJoinSubstitution([
            FindPackageShare("rosoomba_description"), "launch", "description.launch.py"
            ])

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'gui': 'false',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    

    gazebo_launch_file = PathJoinSubstitution([
            FindPackageShare("rosoomba_gazebo"), "launch", "gazebo.launch.py"
            ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    bridge_params_file = PathJoinSubstitution([
        FindPackageShare("rosoomba_bringup"),'config','gz_bridge.yaml'
        ])
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', bridge_params_file],
        ]
    )

    # Joy teleop 

    joy_teleop_launch_file = PathJoinSubstitution([
            FindPackageShare("rosoomba_bringup"), "launch", "joy_teleop.launch.py"
            ])

    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_teleop_launch_file),
    )


    nodes = [
        description_launch,
        gazebo_launch,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge, 
        joy_teleop_launch,
    ]
    
    # Launch all nodes
    return LaunchDescription(declared_arguments + nodes)
