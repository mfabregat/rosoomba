#!/usr/bin/env python3
# Copyright (c) 2023 rosoomba Rrosoomba
# This file is part of the rosoomba rrosoomba project

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare


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
            'publish_joints': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    

    gazebo_launch_file = PathJoinSubstitution([
            FindPackageShare("rosoomba_gazebo"), "launch", "gazebo.launch.py"
            ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
    )





    nodes = [
        description_launch,
        gazebo_launch,
    ]
    
    # Launch all nodes
    return LaunchDescription(declared_arguments + nodes)
